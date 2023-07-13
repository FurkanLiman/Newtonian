import time
import cv2
import numpy as np
import math
import physic

space = np.zeros([1000,1000,3],dtype=np.uint8)

g=physic.g

class Body:
    t=0
    timeStep= 0.05
    def __init__(self,*args):
        # free fall
        if len(args)==5:
            mass, coorX, coorY, vb,vbAngle = args
            vbAngle=-vbAngle
            self.mass= mass
            self.coorX = coorX
            self.coorY = coorY
            self.vb=vb
            self.vbX,self.vbY= self.vbAngleCalculate(vb,vbAngle)
            self.vx, self.vy= 0,0
            self.freeFallAnimation()    
        # pendelum
        elif len(args)==6:
            mass, coorX, coorY, Qmax, Len,tailLen = args
            self.Qmax= Qmax
            self.Q=Qmax
            self.T = physic.findPeriodPendelum(Len)
            self.Len=Len
            self.tailLen=tailLen
            self.mass= mass
            self.coorX = coorX
            self.coorY = coorY
            self.pendelumAnimation()
        # double pendelum
        elif len(args)==9:
            m1,m2,L1,L2,Q1,Q2,Q1V,Q2V,tailLen = args
            
            Q1 = Q1 * math.pi / 180
            Q2 = Q2 * math.pi / 180
            
            x1 = L1 * math.sin(Q1)
            y1 = L1 * math.cos(Q1)

            x2 = x1 + L2 * math.sin(Q2)
            y2 = y1 + L2 * math.cos(Q2)
            
            self.m1=m1
            self.m2=m2
            self.L1=L1
            self.L2=L2
            self.Q1=Q1
            self.Q2=Q2
            self.Q1V=Q1V
            self.Q2V=Q2V
            self.x1=x1
            self.y1=y1
            self.x2=x2
            self.y2=y2
            self.tailLen=tailLen
            self.doublePendelumAnimation()
            
    def cleanDrawFreeFall(self,space,pathXY,t):
        def drawPathFreeFall(space,pathXY):
            for i in pathXY:
                x,y=i
                x=int(x)
                y=int(y)
                if y<space.shape[0] and x<space.shape[1]:
                    space[y,x]=255    
        
        space[:,:]=0
        
        V=int(math.sqrt((self.vx**2)+(self.vy**2)))
        xint,yint,Vxint,Vyint = int(self.coorX),int(self.coorY),int(self.vx),int(self.vy)
        for i in range(space.shape[1]//100):
            space[(2*i)*100:(2*i+1)*100,int(space.shape[0]*0.98):space.shape[0]] = 255
            space[(2*i+1)*100:(2*i+3)*100,int(space.shape[0]*0.98):space.shape[0]] = 125
            space[int(space.shape[0]*0.98):space.shape[0],(2*i)*100:(2*i+1)*100] = 255
            space[int(space.shape[0]*0.98):space.shape[0],(2*i+1)*100:(2*i+3)*100] = 125
        cv2.putText(space,"100m",(20,int(space.shape[0]-5)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0),2)
        cv2.putText(space,f"y={self.coorY:.3f} meter",(15,15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255))
        cv2.putText(space,f"V={V:.3f} m/s",(15,30),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255))
        cv2.putText(space,f"t={t:.3f} s",(15,45),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255))
        cv2.arrowedLine(space,(xint,yint),(xint+Vxint,yint+Vyint),(255,0,0),3) # total-> blue
        cv2.arrowedLine(space,(xint,yint),(xint,yint+Vyint),(0,255,0),3) # y arrow -> green
        cv2.arrowedLine(space,(xint,yint),(xint+Vxint,yint),(0,0,255),3) # x arrow -> red
        cv2.circle(space,(xint,yint),5,(0,0,255),self.mass)
        
        drawPathFreeFall(space,pathXY)  
    
    def vbAngleCalculate(self,vb,vbAngle):
        vbRadianA = vbAngle * math.pi / 180
        vbX=vb*math.cos(vbRadianA)
        vbY=vb*math.sin(vbRadianA)
        return vbX,vbY 
    
    def freeFallAnimation(self):    
    
        t= Body.t
        timeStep= Body.timeStep
        pathXY=physic.drawRouteFreeFall(self,t,space)
        
        while True:
            start = time.time()
            
            cv2.imshow("space",space)
            
            if self.coorY>=space.shape[0]*0.98-r:
                v=0
                break

            self =physic.XYdisplacementFreeFall(self,t)
            self.cleanDrawFreeFall(space,pathXY,t)       
            
            t+=timeStep
            end = time.time()
            spentTime= end-start # spent time in while loop
            
            if timeStep-spentTime<=0:
                time.sleep(0)
            else:
                time.sleep(timeStep-spentTime)
            
            if cv2.waitKey(1) == ord('q'):
                break


        while True:
            cv2.imshow("space",space)
            if cv2.waitKey(1) == ord('q'):
                break

        cv2.waitKey(1)
        cv2.destroyAllWindows()
          
    def cleanDrawPendelum(self,originP,xyL,t):
        def drawPathPendelum(space,xyL):
            for i in xyL:
                x,y=i
                space[int(y),int(x)]=255
        
        x=self.coorX
        y=self.coorY
        x=space.shape[0]//2+x
        T=self.T
        L=self.Len
        Q = self.Q
        
        xyL.append((x,y))
        if len(xyL)>self.tailLen:
            xyL.remove(xyL[0])
            
        
        Qdegree = Q*(180/math.pi)
        a = g*math.tan(Q)
        x=int(x)
        y=int(y)
        space[:,:] = 0
        for i in range(space.shape[1]//100):
            space[(2*i)*100:(2*i+1)*100,int(space.shape[0]*0.98):space.shape[0]] = 255
            space[(2*i+1)*100:(2*i+3)*100,int(space.shape[0]*0.98):space.shape[0]] = 125
            space[int(space.shape[0]*0.98):space.shape[0],(2*i)*100:(2*i+1)*100] = 255
            space[int(space.shape[0]*0.98):space.shape[0],(2*i+1)*100:(2*i+3)*100] = 125
        cv2.line(space,originP,(originP[0],(originP[1]+1)*L),(125,125,125),1)
        cv2.line(space,originP,(x,y),(255,255,255),3)
        cv2.circle(space,(x,y),5,(0,0,255),self.mass)
        cv2.putText(space,"100m",(20,int(space.shape[0]-5)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0),2)
        cv2.putText(space,f"T(period)=  {T:.3f}",(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(space,f"Q(Angle) =  {Qdegree:.3f}",(20,40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(space,f"a(accel.)=  {a:.3f}",(20,60),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(space,f"t(time)  =  {t:.3f}",(20,80),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        drawPathPendelum(space,xyL)
     
    def pendelumAnimation(self):
        
        Qmaks=self.Qmax
        t=Body.t
        timeStep= Body.timeStep
       
        xyL=[]
        
        originP=(space.shape[0]//2,0)
        Qmaks = Qmaks * math.pi / 180
        
        while True:
            start = time.time()
            
            cv2.imshow("space",space)

            self.T=physic.findPeriodPendelum(self.Len)
            self=physic.XYdisplacementPendelum(self,Qmaks,t,self.Len)
            self.cleanDrawPendelum(originP,xyL,t)
            
            t+=timeStep
            end = time.time()
            spentTime= end-start # spent time in while loop
            
            if timeStep-spentTime<=0:
                time.sleep(0)
            else:
                time.sleep(timeStep-spentTime)   
            
            if cv2.waitKey(1) == ord('q'):
                break


        while True:
            cv2.imshow("space",space)
            if cv2.waitKey(1) == ord('q'):
                break

        cv2.waitKey(1)
        cv2.destroyAllWindows()

    def cleanDrawDoublePendelum(self,space,xyL):
        def drawPathPendelum(space,xyL):
            for i in xyL:
                x,y=i
                space[int(y),int(x)]=255    
    
        space[:,:]=0

        drawPathPendelum(space,xyL)
        
        for i in range(space.shape[1]//100):
            space[(2*i)*100:(2*i+1)*100,int(space.shape[0]*0.98):space.shape[0]] = 255
            space[(2*i+1)*100:(2*i+3)*100,int(space.shape[0]*0.98):space.shape[0]] = 125
            space[int(space.shape[0]*0.98):space.shape[0],(2*i)*100:(2*i+1)*100] = 255
            space[int(space.shape[0]*0.98):space.shape[0],(2*i+1)*100:(2*i+3)*100] = 125
        
        midX = space.shape[0]//2
        midY = space.shape[1]//2
        
        x1 = self.x1
        y1 = self.y1
        x2 = self.x2
        y2 = self.y2
        
        x1,y1,x2,y2=int(x1)+midX,int(y1)+midY,int(x2)+midX,int(y2)+midY
        
        Q1degree= self.Q1*(180/math.pi)
        Q2degree= self.Q2*(180/math.pi)
        
        cv2.line(space,(500,500),(x1,y1),(200,200,255),1)
        cv2.line(space,(x1,y1),(x2,y2),(200,255,200),2)  
        cv2.circle(space,(x1,y1),5,(255,0,0),self.m1)
        cv2.circle(space,(x2,y2),5,(0,0,255),self.m2)
        cv2.putText(space,"100m",(20,int(space.shape[0]-5)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0),2)
        cv2.putText(space,f"Q1=  {Q1degree:.3f}",(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(space,f"Q2=  {Q2degree:.3f}",(20,40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(space,f"Q1V=  {self.Q1V:.3f}",(20,60),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(space,f"Q2V=  {self.Q2V:.3f}",(20,80),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
                 
    def doublePendelumAnimation(self):
        t=Body.t
        tailLen=self.tailLen
        
        m1=self.m1
        m2=self.m2
        L1=self.L1
        L2=self.L2
        Q1=self.Q1
        Q2=self.Q2
        Q1V=self.Q1V
        Q2V=self.Q2V
        
        timeStep=self.timeStep

        Q1dd,Q2dd=0,0
        
        xyL=[]

        while True:
            start = time.time()
            
            cv2.imshow("space",space)
            
            self=physic.XYdisplacementDoublePendelum(self)

            self.cleanDrawDoublePendelum(space,xyL)
            
            xyL.append((self.x2+space.shape[0]//2,self.y2+space.shape[1]//2))
            
            if len(xyL)>=tailLen:
                xyL.remove(xyL[0])
            
            
            t+=timeStep
            #time.sleep(timeStep)#sil bunu
            end = time.time()
            spentTime= end-start # spent time in while loop
            """
            if timeStep-spentTime<=0:
                time.sleep(0)
            else:
                time.sleep(timeStep-spentTime)   
            """
            if cv2.waitKey(1) == ord('q'):
                break
            

        while True:
            cv2.imshow("space",space)
            if cv2.waitKey(1) == ord('q'):
                break

        cv2.waitKey(1)
        cv2.destroyAllWindows()
        
    
#------ Common Things -------
# m = mass, r = radius, (coorX,locaitonY) = instant coors
#------ Free Fall -----------
# Vb = initial velocity, vbAngle = initial velocity angle
#-----  Pendelum ------------
# Qmax=starting degree for pendelum, T = period L = rope lenght 

#---Common variables---
m,r=5,10
#---Pendelum variables---
Qmax, Len, tailLen = 30, 200, 200
coorX,coorY=300,800
#---double Pen-------
m1,m2=2,2
L1,L2=200,200
Q1,Q2=90,90
Q1V,Q2V=0,0
#---Free fall variables---
vb,vbAngle=80,45


while True: 
    choice = "3"
    #choice = input("FreeFall (1) or Pendelum (2) or Double Pendelum(3)= ")
    if choice == "1":
        b1 = Body(m, coorX,coorY,vb,vbAngle)
        break
    elif choice=="2":
        b1 = Body(m, coorX,coorY,Qmax,Len,tailLen)
        break
    elif choice=="3":
        b1 = Body(m1,m2,L1,L2,Q1,Q2,Q1V,Q2V,tailLen)
        break
    else:
        continue