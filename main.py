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
        if len(args)==6:
            mass, radius, coorX, coorY, vb,vbAngle = args
            vbAngle=-vbAngle
            self.mass= mass
            self.volume= (4/3)*(math.pi*math.pow(radius,3))
            self.coorX = coorX
            self.coorY = coorY
            self.vb=vb
            self.vbX,self.vbY= self.vbAngleCalculate(vb,vbAngle)
            self.freeFallAnimation()
            
        # pendelum
        elif len(args)==7:
            mass, radius, coorX, coorY, Qmax, Len,tailLen = args
            self.Qmax= Qmax
            self.T = physic.findPeriodPendelum(Len)
            self.Len=Len
            self.tailLen=tailLen
            self.mass= mass
            self.volume= (4/3)*(math.pi*math.pow(radius,3))
            self.coorX = coorX
            self.coorY = coorY
            self.pendelumAnimation()
    
    def cleanDrawFreeFall(space,pathXY):
        def drawPathFreeFall(space,pathXY):
            for i in pathXY:
                x,y=i
                x=int(x)
                y=int(y)
                if y<space.shape[0] and x<space.shape[1]:
                    space[y,x]=255    
        
        space[:,:]=0
        
        for i in range(space.shape[1]//100):
            space[(2*i)*100:(2*i+1)*100,int(space.shape[0]*0.98):space.shape[0]] = 255
            space[(2*i+1)*100:(2*i+3)*100,int(space.shape[0]*0.98):space.shape[0]] = 125
            space[int(space.shape[0]*0.98):space.shape[0],(2*i)*100:(2*i+1)*100] = 255
            space[int(space.shape[0]*0.98):space.shape[0],(2*i+1)*100:(2*i+3)*100] = 125
        cv2.putText(space,"100m",(20,int(space.shape[0]-5)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0),2)
        drawPathFreeFall(space,pathXY)  
    
    def vbAngleCalculate(self,vb,vbAngle):
        vbRadianA = vbAngle * math.pi / 180
        vbX=vb*math.cos(vbRadianA)
        vbY=vb*math.sin(vbRadianA)
        return vbX,vbY 
    
    def freeFallAnimation(self):    
        v=0
        vx=0
        vy=0
        t= Body.t
        x=self.coorX
        y=self.coorY
        timeStep= Body.timeStep
        pathXY=physic.drawRoute(self,t,timeStep,space)
        while True:
            start = time.time()
            
            cv2.imshow("space",space)

            if self.coorY>=space.shape[0]*0.98-r:
                v=0
                break

            Body.cleanDrawFreeFall(space,pathXY)

            x,y,vy,vx=physic.XYdisplacementFreeFall(self,t,timeStep)
            
            #tam sayıları kaldır
            xint=int(x)
            yint=int(y)
            Vxint=int(vx)
            Vyint=int(vy)
            vint=int(math.sqrt((vx**2)+(vy**2)))
            

            self.coorX=x
            self.coorY=y

            cv2.putText(space,f"y={yint} meter",(15,15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255))
            cv2.putText(space,f"V={vint} m/s",(15,30),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255))
            cv2.putText(space,f"t={t} s",(15,45),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255))
            cv2.arrowedLine(space,(xint,yint),(xint+Vxint,yint+Vyint),(255,0,0),3) # total-> blue
            cv2.arrowedLine(space,(xint,yint),(xint,yint+Vyint),(0,255,0),3) # y arrow -> green
            cv2.arrowedLine(space,(xint,yint),(xint+Vxint,yint),(0,0,255),3) # x arrow -> red
            cv2.circle(space,(xint,yint),5,(0,0,255),r)
            
            
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
          
    def cleanDrawPendelum(self,originP,xyL,Q,t):
        def drawPathPendelum(space,xyL):
            for i in xyL:
                x,y=i
                space[int(y),int(x)]=255
        
        x=self.coorX
        y=self.coorY
        T=self.T
        L=self.Len
               
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
        xS=space.shape[0]//2
        x,y=0,0
        xyL=[]
        
        originP=(space.shape[0]//2,0)
        Qmaks = Qmaks * math.pi / 180
        
        while True:
            start = time.time()
            
            cv2.imshow("space",space)


            Q,x,y=physic.XYdisplacementPendelum(self.T,Qmaks,t,self.Len)
            x=xS+x
            self.coorX=x
            self.coorY=y
            
            xyL.append((x,y))
            if len(xyL)>tailLen:
                xyL.remove(xyL[0])
            
            Body.cleanDrawPendelum(self,originP,xyL,Q,t)
            
            
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

#------ Common Things -------
# m = mass, r = radius, (coorX,locaitonY) = instant coors
#------ Free Fall -----------
# Vb = initial velocity, vbAngle = initial velocity angle
#-----  Pendelum ------------
# Qmax=starting degree for pendelum, T = period L = rope lenght 

#---Common variables---
m,r=1,10
#---Pendelum variables---
Qmax, Len, tailLen = 30, 500, 200
#---Free fall variables---
vb,vbAngle=50,45
coorX,coorY=00,500

while True: 
    #choice = "2"
    choice = input("FreeFall (1) or Pendelum (2)= ")
    if choice == "1":
        b1 = Body(m, r, coorX,coorY,vb,vbAngle)
        break
    elif choice=="2":
        b1 = Body(m,r,coorX,coorY,Qmax,Len,tailLen)
        break
    else:
        continue