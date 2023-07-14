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
        
        # pendelum
        if len(args)==6:
            mass, coorX, coorY, Qmax, Len,tailLen = args
            self.Qmax= Qmax
            self.Q=Qmax* math.pi / 180
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
                    
    def cleanDrawPendelum(self,originP,xyL,t,realOrStep):
        def drawPathPendelum(space,xyL):
            a = [0,0,0]
            for i in xyL:
                a[2]+=255//len(xyL)
                x,y=i
                if x > 0 and x < space.shape[0] and y > 0 and  y < space.shape[1]:
                    space[int(y),int(x)]=a
        
        x=self.coorX
        y=self.coorY
        x=space.shape[0]//2+x
        T=self.T
        L=self.Len
        Q = self.Q
        
        xyL.append((x,y))
        
        fark=len(xyL)-self.tailLen
        try:
            if fark>0:
                for i in range(fark):
                    xyL.remove(xyL[i])
        except:
            pass
            
        
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
        cv2.line(space,originP,(originP[0],(originP[1]+1)*self.Len),(100,60,60),1)
        cv2.line(space,originP,(x,y),(255,255,255),3)
        cv2.circle(space,(x,y),5,(0,0,255),self.mass)
        cv2.putText(space,"100m",(20,int(space.shape[0]-5)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0),2)
        cv2.putText(space,f"T(period)=  {T:.3f}",(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(space,f"Q(Angle) =  {Qdegree:.3f}",(20,40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(space,f"a(accel.)=  {a:.3f}",(20,60),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(space,f"t(time)  =  {t:.3f}",(20,80),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        if realOrStep==1:
            cv2.putText(space,"Real Time ON",(20,100),cv2.FONT_HERSHEY_SIMPLEX,0.5,(10,10,225),1)
        elif realOrStep==0:
            cv2.putText(space,"Time Step ON",(20,100),cv2.FONT_HERSHEY_SIMPLEX,0.5,(10,225,10),1)
        drawPathPendelum(space,xyL)
    
    def pendelumAnimation(self):
         
    
        def empty(a):
            pass    
        def Qchange(a):
            Qs = a * math.pi / 180  
            self.Qmax=Qs
        cv2.namedWindow("TrackBars")
        cv2.resizeWindow("TrackBars",640,360)
        cv2.createTrackbar("mass","TrackBars",1,25,empty)
        cv2.createTrackbar("Q","TrackBars",30,90,Qchange)
        cv2.createTrackbar("g","TrackBars",98,300,empty)
        cv2.createTrackbar("Len","TrackBars",400,1000,empty)
        cv2.createTrackbar("tailLen","TrackBars",200,800,empty)
        cv2.createTrackbar("TimeStep","TrackBars",5,100,empty)
        cv2.createTrackbar("RealTime","TrackBars",0,1,empty)
        cv2.createTrackbar("Open_Close","TrackBars",1,1,empty)
        t=self.t
        timeStep= Body.timeStep
        Close=1
        while Close==1:
            
            xyL=[]
            
            originP=(space.shape[0]//2,0)
            
            while True:
                start = time.time()
                
                self.mass = cv2.getTrackbarPos("mass","TrackBars")              
                physic.g = cv2.getTrackbarPos("g","TrackBars")/10
                self.Len = cv2.getTrackbarPos("Len","TrackBars")
                self.tailLen = cv2.getTrackbarPos("tailLen","TrackBars")
                Close = cv2.getTrackbarPos("Open_Close","TrackBars")
                Body.timeStep = cv2.getTrackbarPos("TimeStep","TrackBars")/100
                realOrStep = cv2.getTrackbarPos("RealTime","TrackBars")
                
                if Body.timeStep==0:
                    Body.timeStep=0.01
                if self.mass==0:
                    self.mass=1 
                if Close==0:
                    break  
                if physic.g==0:
                    physic.g=1
                if self.Len < 4:
                    self.Len=5
                if self.tailLen<10:
                    self.tailLen=10 
             
                self.T=physic.findPeriodPendelum(self.Len)
                self=physic.XYdisplacementPendelum(self,self.t,self.Len)
                self.cleanDrawPendelum(originP,xyL,self.t,realOrStep)
                    
                cv2.imshow("space",space)                
                
                end = time.time()
                if realOrStep==1:
                    self.t+=0.04
                    spentTime= end-start # spent time in while loop
                    print(spentTime)
                    if 0.04-spentTime<=0:
                        time.sleep(0)
                    else:
                        time.sleep(0.04-spentTime)   
                elif realOrStep==0:
                    self.t+=Body.timeStep
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
            time.sleep(timeStep)#sil bunu
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
# m = mass, (coorX,locaitonY) = instant coors
#-----  Pendelum ------------
# Qmax=starting degree for pendelum, T = period L = rope lenght 


#---Pendulum variables---
m=5
Qmax, Len, tailLen = 30, 400, 200
coorX,coorY=200,900
#---Double Pendulum-------
m1,m2=2,2
L1,L2=200,200
Q1,Q2=90,90
Q1V,Q2V=0,0


while True: 
    choice = "1"
    #choice = input("FreeFall (1) or Pendelum (2) or Double Pendelum(3)= ")

    if choice=="1":
        b1 = Body(m, coorX,coorY,Qmax,Len,tailLen)
        break
    elif choice=="2":
        b1 = Body(m1,m2,L1,L2,Q1,Q2,Q1V,Q2V,tailLen)
        break
    else:
        continue