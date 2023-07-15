import time
import cv2
import numpy as np
import math
import physic

space = np.zeros([1000,1000,3],dtype=np.uint8)
space1 = np.zeros([1000,1000,3],dtype=np.uint8)
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
                if self.tailLen!=0:
                    if len(xyL)<255:
                        a[2]+=255//len(xyL)
                    elif len(xyL)>=255 and len(xyL)<510:
                        a[2]+=(510//len(xyL))/2
                    elif len(xyL)>=510 and len(xyL)<765:
                        a[2]+=(765//len(xyL))/3
                    elif len(xyL)>=765 and len(xyL)<1020:
                        a[2]+=(1020//len(xyL))/4
                    elif len(xyL)>=1020 and len(xyL)<1275:
                        a[2]+=(1275//len(xyL))/5
                else: 
                    a[2]=255
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
            if self.tailLen!=0:
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
            cv2.putText(space,"Real Time ON",(20,100),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,10,10),1)
        elif realOrStep==0:
            cv2.putText(space,"Time Step ON",(20,100),cv2.FONT_HERSHEY_SIMPLEX,0.5,(10,225,10),1)
        elif realOrStep ==2 :
            cv2.putText(space,"Stopped",(20,100),cv2.FONT_HERSHEY_SIMPLEX,0.5,(10,10,255),1)
        if self.tailLen==0:
            cv2.putText(space1,"Full Painting",(20,120),cv2.FONT_HERSHEY_SIMPLEX,0.5,(10,10,255),1)
        drawPathPendelum(space,xyL)
    
    def pendelumAnimation(self):
         
    
        def empty(a):
            pass    
        def Qchange(a):
            Qs = a * math.pi / 180  
            self.Qmax=Qs
        def rSS(a):
            if a==2:
                self.timeStep=0
        cv2.namedWindow("TrackBars")
        cv2.resizeWindow("TrackBars",640,360)
        cv2.createTrackbar("Mass","TrackBars",1,25,empty)
        cv2.createTrackbar("Q","TrackBars",30,90,Qchange)
        cv2.createTrackbar("gravity*10","TrackBars",98,300,empty)
        cv2.createTrackbar("Length","TrackBars",400,1000,empty)
        cv2.createTrackbar("TailLength","TrackBars",200,1274,empty)
        cv2.createTrackbar("TimeStep","TrackBars",5,100,empty)
        cv2.createTrackbar("RealTime","TrackBars",2,2,rSS)
        cv2.createTrackbar("StopApp","TrackBars",1,1,empty)
        cv2.setTrackbarMin("TimeStep","TrackBars",1)
        cv2.setTrackbarMin("Mass","TrackBars",1)
        cv2.setTrackbarMin("gravity*10","TrackBars",1)
        cv2.setTrackbarMin("Length","TrackBars",5)
        cv2.setTrackbarMin("TailLength","TrackBars",0)
        t=self.t
        timeStep= Body.timeStep
        Close=1
        while Close==1:
            
            xyL=[]
            
            originP=(space.shape[0]//2,0)
            
            while True:
                start = time.time()
                
                self.mass = cv2.getTrackbarPos("Mass","TrackBars")              
                physic.g = cv2.getTrackbarPos("gravity*10","TrackBars")/10
                self.Len = cv2.getTrackbarPos("Length","TrackBars")
                self.tailLen = cv2.getTrackbarPos("TailLength","TrackBars")
                Close = cv2.getTrackbarPos("StopApp","TrackBars")
                Body.timeStep = cv2.getTrackbarPos("TimeStep","TrackBars")/100
                realOrStep = cv2.getTrackbarPos("RealTime","TrackBars")
                 
                if Close==0:
                    break  
                
                self.T=physic.findPeriodPendelum(self.Len)
                self=physic.XYdisplacementPendelum(self,self.t,self.Len)
                self.cleanDrawPendelum(originP,xyL,self.t,realOrStep)
                    
                cv2.imshow("space",space)                
                
                end = time.time()
                if realOrStep==1:
                    self.t+=0.04
                    spentTime= end-start # spent time in while loop
                    
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

    def cleanDrawDoublePendelum(self,space1,xyL,realOrStep):
        def drawPathPendelum(space1,xyL):
            a = [0,0,0]
            for i in xyL:
                if self.tailLen!=0:
                    if len(xyL)<255:
                        a[2]+=255//len(xyL)
                    elif len(xyL)>=255 and len(xyL)<510:
                        a[2]+=(510//len(xyL))/2
                    elif len(xyL)>=510 and len(xyL)<765:
                        a[2]+=(765//len(xyL))/3
                    elif len(xyL)>=765 and len(xyL)<1020:
                        a[2]+=(1020//len(xyL))/4
                    elif len(xyL)>=1020 and len(xyL)<1275:
                        a[2]+=(1275//len(xyL))/5
                else: 
                    a[2]=255
                x,y=i
                if x > 0 and x < space1.shape[0] and y > 0 and  y < space1.shape[1]:
                    space1[int(y),int(x)]=a
    
        space1[:,:]=0

        
        for i in range(space1.shape[1]//100):
            space1[(2*i)*100:(2*i+1)*100,int(space1.shape[0]*0.98):space1.shape[0]] = 255
            space1[(2*i+1)*100:(2*i+3)*100,int(space1.shape[0]*0.98):space1.shape[0]] = 125
            space1[int(space1.shape[0]*0.98):space1.shape[0],(2*i)*100:(2*i+1)*100] = 255
            space1[int(space1.shape[0]*0.98):space1.shape[0],(2*i+1)*100:(2*i+3)*100] = 125
        
        midX = space1.shape[0]//2
        midY = space1.shape[1]//2
        
        x1 = self.x1
        y1 = self.y1
        x2 = self.x2
        y2 = self.y2
        
        x1,y1,x2,y2=int(x1)+midX,int(y1)+midY,int(x2)+midX,int(y2)+midY
        
        Q1degree= self.Q1*(180/math.pi)
        Q2degree= self.Q2*(180/math.pi)
        
        xyL.append((self.x2+space1.shape[0]//2,self.y2+space1.shape[1]//2))
        fark=len(xyL)-self.tailLen
        try:
            if self.tailLen!=0:
                if fark>0:
                    for i in range(fark):
                        xyL.remove(xyL[i])
        except:
            pass
        
        cv2.line(space1,(500,500),(x1,y1),(200,200,255),1)
        cv2.line(space1,(x1,y1),(x2,y2),(200,255,200),2)  
        cv2.circle(space1,(x1,y1),5,(255,0,0),self.m1)
        cv2.circle(space1,(x2,y2),5,(0,0,255),self.m2)
        cv2.putText(space1,"100m",(20,int(space1.shape[0]-5)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0),2)
        cv2.putText(space1,f"Q1=  {Q1degree:.3f}",(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(space1,f"Q2=  {Q2degree:.3f}",(20,40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(space1,f"Q1V=  {self.Q1V:.3f}",(20,60),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cv2.putText(space1,f"Q2V=  {self.Q2V:.3f}",(20,80),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        if realOrStep==1:
            cv2.putText(space1,"Real Time ON",(20,100),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,10,10),1)
        elif realOrStep==0:
            cv2.putText(space1,"Time Step ON",(20,100),cv2.FONT_HERSHEY_SIMPLEX,0.5,(10,225,10),1)
        elif realOrStep ==2 :
            cv2.putText(space1,"Stopped",(20,100),cv2.FONT_HERSHEY_SIMPLEX,0.5,(10,10,255),1)
        if self.tailLen==0:
            cv2.putText(space1,"Full Painting",(20,120),cv2.FONT_HERSHEY_SIMPLEX,0.5,(10,10,255),1)
        drawPathPendelum(space1,xyL)
                  
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

        def empty(a):
            pass    
        def Qchange1(a):
            Qs = a * math.pi / 180  
            self.Q1=Qs
        def Qchange2(a):
            Qs = a * math.pi / 180  
            self.Q2=Qs
        def rSS(a):
            if a==2:
                self.timeStep = 0
        cv2.namedWindow("TrackBarsDouble")
        cv2.resizeWindow("TrackBarsDouble",640,500)
        cv2.createTrackbar("Mass1","TrackBarsDouble",self.m1,25,empty)
        cv2.createTrackbar("Mass2","TrackBarsDouble",self.m2,25,empty)
        cv2.createTrackbar("Angle1","TrackBarsDouble",60,360,Qchange1)
        cv2.createTrackbar("Angle2","TrackBarsDouble",90,360,Qchange2)
        cv2.createTrackbar("gravity*10","TrackBarsDouble",int(physic.g*10),300,empty)
        cv2.createTrackbar("Length1","TrackBarsDouble",200,500,empty)
        cv2.createTrackbar("Length2","TrackBarsDouble",200,500,empty)
        cv2.createTrackbar("TailLength","TrackBarsDouble",200,1274,empty)
        cv2.createTrackbar("TimeStep","TrackBarsDouble",int(self.timeStep*100),100,empty)
        cv2.createTrackbar("RealTime","TrackBarsDouble",2,2,rSS)
        cv2.createTrackbar("StopApp","TrackBarsDouble",1,1,empty)
        cv2.setTrackbarMin("TimeStep","TrackBarsDouble",1)
        cv2.setTrackbarMin("Mass1","TrackBarsDouble",1)
        cv2.setTrackbarMin("Mass2","TrackBarsDouble",1)
        cv2.setTrackbarMin("gravity*10","TrackBarsDouble",1)
        cv2.setTrackbarMin("Length1","TrackBarsDouble",5)
        cv2.setTrackbarMin("Length2","TrackBarsDouble",5)
        cv2.setTrackbarMin("TailLength","TrackBarsDouble",0)
        Close=1
        while Close==1:
            
            xyL=[]
            while True:
                start = time.time()
                
                self.m1= cv2.getTrackbarPos("Mass1","TrackBarsDouble") 
                self.m2= cv2.getTrackbarPos("Mass2","TrackBarsDouble")
                physic.g= cv2.getTrackbarPos("gravity*10","TrackBarsDouble")/10
                self.L1 = cv2.getTrackbarPos("Length1","TrackBarsDouble")
                self.L2 = cv2.getTrackbarPos("Length2","TrackBarsDouble")   
                self.tailLen = cv2.getTrackbarPos("TailLength","TrackBarsDouble")
                Close =cv2.getTrackbarPos("StopApp","TrackBarsDouble")  
                ts = cv2.getTrackbarPos("TimeStep","TrackBarsDouble")/100
                realOrStep = cv2.getTrackbarPos("RealTime","TrackBarsDouble")
                
                
                if Close==0:
                    break  
              
                
                self=physic.XYdisplacementDoublePendelum(self)

                self.cleanDrawDoublePendelum(space1,xyL,realOrStep)
                
                
                cv2.imshow("space1",space1)
                
                
                
                
                end = time.time()
                if realOrStep==1:
                    self.timeStep=0.04
                    self.t+=0.04
                    spentTime= end-start # spent time in while loop

                    if 0.04-spentTime<=0:
                        time.sleep(0)
                    else:
                        time.sleep(0.04-spentTime)   
                elif realOrStep==0:
                    self.timeStep=ts
                    self.t+=self.timeStep
                if cv2.waitKey(1) == ord('q'):
                    break
                

            while True:
                cv2.imshow("space1",space1)
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

def empty(a):
    pass    


cv2.namedWindow("Select")
cv2.resizeWindow("Select",350,90)
cv2.createTrackbar("MonoDouble","Select",0,1,empty)
cv2.createTrackbar("Start","Select",0,1,empty)

while True:
    

    start = cv2.getTrackbarPos("Start","Select")
    monodouble= cv2.getTrackbarPos("MonoDouble","Select")


    if start==1:
        cv2.destroyWindow("Select")
        if  monodouble == 0:             
            cv2.waitKey(1)
            cv2.destroyAllWindows()
            b1 = Body(m, coorX,coorY,Qmax,Len,tailLen)
        elif monodouble==1:    
            cv2.waitKey(1)
            cv2.destroyAllWindows()
            b2 = Body(m1,m2,L1,L2,Q1,Q2,Q1V,Q2V,tailLen)

        break
    if cv2.waitKey(1) == ord('q'):
        break

cv2.waitKey(1)
cv2.destroyAllWindows()
    