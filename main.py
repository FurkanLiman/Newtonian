import time
import cv2
import numpy as np
import math
import physic

class body:
    def __init__(self, mass, radius, konumX, konumY, vb,vbAngle):
        self.mass= mass
        self.volume= (4/3)*(math.pi*math.pow(radius,3))
        self.konumX = konumX
        self.konumY = konumY
        self.vb=vb
        self.vbX,self.vbY= self.vbAngleCalculate(vb,vbAngle)
        
    def vbAngleCalculate(self,vb,vbAngle):
        vbRadianA = vbAngle * math.pi / 180
        vbX=vb*math.cos(vbRadianA)
        vbY=vb*math.sin(vbRadianA)
        return vbX,vbY 
# b1: 1kg, r = 1m
m,r,konumX,konumY,vb,vbAngle=1,10,00,500,-50,135
b1 = body(m, r, konumX, konumY,vb,vbAngle)

v=0
vx=0
vy=0

space = np.zeros([1000,1000,3],dtype=np.uint8)

def temizleCiz():
    space[:,:]=0
    for i in range(space.shape[1]//100):
        space[(2*i)*100:(2*i+1)*100,int(space.shape[0]*0.98):space.shape[0]] = 255
        space[(2*i+1)*100:(2*i+3)*100,int(space.shape[0]*0.98):space.shape[0]] = 125
        space[int(space.shape[0]*0.98):space.shape[0],(2*i)*100:(2*i+1)*100] = 255
        space[int(space.shape[0]*0.98):space.shape[0],(2*i+1)*100:(2*i+3)*100] = 125
    cv2.putText(space,"100m",(20,int(space.shape[0]-5)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0),2)

t=0 
x=b1.konumX
y=b1.konumY
timeStep= 0.05
while True:
    start = time.time()
    
    cv2.imshow("space",space)

    if b1.konumY>=space.shape[0]*0.98-r:
        v=0
        break

    temizleCiz()

    x,y,vy,vx=physic.XYdisplacement(m,b1.vbX,b1.vbY,t,timeStep,x,y)
    xint=int(x)
    yint=int(y)
    Vxint=int(vx)
    Vyint=int(vy)
    vint=int(math.sqrt((vx**2)+(vy**2)))
    

    b1.konumX=xint
    b1.konumY=yint

    cv2.putText(space,f"y={yint} meter",(15,15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255))
    cv2.putText(space,f"V={vint} m/s",(15,30),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255))
    cv2.putText(space,f"t={t} s",(15,45),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255))
    cv2.arrowedLine(space,(b1.konumX,b1.konumY),(b1.konumX+Vxint,b1.konumY+Vyint),(255,0,0),3) # total-> blue
    cv2.arrowedLine(space,(b1.konumX,b1.konumY),(b1.konumX,b1.konumY+Vyint),(0,255,0),3) # y arrow -> green
    cv2.arrowedLine(space,(b1.konumX,b1.konumY),(b1.konumX+Vxint,b1.konumY),(0,0,255),3) # x arrow -> red
    cv2.circle(space,(b1.konumX,b1.konumY),5,(0,0,255),r)
    
    
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