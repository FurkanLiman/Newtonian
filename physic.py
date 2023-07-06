import math
# g = 9.8 m/s^2
# radius = meter
# volume = meter^3
# mass = kg

g = 9.8
# sun 274
# mars 3.71
# earth 9.8


def findPeriodPendelum(L):
    T=2*math.pi*math.sqrt((L/g))
    return T

def XYdisplacementPendelum(T,Qmaks,t,L):
    Q = Qmaks*math.cos((((2*math.pi)/T)*t))
    print(f"{Q}         {t}     {Qmaks}")
    x=L*math.sin(Q)
    y=L*math.cos(Q)
    return Q,x,y

def XYdisplacementFreeFall(body,t,timeStep):
    x=body.coorX
    y=body.coorY
    m=body.mass
    vbX=body.vbX
    vbY=body.vbY
    x+=timeStep*(vbX)
    f = m * g
    a = f/m
    vy = vbY+a*t
    vy2 = vy
    vy1 = vbY+a*(t-timeStep)
    y+=timeStep*((vy1+vy2)/2)
    return x,y,vy,vbX
def drawRoute(body,t,timeStep,space):
    pathXY=[]
    x=body.coorX
    y=body.coorY
    m=body.mass
    vbX=body.vbX
    vbY=body.vbY
    while True:
        x+=timeStep*(vbX)
        f = m * g
        a = f/m
        vy = vbY+a*t
        vy2 = vy
        vy1 = vbY+a*(t-timeStep)
        y+=timeStep*((vy1+vy2)/2)
        t+=timeStep
        pathXY.append((x,y))
        if y>=space.shape[0]*0.98:
            break
    return pathXY 