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

def XYdisplacementPendelum(body,Qmaks,t,L):
    T = body.T
    Q = Qmaks*math.cos((((2*math.pi)/T)*t))
    
    x=L*math.sin(Q)
    y=L*math.cos(Q)
    
    body.Q = Q
    body.coorX=x
    body.coorY=y
    return body

def XYdisplacementFreeFall(body,t):
    timeStep= body.timeStep
    x=body.coorX
    y=body.coorY
    m=body.mass
    vbX=body.vbX
    vbY=body.vbY
    vx=body.vx
    vy=body.vy
    
    vx=vbX
    x+=timeStep*(vbX)
    f = m * g
    a = f/m
    vy = vbY+a*t
    vy2 = vy
    vy1 = vbY+a*(t-timeStep)
    y+=timeStep*((vy1+vy2)/2)
    
    body.coorX = x
    body.coorY = y

    body.vx = vx
    body.vy = vy
    

    return body

def drawRouteFreeFall(body,t,space):
    timeStep= body.timeStep
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

def XYdisplacementDoublePendelum(body):

    m1=body.m1
    m2=body.m2
    L1=body.L1
    L2=body.L2
    Q1=body.Q1
    Q2=body.Q2
    Q1V=body.Q1V
    Q2V=body.Q2V
    timestep = body.timeStep

    Q1a = ((-g * (2 * m1 + m2) * math.sin(Q1))+(-m2 * g * math.sin(Q1-2*Q2)) + (-2*math.sin(Q1-Q2)*m2)*(Q2V*Q2V*L2+Q1V*Q1V*L1*math.cos(Q1-Q2))) / (L1 * (2*m1+m2-m2*math.cos(2*Q1-2*Q2)))

    Q2a = ((2 * math.sin(Q1-Q2))*(((Q1V*Q1V*L1*(m1+m2)))+(g * (m1 + m2) * math.cos(Q1))+(Q2V*Q2V*L2*m2*math.cos(Q1-Q2)))) / (L2 * (2*m1+m2-m2*math.cos(2*Q1-2*Q2)))

    x1 = L1 * math.sin(Q1)
    y1 = L1 * math.cos(Q1)

    x2 = x1 + L2 * math.sin(Q2)
    y2 = y1 + L2 * math.cos(Q2)

    Q1V += Q1a*timestep
    Q2V += Q2a*timestep
    Q1 += Q1V*timestep
    Q2 += Q2V*timestep
    
    body.Q1 = Q1
    body.Q2 = Q2
    body.Q1V= Q1V
    body.Q2V= Q2V
    
    body.x1 = x1
    body.y1 = y1
    body.x2 = x2
    body.y2 = y2
    
    return body