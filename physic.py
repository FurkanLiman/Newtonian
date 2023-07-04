import math
# g = 9.8 m/s^2
# radius = meter
# volume = meter^3
# mass = kg

g = 9.8
# sun 274
# mars 3.71
# earth 9.8

# f = ma
# a = f/m
# a = m*g/m

def XYdisplacement(body,t,timeStep):

    x=body.konumX
    y=body.konumY
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
    x=body.konumX
    y=body.konumY
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