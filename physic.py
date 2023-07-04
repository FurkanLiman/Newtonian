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

def XYdisplacement(m,vbX,vbY,t,timeStep,x,y):
    # sadece body ile verileri derle
    x+=timeStep*(vbX)
    f = m * g
    a = f/m
    vy = vbY+a*t
    vy2 = vy
    vy1 = vbY+a*(t-timeStep)
    y+=timeStep*((vy1+vy2)/2)
    return x,y,vy,vbX