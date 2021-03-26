#!/usr/bin/env python3
import pygame, sys #importar pygame
import numpy as np
from numpy.linalg import inv
import math

import math
pygame.init()  #iniciar libreria
pygame.display.set_caption('DDR')
size=(1200,900)  #variable indicating windows size
#Defining color
BLACK = (  0,  0,  0)
GRAY = ( 169, 169, 169)
WHITE = ( 255, 255, 255)
GREEN = (  56, 188, 28)
RED = ( 255, 0, 0)
BROWN = (  128,  64, 0)

#CORE CLASS
#
class DDR:
    def __init__(self, x,y,theta):
        self.x = x
        self.y = y
        self.theta_rad = -theta * np.pi / 180
        self.l = 30
        self.lw = 25
        self.alpha = 0
        self.th_w1 = (-theta - 90) * np.pi / 180
        self.th_w2 = (-theta + 90) * np.pi / 180

        self.p0 = np.array([[self.x],
                            [self.y]])

        self.p1 = np.array([[self.l * np.cos(self.theta_rad) ],
                            [self.l * np.sin(self.theta_rad)],
                            [1]])

        self.pw1 = np.array([[self.lw * np.cos(self.th_w1)],
                             [self.lw * np.sin(self.th_w1)],
                             [1]])

        self.pw2 = np.array([[self.lw * np.cos(self.th_w2)],
                             [self.lw * np.sin(self.th_w2)],
                             [1]])

        self.transform = np.array([[np.cos(self.alpha), -np.sin(self.alpha)],
                                [np.sin(self.alpha), np.cos(self.alpha)]])

        self.transform2 = np.array([[np.cos(self.alpha), -np.sin(self.alpha),self.x],
                              [np.sin(self.alpha), np.cos(self.alpha), self.y],
                                   [0,0,1]])


        # BODY COORDINATES
        self.p0 = self.p0
        self.p1 = self.transform2@self.p1
        self.pw1 = self.transform2@self.pw1
        self.pw2 = self.transform2@self.pw2

    #DRAWINF THE VEHICLE
    def body(self):
        pygame.draw.circle(screen, GRAY, (self.p0[0,0], self.p0[1,0]), 30)
        pygame.draw.circle(screen, BROWN, (self.p1[0, 0], self.p1[1, 0]), 5)
        pygame.draw.circle(screen, BLACK, (self.pw1[0, 0], self.pw1[1, 0]), 10)
        pygame.draw.circle(screen, BLACK, (self.pw2[0, 0], self.pw2[1, 0]), 10)

#Creating window

screen = pygame.display.set_mode(size)  #calling the las variable in tuple
# clock = pygame.time.Clock() #To control FPS

#initial conditions
xi = 10
yi = 10
theta = np.pi/4
xl = xi
yl = yi
#Goal position
xd = 400
yd = 400
th_d = np.pi
#Velocity
L = 80
a=0.1

# xi = xi + a*np.cos(theta)
# yi = yi + a*np.cos(theta)

# vx = 3
# vy = 3
# w = 5
# v = 6
vl = 0.01   # meter per second
vr = 0.01


#obstacles
obstacle = []
for i in range(5):
    x = np.random.choice(range(200,1100,100))
    y = np.random.choice(range(200, 600,100))
    diameter = np.random.randint(20,50)
    obstacle.append([x,y,diameter])
#
dt = 0
lasttime = pygame.time.get_ticks()
trail = []
while True:
    for event in pygame.event.get(): #identificar que sicede en ventana
        #print(event) #imprime lo que sucede en ventana
        if event.type == pygame.QUIT:
            sys.exit()
    #SECTION OF LOGIC
     # for i in range(1,50):


    xe = xd - xi  #position error x
    ye = yd - yi #position error y
    th_d = math.atan2(ye, xe)
    th_d = (th_d*180)/np.pi
    th_e = th_d-theta

    e = np.array([[xe],
                [ye]])

    #matriz
    J=np.array([[np.cos(theta),-a*np.sin(theta)],
                [np.sin(theta), a*np.cos(theta)]])
    k = np.array([[0.1, 0],
                    [0,0.1]])
    #control law
    vel = (inv(J)@k)@e
    #     #
    v = vel[0,0]
    # w = vel[1,0]
    w = th_e
    #
    #
    xrp = v * np.cos(theta)-a*w*np.sin(theta)
    yrp = v * np.sin(theta)+a*w*np.cos(theta)

    xi += xrp * dt
    yi += yrp * dt
    theta += w*dt
    print(theta)
    # #     # xi += ((vl+vr)/2)*np.cos(theta)*dt
    # #     # yi += ((vl + vr) / 2) * np.sin(theta) * dt
    # #     # theta +=(vr-vl)/w*dt
    # #
    # #     # xi += vx
    # #     # yi += vy
    #
    trail.append([xi,yi])

    ddr = DDR(xi, yi, theta)




    #SECTION OF LOGIC
    # color de fondo de pantalla
    dt = (pygame.time.get_ticks() - lasttime)/1000  # in seconds
    lasttime = pygame.time.get_ticks()

    screen.fill(WHITE)
    ###----- ZONA DE DIBUJO

    ddr.body()

    #trail of robot
    for i in range(0,len(trail)-1):
        pygame.draw.line(screen,BLACK,(trail[i][0],trail[i][1]),
                         (trail[i+1][0],trail[i+1][1]))
    if trail.__sizeof__()>5000:
        trail.pop(0)
    trail.append([xi,yi])

    #obstacle drawing
    for k in obstacle:
        x = k[0]
        y = k[1]
        diameter = k[2]
        pygame.draw.circle(screen,BROWN,(x,y),diameter)

    pygame.draw.circle(screen, GREEN, (xd, yd), 5)
    pygame.draw.line(screen,RED,(xl,yl),(xd,yd),1)
    # actualizar pantalla
    pygame.display.flip()
    # clock.tick(10)
