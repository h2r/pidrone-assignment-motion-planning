import matplotlib
#matplotlib.use('Qt4Agg')
from matplotlib.patches import Circle

from rrt import RRT, dist

import numpy as na
from time import time
from mpl_toolkits.mplot3d import axes3d, Axes3D
from math import cos, sin

import matplotlib.pyplot as plt
import math
import matplotlib.animation as animation

def deg2rad(deg):
    return deg / 180.0 * math.pi

class Arm2D:
    def __init__(self):
        self.joints = na.array([deg2rad(90), deg2rad(30.0)])
        self.lengths = [3.0, 5.0]
        self.origin = [0.0, 0.0, 0.0]
        self.trajectory = [()]
        self.trajectoryLoc = None
        self.obstacles = [(1.0, 1.0, 0.5)]
        self.makeikmap()
        
        self.update()

    def makeikmap(self):
        self.ikmap = {}
        oldjoints = list(self.joints)
        
        for theta1 in na.arange(0, 2*math.pi, 0.1):
            for theta2 in na.arange(0, 2*math.pi, 0.1):
                self.update(joints=[theta1, theta2])
                if not self.inCollision():
                    self.ikmap[tuple(self.jointGlobals[-1])] = (theta1, theta2)

        self.joints = oldjoints
        self.update()
        
    def update(self, joints=None):
        if joints != None:
            self.joints = list(joints)
        if self.trajectoryLoc != None:
            for i, dtheta in enumerate(self.trajectory[self.trajectoryLoc]):
                self.joints[i] = dtheta
            self.trajectoryLoc += 1
            if self.trajectoryLoc >= len(self.trajectory):
                self.trajectoryLoc = None
            
        jointGlobals = []
        jointTransforms = []
        here = na.array(self.origin)
        jointGlobals.append(na.copy(here))
        for i, l in enumerate(self.lengths):
            here[2] += self.joints[i]
            here[0] += self.lengths[i] * math.cos(here[2])
            here[1] += self.lengths[i] * math.sin(here[2])
            jointGlobals.append(na.copy(here))
            

        self.jointGlobals = na.array(jointGlobals)
        self.jointTransforms = na.array(jointTransforms)
    


    def intersects(self, x1, y1, x2, y2, x, y, r):
        " From: http://mathworld.wolfram.com/Circle-LineIntersection.html"
        x1 = x1 - x
        y1 = y1 - y
        x2 = x2 - x
        y2 = y2 - y

        dx = x2 - x1
        dy = y2 - y1
        dr = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)) 

        D = x1 * y2 - x2 * y1
        #print
        #print "D", D
        #print "t1", math.pow(r,2) * math.pow(dr, 2)
        delta = math.pow(r,2)*math.pow(dr, 2) - math.pow(D,2)
        #print "delta", delta
        result = None
        if delta < 0:
            result = False
        else:

            ix1 = (D * dy + na.sign(dy) * dx * math.sqrt(math.pow(r,2) *
                                                         math.pow(dr,2) -
                                                         math.pow(D,2))) / math.pow(dr, 2)
            ix2 = (D * dy - na.sign(dy) * dx * math.sqrt(math.pow(r,2) *
                                                         math.pow(dr,2) -
                                                         math.pow(D,2))) / math.pow(dr, 2)

            iy1 = (-D * dx + na.abs(dy) * math.sqrt(math.pow(r,2) *
                                                     math.pow(dr,2) -
                                                     math.pow(D,2))) / math.pow(dr, 2)

            iy2 = (-D * dx + na.abs(dy) * math.sqrt(math.pow(r,2) *
                                                     math.pow(dr,2) -
                                                     math.pow(D,2))) / math.pow(dr, 2)
            # ix1 += x
            # ix2 += x
            # iy1 += y
            # iy2 += y
            #print "ip1: ", ix1, iy1
            #print "ip2: ", ix2, iy2


            if (((x1 <= ix1 and ix1 <= x2) or (x2 <= ix1 and ix1 <= x1)) and
                ((y1 <= iy1 and iy1 <= y2) or (y2 <= iy1 and iy1 <= y1)) and
                ((x1 <= ix2 and ix2 <= x2) or (x2 <= ix2 and ix2 <= x1)) and
                ((y1 <= iy2 and iy2 <= y2) or (y2 <= iy2 and iy2 <= y1))):
                result = True
            else:
                result = False
        #print "p1", x1, y1
        #print "p2", x2, y2
        #print "dx, dy", dx, dy
        #print "dr", dr
        #print "D", D
        #print "r", r
        return result
    

    def inCollision(self):
        for x, y, r in self.obstacles:
            for (x1, y1, theta1), (x2, y2, theta2) in zip(self.jointGlobals, 
                                                          self.jointGlobals[1:]):
                if self.intersects(x1, y1, x2, y2, x, y, r):
                    return True
        return False
                
        
class Arm2DGraph:
    def __init__(self, armax, confax, rrt):

        self.arm = Arm2D()
        self.press = None
        self.armax = armax
        self.confax = confax
        self.last_artists = []
        self.rrt = rrt


        self.start_rrt = False
        self.pause_rrt = False
        self.reset_rrt = True
        self.collisiontext = armax.text(-10, 10, "Collision")



    def drawFrame(self, x, y, theta):
        artists = []

        artists.extend(self.armax.plot([x, x + math.cos(theta)],
                                      [y, y + math.sin(theta)], 'o-', lw=2, c='r'))
        artists.extend(self.armax.plot([x, x + math.cos(theta + math.pi/2)],
                                      [y, y + math.sin(theta + math.pi/2)], 'o-', lw=2, c='g'))
        
        return artists
    def step(self, t):
        pass


    def animate(self, t):

        for a in self.last_artists:
            a.remove()



        t = time()
        self.step(t)
        self.arm.update()
        artists = []
        artists.extend(self.armax.plot(self.arm.jointGlobals[:,0], self.arm.jointGlobals[:,1], 'o-', lw=2, c='k'))

        
        here = na.array([0.0, 0.0, 0.0])

        for i in range(len(self.arm.jointGlobals)):
            here = self.arm.jointGlobals[i]
            x, y, theta = here
            artists.extend(self.drawFrame(x, y, theta))
            
        artists.append(self.confax.scatter([self.arm.joints[0]],
                                           [self.arm.joints[1]]))
        
        if self.reset_rrt:
            self.rrt.reset(tuple(self.arm.joints))
            self.reset_rrt = False

        artists.extend(self.rrt.animate(t, self.start_rrt))
            
        for x, y, r in self.arm.obstacles:
            c = Circle((x, y), r)
            artists.append(c)
            self.armax.add_artist(c)

        self.last_artists = artists

        if self.arm.inCollision():
            self.collisiontext.set_text("collision!")
        else:
            self.collisiontext.set_text("no collision!")

        return artists
        
    def on_press(self, event):
        if event.inaxes != self.confax: 
            return
        self.press = event.xdata, event.ydata
        print "press"

    def on_release(self, event):
        self.press = None
        if event.inaxes == self.confax: 
            print "calling rrt path"
            min_dist, path = self.rrt.path((event.xdata, event.ydata))
            print "path: ", path
            self.arm.trajectory = path
            self.arm.trajectoryLoc = 0
        elif event.inaxes == self.armax:
            eetargetp = event.xdata, event.ydata
            print "armax", eetargetp

            mind = None
            mincspace = None
            minreal = None
            for realpt, cspacep in self.arm.ikmap.iteritems():
                print "cspacep", cspacep
                
                d = dist(realpt[0:2], eetargetp)
                if mind == None or d < mind:
                    mind = d
                    mincspace = cspacep
                    minreal = realpt
            print "target:", eetargetp
            print "best ik real", minreal
            print "best ik cspace", mincspace
            min_dist, path = self.rrt.path((mincspace[0], mincspace[1]))
            self.arm.trajectory = path
            self.arm.trajectoryLoc = 0


    def on_motion(self, event):
        if self.press is None: 
            return
        if event.xdata == None or event.ydata == None:
            return

        xpress, ypress = self.press
        dx = event.xdata - xpress
        dy = event.ydata - ypress
        #print('x0=%f, xpress=%f, event.xdata=%f, dx=%f, x0+dx=%f' %
        #      (x0, xpress, event.xdata, dx, x0+dx))
        #target = self.jointGlobals[-1] + [dx, dy]
        
        print "dx, dy", dx, dy

    def on_key_press(self, event):
        if event.key == 'j':
            self.arm.joints[0] += 0.1
        elif event.key == 'l':
            self.arm.joints[0] -= 0.1

        if event.key == 'i':
            self.arm.joints[1] -= 0.1

        if event.key == 'k':
            self.arm.joints[1] += 0.1

        if event.key == 'r':
            self.start_rrt = not self.start_rrt
        if event.key == 'x':
            self.reset_rrt = True

    def on_key_release(self, event):
        pass

def main():
    fig = plt.figure()
    armax = fig.add_subplot(121, autoscale_on=False, xlim=(-15, 15), ylim=(-15,15))

    confax = fig.add_subplot(122, autoscale_on=False, 
                             xlim=(-2 * math.pi, 2.5 * math.pi), 
                             ylim=(-2 * math.pi, 2.5 * math.pi))

    rrt = RRT(confax, Arm2D())
    confax.set_title("Configuration Space")
    armax.set_title("Arm")
    arm = Arm2DGraph(armax, confax, rrt)
    armax.set_aspect('equal')
    confax.set_aspect('equal')
    oldjoints = arm.arm.joints

    cx = []
    cy = []
    for theta1 in na.arange(-2*math.pi, 2.5*math.pi, 0.1):
        for theta2 in na.arange(-2*math.pi, 2.5*math.pi, 0.1):
            arm.arm.update(joints=[theta1, theta2])
            if arm.arm.inCollision():
                cx.append(theta1)
                cy.append(theta2)
    confax.scatter(cx, cy, c='r', marker='x')
    arm.arm.joints = oldjoints

    #ax.axis((-10, 10, -10, 10))
    fig.canvas.mpl_connect('button_press_event', arm.on_press)
    fig.canvas.mpl_connect('button_release_event', arm.on_release)
    fig.canvas.mpl_connect('motion_notify_event', arm.on_motion)
    fig.canvas.mpl_connect('motion_notify_event', arm.on_motion)

    fig.canvas.mpl_connect('key_press_event', arm.on_key_press)
    fig.canvas.mpl_connect('key_release_event', arm.on_key_release)
    armax.grid()
    confax.grid()
    
    ani = animation.FuncAnimation(fig, arm.animate,
                                  interval=10, blit=False)



    plt.show()
    

if __name__ == "__main__":
    main()

