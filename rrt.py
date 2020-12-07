import matplotlib.pyplot as plt
import math
import random
import matplotlib.animation as animation
import numpy as na

def dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt(math.pow(x2 - x1, 2) +
                     math.pow(y2 - y1, 2))


def deg2rad(deg):
    return deg / 180.0 * math.pi

def rad2deg(rad):
    return rad / math.pi * 180.0


class RRT:
    def __init__(self, axes, arm):
        self.axes = axes
        self.reset((0,0))
        self.arm = arm

    def reset(self, angles):
        """Angles is a tuple of (theta1, theta2), two angles in the configuration space.
        """

        # the vertices in the graph, initialized with angles. 
        self._vertices = [angles]

        
        self._edges = {}  # a map of a tuple of angles to a list of new angles. 
        self._edges.setdefault(self._vertices[0], [])
        
    
    def animate(self, x, doStep=True):
        if doStep:
            q_near, q_new = self.step()

        return self.plotAll()

    def plotAll(self):
        artists = []

        for v in self._vertices:
            x1, y1 = v
            for x2, y2 in self._edges[v]:
                artists.extend(self.axes.plot([x1, x2], [y1, y2], 'o-', c='k', marker="."))
        return artists

        


    def nearest_vertex(self, q):

        """Takes as input a tuple of angles.  Returns the vertex in the graph
        nearest to this tuple.
        """
        
        min_dist = 10000
        min_p = None
        for v in self._vertices:
            d = dist(v, q) 
            if d < min_dist:
                min_p = v
                min_dist = d
        if min_p != None:
            return min_p, min_dist
        else:
            raise ValueError("Must have a minimum.")

    def new_conf(self, q_near, q_rand, deltaq):
        """Takes a vertex in the graph, the randomly sampled point, and a
        delta.  Returns a new point delta away from q_near, in the
        direction of q_rand.
        """
        return None
    
    def path(self, node):

        """Given a node, returns a tuple of min_dist and best_path, a list of
        angles in the graph.  I did it with BFS and it was fast
        enough, although A* is better if things get really big.
        """
        


    def pointsOnLine(self, p1, p2, npoints):
        x1, y1 = p1
        x2, y2 = p2
        m = (y2 - y1) / (x2 - x1)
        b = y1 - x1 * m
        length = dist(p1, p2)
        dx = (x2 - x1) / npoints

        return [(x, m * x + b) for x in na.arange(x1, x2, dx)]
        
    def inCollision(self, q_near, q_new):

        """Takes two points in configuration space and checks if the line
        segment of points in configuration space is in collision with
        the arm in real space.
        """
        
        for p in self.pointsOnLine(q_near, q_new, 10):
            self.arm.joints[0] = p[0]
            self.arm.joints[1] = p[1]
            self.arm.update()
            if self.arm.inCollision():
                return True
        return False

        
        
    def step(self):
        """
        Steps the RRT one step. 
        """

        while True:
            q_rand = (random.random() * 4 * math.pi - 2 * math.pi,
                      random.random() * 4 * math.pi - 2 * math.pi)
            #q_rand = -5, -10
            
            print "q_rand", q_rand
            q_near, d = self.nearest_vertex(q_rand)
            print "q_near", q_near
            
            q_new = self.new_conf(q_near, q_rand, 0.5)
            print "new", q_new
            if not self.inCollision(q_near, q_new):
                break
            
        self._vertices.append(q_new)
        self._edges.setdefault(q_new, [])
        self._edges[q_near].append(q_new)
        return q_near, q_new
        


def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-15, 15), ylim=(-15,15))
    ax.set_aspect('equal')
    ax.axis((-50, 50, -50, 50))
    plt.title("2D RRT")
    ax.grid()
    rrt = RRT(ax)
    
    ani = animation.FuncAnimation(fig, rrt.animate,
                                  interval=10, blit=True)
    print ani    
    plt.show()

if __name__ == "__main__":
    main()


