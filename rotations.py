import matplotlib.pyplot as plt
import math
import random
import matplotlib.animation as animation
import numpy as na

def deg2rad(deg):
    return deg / 180.0 * math.pi

def rad2deg(rad):
    return rad / math.pi * 180.0

class Complex:
    def __init__(self, real, img):
        self.real = real
        self.img = img
        
    @staticmethod
    def fromAngle(theta):
        return Complex(math.cos(theta), math.sin(theta))
    @staticmethod
    def conjugate(c1, c2):
        return Complex(c1.real * c2.real - c1.img * c2.img,
                       c1.real * c2.img + c1.img * c2.real)

    def rotate(self, point):
        return point

    def toAngle(self):
        return math.atan2(self.img, self.real)

class RotationMatrix:
    def __init__(self, theta):
        self._matrix = na.array([])
        
    def rotate(self, point):
        return point


class Robot:
    def __init__(self, axes):
        self.pos = na.array([0.0, 0.0])
        #self.rot = 0.0
        self.rot = Complex(1, 0)
        self.axes = axes

    def translate(self, amount):
        # theta
        #self.pos[0] = self.pos[0] + amount * math.cos(self.rot)
        #self.pos[1] = self.pos[1] + amount * math.sin(self.rot)

        
        self.pos[0] = self.pos[0] + amount * math.cos(self.rot.toAngle())
        self.pos[1] = self.pos[1] + amount * math.sin(self.rot.toAngle())

    def rotate(self, theta):
        #self.rot += theta
        self.rot = Complex.conjugate(self.rot, Complex.fromAngle(theta))

    def animate(self, frame):
        return self.draw()

    def draw(self):
        artists = []
        artists.append(self.axes.scatter([self.pos[0]], [self.pos[1]], 
                                        s=200, c='k', marker='o'))

        artists.extend(self.axes.plot([self.pos[0], self.pos[0] + math.cos(self.rot.toAngle()) * 3],
                                      [self.pos[1], self.pos[1] + math.sin(self.rot.toAngle()) * 3], c='w'))
        return artists

    def on_key_press(self, event):
        """
        On key press. Handles keyboard-based joint control.
        """
        if event.key == 'j':
            self.rotate(0.1)
        elif event.key == 'l':
            self.rotate(-0.1)
        elif event.key == 'i':
            self.translate(1.0)
        elif event.key == 'k':
            self.translate(-1.0)

def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-15, 15), ylim=(-15,15))
    ax.set_aspect('equal')
    ax.axis((-100, 100, -100, 100))
    plt.title("2D Rotations")
    ax.grid()

    robot = Robot(ax);
    
    ani = animation.FuncAnimation(fig, robot.animate,
                                  interval=10, blit=True)

    fig.canvas.mpl_connect('key_press_event', robot.on_key_press)

    ax.grid()

    print ani    
    plt.show()


if __name__ == "__main__":
    main()


