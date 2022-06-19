import turtle

class Draw:
    def __init__(self, point):
        self.point = point
        self.t = turtle.Turtle()
        self.t.speed(0)

    def setup(self):
        for i in range (0, 4):
            ix = self.point[i][0]
            iy = self.point[i][1]
            self.t.goto(ix, iy)
            self.t.dot(5, "black")
            self.t.pendown()
        self.t.penup()

    def run(self, z):
        self.t.goto(z[0], z[1])
        self.t.dot(5, "red")
        self.t.pendown()
        self.t.pencolor("red")
        self.t.getscreen().update()