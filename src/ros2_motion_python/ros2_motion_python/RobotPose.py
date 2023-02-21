class RobotPose:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def get_x(self):
        return self.x
    
    def set_x(self, x):
        self.x = x

    def get_y(self):
        return self.y

    def set_y(self, y):
        self.y = y 

    def get_theta(self):
        return self.theta

    def set_theta(self, theta):
        self.theta = theta 

    def get_pose(self):
        return self

    def __str__(self):
        return "RobotPose(x={}, y={}, theta={})".format(self.x, self.y, self.theta)