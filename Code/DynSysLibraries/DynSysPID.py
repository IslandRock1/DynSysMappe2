
import numpy as np
import matplotlib.pyplot as plt

class PID:
    def __init__(self, setpoint = 0, kp = 0, ki = 0, kd = 0):
        self.setpoint = setpoint
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.prevError = 0
        self.sumError = 0

        # 0.8G's of acceleration is MAX, based on data from https://www.quora.com/How-many-Gs-do-we-feel-driving-a-car
        # I know some cars (Like the polestar) can easily go above 1G when accelerating hard, and probably a bit more
        # when braking. I just picked 0.8G's as a start.
        self.maxGass = 0.8 * 9.81

        self.isFirst = True
    
    def update(self, value, dt = (1 / 60)):
        error = self.setpoint - value

        if (self.isFirst):
            self.isFirst = False
            self.prevError = error

        self.sumError += error * dt

        deltaError = (error - self.prevError) / dt
        self.prevError = error

        output = self.kp * error + deltaError * self.kd + self.sumError * self.ki
        if (output >= self.maxGass):
            self.sumError = 0
            return self.maxGass

        if (output <= (-self.maxGass)):
            self.sumError = 0
            return -self.maxGass

        return output

def main():
    
    pid = PID(1, 10.0, 0.2, 0.1)

    time = 10 # seconds, 60FPS
    t_values = np.linspace(0, 10, 60 * time + 1)
    y_values = [0] * (60 * time + 1)
    pådrag = [0] * (60 * time + 1)


    for ix in range(1, len(y_values)):
        if (ix == 300):
            pid.setpoint = 2.0
        
        prevY = y_values[ix - 1]
        speed = pid.update(prevY)
        pådrag[ix] = speed
        newValue = (prevY + speed * (1 / 60)) * 0.1 + (prevY * 0.9)
        y_values[ix] = newValue
    
    plt.plot(t_values, y_values, label = "Y Values")
    plt.plot(t_values, pådrag, label = "PID Output")
    plt.legend()
    plt.show()


if __name__ == "__main__": main()