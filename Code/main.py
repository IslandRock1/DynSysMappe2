
from math import sin
import numpy as np
import matplotlib.pyplot as plt
from DynSysLibraries.Car import Car

def main():
    car = Car(20.0)
    car.setVel(15.0)

    frontCar = Car(15.0)
    frontCar.setPos(150)
    frontCar.setVel(15.0)
    frontCar.setAcc(10.0)

    time = 200 # seconds
    frequenzy = 60 # hertz
    t_values = np.linspace(0, time, frequenzy * time + 1)
    dist = [frontCar.getPos() - car.getPos()] * (frequenzy * time + 1)

    dt = (1 / frequenzy)

    for ix in range(1, len(t_values)):
        frontCar.pidVel.setpoint = 15 + 10 * sin(16 * ix / (len(t_values)))
        
        frontCar.update(dt)
        car.update(dt, frontCar)
        dist[ix] = frontCar.getPos() - car.getPos()

    if (True):
        plt.title("Position")
        plt.plot(t_values, car.logPos, label = "Back Car Position")
        plt.plot(t_values, frontCar.logPos, label = "Lead Car Position")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")

        plt.legend()
        plt.show()

    if (True):
        plt.title("Distance between cars")
        plt.plot(t_values, np.array(dist), label = "Distance")
        plt.xlabel("Time (s)")
        plt.ylabel("Distance (m)")

        print(f"Min dist: {min(dist)}")

        #plt.legend()
        plt.show()

    if (True):
        plt.plot(t_values, np.array(car.logAcc), label = "Back Car Acceleration")

        plt.title("Acceleration of car")
        plt.xlabel("Time (s)")
        plt.ylabel("Acceleration (m/s^2)")

        #plt.legend()
        plt.show()

    if (True):
        plt.title("Velocity")
        plt.plot(t_values, car.logVel, label = "Back Car Velocity")
        plt.plot(t_values, frontCar.logVel, label = "Lead Car Velocity")
        print(f"Max speed back car: {car.logVel[-1]}")

        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")

        plt.legend()
        plt.show()

if __name__ == "__main__":
    main()
