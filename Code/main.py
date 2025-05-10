
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
    dist = [frontCar._pos - car._pos] * (frequenzy * time + 1)

    dt = (1 / frequenzy)

    for ix in range(1, len(t_values)):
        # frontCar.setVel(15 + 15 * sin(16 * ix / (len(t_values))))
        frontCar.pidVel.setpoint = 15 + 10 * sin(16 * ix / (len(t_values)))
        
        frontCar.update()
        car.update(dt, frontCar)
        dist[ix] = frontCar._pos - car._pos

    if (False):
        plt.plot(t_values, car.logPos, label = "Back Car Position")
        plt.plot(t_values, frontCar.logPos, label = "Front Car Position")

    if (False):
        plt.plot(t_values, dist, label = "Distance")

    if (True):
        plt.plot(t_values, np.array(car.logAcc), label = "Back Car Acceleration")

    if (True):
        plt.plot(t_values, car.logVel, label = "Back Car Velocity")
        plt.plot(t_values, frontCar.logVel, label = "Front Car Velocity")
        # print(f"Max speed front car: {frontCar.logVel[-1]}")

    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
