
import numpy as np
import matplotlib.pyplot as plt
from DynSysLibraries.Car import Car

def main():
    car = Car(20.0)
    car.vel = 10
    frontCar = Car(15.0)
    frontCar.setPos(100)
    frontCar.setVel(15.0)
    frontCar.setAcc(10.0)

    time = 20 # seconds
    frequenzy = 60 # hertz
    t_values = np.linspace(0, time, frequenzy * time + 1)
    dist = [frontCar._pos - car._pos] * (frequenzy * time + 1)

    dt = (1 / frequenzy)

    for ix in range(1, len(t_values)):
        # if (ix > (len(t_values) / 2)):
        #     frontCar.pidVel.setpoint = 30
        
        frontCar.update()
        car.update(dt, frontCar)
        dist[ix] = frontCar._pos - car._pos

    if (True):
        plt.plot(t_values, car.logPos, label = "Back Car Position")
        plt.plot(t_values, frontCar.logPos, label = "Front Car Position")

    if (True):
        plt.plot(t_values, car.logVel, label = "Back Car Velocity")
        plt.plot(t_values, frontCar.logVel, label = "Front Car Velocity")
    
    #plt.plot(t_values, dist, label = "Distance")
    #plt.plot(t_values, car.logAcc, label = "Acceleration")

    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
