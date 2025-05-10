
import numpy as np
from DynSysLibraries.DynSysPID import PID

class Car:
    def __init__(self, targetVelocity, targetDistance = 100):
        self._pos = 0.0
        self._vel = 0.0
        self._acc = 0.0
        self._power = 80_000 # Power of Nissan Leaf motor

        self.pidVel = PID(targetVelocity, 0.2, 0, 0, 1.0)
        self.pidDist = PID(targetDistance, 0.2, 0, 0.5, 1.0)

        self.logPos = [self._pos]
        self.logVel = [self._vel]
        self.logAcc = [self._acc]

        self._output = 0
        self.simulateResistance = True
        self.rhoDiv2 = 1.29 / 2.0
        self.C = 0.3 #Drag coeficent
        self.A = 2.5 #Front area
        self.Mass = 1500.0
        self.Crr = 0.015


        self.maxA = 0
    
    def setPos(self, pos):
        self._pos = pos
        self.logPos[-1] = pos
    
    def setVel(self, vel):
        self._vel = vel
        self.logVel[-1] = vel
    
    def setAcc(self, acc):
        self._acc = acc
        self.logAcc[-1] = acc

    def _regulator(self, dt, nextCar: "Car"):
        if (nextCar):
            dist = nextCar._pos - self._pos
            outputDist = -self.pidDist.update(dist)
            outputVel = self.pidVel.update(self._vel, dt)
            output = min(outputVel, outputDist)
        else:
            output = self.pidVel.update(self._vel, dt)

        a = 0.9
        self._output = self._output * a + output * (1 - a)

    def _simulateStateSpace(self, dt):
        k = self.rhoDiv2 * self.C * self.A

        AMatrix = np.matrix([[0, 1], [0, -k * self._vel / self.Mass]])
        BMatrix = np.matrix([[0], [self._power / (self.Mass * abs(max(20.0, self._vel)))]])

        xDot = AMatrix @ np.matrix([[self._pos], [self._vel]]) + BMatrix * self._output

        state = np.matrix([[self._pos], [self._vel]])
        state += xDot * dt

        self._pos = state[0, 0]
        self._vel = state[1, 0]

    def _simulate(self):
        force = 0.0
        
        airResistance = self.rhoDiv2 * self.C * self.A * self._vel * self._vel
        rollingResistance = self.Crr * self.Mass * 9.81

        if (self.simulateResistance):

            if (self._vel >= 0.0):
                sign = -1
            else:
                sign = 1

            force += sign * airResistance
            # force += sign * rollingResistance
        
        minVel = 20.0
        if (abs(self._vel) > minVel):
            carForce = (self._power * self._output) / (abs(self._vel)) # + 0.1 to not div by zero
        else:
            carForce = (self._power * self._output) / (minVel)

        force += carForce
        self._acc = force / self.Mass

    def update(self, dt = (1 / 60), nextCar = None):
        self._regulator(dt, nextCar)
        self._simulateStateSpace(dt)

        # Only use if using self._simulate.
        #self._vel += self._acc * dt
        #self._pos += self._vel * dt

        self.logAcc.append(self._acc)
        self.logVel.append(self._vel)
        self.logPos.append(self._pos)