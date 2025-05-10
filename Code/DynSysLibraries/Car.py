
import numpy as np
from DynSysLibraries.DynSysPID import PID

class Car:
    def __init__(self, targetVelocity, targetDistance = 100):
        self._state = np.matrix([[0.0], [0.0]])
        self._acc = 0.0

        self.pidVel = PID(targetVelocity, 0.2, 0, 0, 1.0)
        self.pidDist = PID(targetDistance, 0.2, 0, 0.5, 1.0)

        self.logPos = [self.getPos()]
        self.logVel = [self.getVel()]
        self.logAcc = [self._acc]

        self._output = 0
        self.simulateResistance = True
        
        rhoDiv2 = 1.29 / 2.0
        C = 0.3 #Drag coeficent
        A = 2.5 #Front area
        Mass = 1500.0

        self._k = rhoDiv2 * C * A / Mass
        self._P = 80_000 / Mass

        self.maxA = 0
    
    def setPos(self, pos):
        self.logPos[-1] = pos
        self._state[0, 0] = pos
    
    def getPos(self):
        return self._state[0, 0]

    def setVel(self, vel):
        self.logVel[-1] = vel
        self._state[1, 0] = vel
    
    def getVel(self):
        return self._state[1, 0]

    def setAcc(self, acc):
        self._acc = acc
        self.logAcc[-1] = acc

    def _regulator(self, dt, nextCar: "Car"):
        if (nextCar):
            dist = nextCar.getPos() - self.getPos()
            outputDist = -self.pidDist.update(dist, dt)
            outputVel = self.pidVel.update(self.getVel(), dt)
            output = min(outputVel, outputDist)
        else:
            output = self.pidVel.update(self.getVel(), dt)

        a = 0.0
        self._output = self._output * a + output * (1 - a)

    def _simulateStateSpace(self, dt):
        AMatrix = np.matrix([[0, 1], [0, -self._k * self.getVel()]])
        usedVelocity = abs(max(20.0, self.getVel()))
        BMatrix = np.matrix([[0], [self._P / usedVelocity]])

        xDot = AMatrix @ self._state + BMatrix * self._output
        self._state += xDot * dt
        self._acc = xDot[1, 0]

    def update(self, dt, nextCar = None):
        self._regulator(dt, nextCar)
        self._simulateStateSpace(dt)

        self.logAcc.append(self._acc)
        self.logVel.append(self.getVel())
        self.logPos.append(self.getPos())