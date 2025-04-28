
from DynSysLibraries.DynSysPID import PID

class Car:
    def __init__(self, targetVelocity, targetDistance = 100):
        self._pos = 0
        self._vel = 0
        self._acc = 0

        self.pidVel = PID(targetVelocity, 0.2, 0, 0)
        self.pidDist = PID(targetDistance, 0.2, 0, 0)

        self.logPos = [self._pos]
        self.logVel = [self._vel]
        self.logAcc = [self._acc]
    
    def setPos(self, pos):
        self._pos = pos
        self.logPos[-1] = pos
    
    def setVel(self, vel):
        self._vel = vel
        self.logVel[-1] = vel
    
    def setAcc(self, acc):
        self._acc = acc
        self.logAcc[-1] = acc

    def regulator(self, dt, nextCar):
        accVel = self.pidVel.update(self._vel, dt)
        if (nextCar == None):
            self._acc = accVel
            return
        
        dist = nextCar._pos - self._pos
        accDist = -self.pidDist.update(dist)

        self._acc = min(accVel, accDist)

    def update(self, dt = (1 / 60), nextCar = None):
        self.regulator(dt, nextCar)

        self._vel += self._acc * dt
        self._pos += self._vel * dt

        self.logAcc.append(self._acc)
        self.logVel.append(self._vel)
        self.logPos.append(self._pos)