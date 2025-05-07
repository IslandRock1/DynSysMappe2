
from DynSysLibraries.DynSysPID import PID

class Car:
    def __init__(self, targetVelocity, targetDistance = 100):
        self._pos = 0.0
        self._vel = 0.0
        self._acc = 0.0

        # 0.8G's of acceleration is MAX, based on data from https://www.quora.com/How-many-Gs-do-we-feel-driving-a-car
        # I know some cars (Like the polestar) can easily go above 1G when accelerating hard, and probably a bit more
        # when braking. I just picked 0.8G's as a start.

        self.pidVel = PID(targetVelocity, 0.2, 0, 0, 0.3 * 9.81)
        self.pidDist = PID(targetDistance, 0.2, 0, 0.5, 0.3 * 9.81)

        self.logPos = [self._pos]
        self.logVel = [self._vel]
        self.logAcc = [self._acc]

        self.simulateResistance = True
        self.rhoDiv2 = 1.29 / 2.0
        self.C = 0.3 #Drag coeficent
        self.A = 2.5 #Front area
        self.Mass = 1500.0
        self.Crr = 0.015
    
    def setPos(self, pos):
        self._pos = pos
        self.logPos[-1] = pos
    
    def setVel(self, vel):
        self._vel = vel
        self.logVel[-1] = vel
    
    def setAcc(self, acc):
        self._acc = acc
        self.logAcc[-1] = acc

    def regulator(self, dt, nextCar: "Car"):
        self._acc = 0.0
        
        airResistance = self.rhoDiv2 * self.C * self.A * self._vel * self._vel
        airAcc = airResistance / self.Mass

        rollingResistance = self.Crr * self.Mass * 9.81
        rollingAcc = rollingResistance / self.Mass

        if (self.simulateResistance):
            self._acc += -airAcc
            self._acc += -rollingAcc
        
        accVel = self.pidVel.update(self._vel, dt)
        if (nextCar == None):
            self._acc += accVel
            return
        
        dist = nextCar._pos - self._pos
        accDist = -self.pidDist.update(dist)

        self._acc += min(accVel, accDist)

    def update(self, dt = (1 / 60), nextCar = None):
        self.regulator(dt, nextCar)

        self._vel += self._acc * dt
        self._pos += self._vel * dt

        self.logAcc.append(self._acc)
        self.logVel.append(self._vel)
        self.logPos.append(self._pos)