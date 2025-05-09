
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
            self._output = min(outputVel, outputDist)
            return
        else:
            self._output = self.pidVel.update(self._vel, dt)

    def _simulate(self):
        force = 0.0
        
        airResistance = self.rhoDiv2 * self.C * self.A * self._vel * self._vel
        rollingResistance = self.Crr * self.Mass * 9.81

        if (self.simulateResistance):
            force += -airResistance
            force += -rollingResistance
        
        carForce = (self._power * self._output) / (self._vel + 0.1) # + 0.1 to not div by zero
        
        force += carForce
        self._acc = force / self.Mass

    def update(self, dt = (1 / 60), nextCar = None):
        self._regulator(dt, nextCar)
        self._simulate()

        self._vel += self._acc * dt
        self._pos += self._vel * dt

        self.logAcc.append(self._acc)
        self.logVel.append(self._vel)
        self.logPos.append(self._pos)