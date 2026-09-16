class SimulationClock:
    def __init__(self, dt: float = 1.0):
        if dt<=0:
            raise ValueError("dt must be greater than 0")
        self.time=0.0
        self.dt=dt
    def advance(self):
        self.time+=self.dt
        return self.time