import numpy as np

class Gait():
    def __init__(self, nSegment, offset, durations, name):
        self._nIterations = nSegment
        self._offsets = offset
        self._durations = durations
        self.name = name

        self._mpc_table = [0]*nSegment*4
        self._offsetsFloat = offset / nSegment
        self._durationsFloat = durations / nSegment
        self._stance = durations[0]
        self._swing = nSegment - durations[0]

    def getContactState(self):
        progress = self._phase - self._offsetsFloat

        for i in range(4):
            if progress[i] < 0:
                progress[i] += 1.
            if progress[i] > self._durationsFloat[i]:
                progress[i] = 0.
            else:
                progress[i] = progress[i] / self._durationsFloat[i]

        return progress;

    def getSwingState(self):
        swing_offset = self._offsetsFloat + self._durationsFloat
        for i in range(4):
            if swing_offset[i] > 1:
                swing_offset[i] -= 1.


        swing_duration = 1. - self._durationsFloat
        progress = self._phase - swing_offset
        for i in range(4):
            if progress[i] < 0:
                progress[i] += 1.
            if progress[i] >= swing_duration[i]:
                progress[i] = 0.
            else:
                progress[i] = progress[i] / swing_duration[i]

        return progress;

    def getMpcTable(self):
        for i in range(self._nIterations):
            iter = (i + self._iteration + 1) % self._nIterations
            progress = iter - self._offsets
            for j in range(4):
                if progress[j] < 0:
                    progress[j] += self._nIterations
                if progress[j] < self._durations[j]:
                    self._mpc_table[i*4 + j] = 1
                else:
                    self._mpc_table[i*4 + j] = 0

        return self._mpc_table

    def setIterations(self, iterationsPerMPC, currentIteration):
        self._iteration = (currentIteration / iterationsPerMPC) % self._nIterations
        self._phase = (currentIteration % (iterationsPerMPC * self._nIterations)) / (iterationsPerMPC * self._nIterations)


    def getCurrentGaitPhase(self):
        return self._iteration

    def getCurrentSwingTime(self, dtMPC, leg):
        return dtMPC * self._swing

    def getCurrentStanceTime(self, dtMPC, leg):
        return dtMPC * self._stance

