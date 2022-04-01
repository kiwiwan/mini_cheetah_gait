import numpy as np


def cubicBezier(y0, yf, x):
    assert x >= 0 and x <= 1, 'cubicBezier x need in 0-1'
    yDiff = yf - y0
    bezier = x * x * x + 3 * (x * x * (1 - x))
    return y0 + bezier * yDiff

def cubicBezierFirstDerivative(y0, yf, x):
    assert x >= 0 and x <= 1, 'cubicBezier x need in 0-1'
    yDiff = yf - y0
    bezier = 6 * x * (1 - x)
    return bezier * yDiff

def cubicBezierSecondDerivative(y0, yf, x):
    assert x >= 0 and x <= 1, 'cubicBezier x need in 0-1'
    yDiff = yf - y0
    bezier = 6 - 12 * x
    return bezier * yDiff

class FootSwingTrajectory():
    def __init__(self):
        self._p0 = np.array([0]*3)
        self._pf = np.array([0]*3)
        self._p = np.array([0]*3)
        self._v = np.array([0]*3)
        self._a = np.array([0]*3)
        
        self._height = 0


    def computeSwingTrajectoryBezier(self, phase, swingTime):
        self._p = cubicBezier(self._p0, self._pf, phase)
        self._v = cubicBezierFirstDerivative(self._p0, self._pf, phase) / swingTime
        self._a = cubicBezierSecondDerivative(self._p0, self._pf, phase) / (swingTime * swingTime)

        if phase < 0.5:
            zp = cubicBezier(self._p0[2], self._p0[2] + self._height, phase * 2)
            zv = cubicBezierFirstDerivative(self._p0[2], self._p0[2] + self._height, phase * 2) * 2 / swingTime
            za = cubicBezierSecondDerivative(self._p0[2], self._p0[2] + self._height, phase * 2) * 4 / (swingTime * swingTime)
        else:
            zp = cubicBezier(self._p0[2] + self._height, self._pf[2], phase * 2 - 1)
            zv = cubicBezierFirstDerivative(self._p0[2] + self._height, self._pf[2], phase * 2 - 1) * 2 / swingTime
            za = cubicBezierSecondDerivative(self._p0[2] + self._height, self._pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime)

        self._p[2] = zp
        self._v[2] = zv
        self._a[2] = za
        # print("p2:",self._p0)

    def setInitialPosition(self, p0):
        self._p0 = p0

    def setFinalPosition(self, pf):
        self._pf = pf

    def setHeight(self, h):
        self._height = h

    def getPosition(self):
        return self._p

    def getVelocity(self):
        return self._v

    def getAcceleration(self):
        return self._a

