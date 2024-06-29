#!/usr/bin/env python2.7

import numpy as np
from math import *


class pid(object):
    def __init__(self, freq=50, kp=1, ki=0, kd=0, k=None, setpt=np.array([0])):
        """
            Constructor for pid controller class
            @param freq: control loop frequency of controller
            @param kp: Proportional controller gain, defaults to 1
                NOTE: This parameter will be ignored if @param k is supplied
            @param ki: Integral controller gain, defaults to 0
                NOTE: This parameter will be ignored if @param k is supplied
            @param kd: Derivative controller gain, defaults to 0
                NOTE: This parameter will be ignored if @param k is supplied
            @param k: Numpy array of shape (1, 3) or (3,), containing the gains Kp, Ki, and Kd
                NOTE: Defaults to None. @params kp, ki, kd are ignored if this param is supplied
            @param setpt: Setpoint(s) for PID controller.
                NOTE: Setpoint may be changed dynamically using method set_setpt
        """
        if k is None:
            self._k = np.array([kp, ki, kd])
        else:
            self._k = k
        self._k = self._k.reshape((1,3))
        self._f = freq
        self._setPt = setpt.reshape((setpt.shape[0],1))
        self._accErr = np.zeros(self._setPt.shape)
        self._prevErr = np.zeros(self._setPt.shape)

    def set_setpt(self, setpt):
        # assert (self._setPt.shape == setpt.shape), "ERROR: Shape mismatch: shape of new setpoint does not match existing setpoint"
        self._setPt = setpt.reshape((setpt.shape[0],1))

    def output(self, currVal):
        """
            Method to compute control signal values.
            @param currVal: current value of controlled parameter. Can be both scalar and vector
        """
        e = self._setPt - currVal
        self._accErr += (e / self._f)
        dErr = (self._prevErr - e)*self._f
        self._prevErr = e
        if e.shape == (1,1):
            eVec = np.concatenate((e, self._accErr, dErr), axis=0)
        else:
            eVec = np.concatenate((e, self._accErr, dErr), axis=1)
        return np.matmul(self._k , eVec).squeeze()
    
    def debug(self):
        for k in self.__dict__:
            print(k, ':', self.__dict__[k])
            if type(self.__dict__[k])!=int:
                print(k, ' shape: ', self.__dict__[k].shape)
    

if __name__ == "__main__":
    k = np.array([[1, 0.2, 0.5]])
    ctrl0 = pid(k=k)
    ctrl1 = pid(k=k, setpt=np.array([0, 0, 0]))
    x0 = 0.5
    x1 = np.array([0.5, 1, 0.8]).reshape((3, 1))
    x2 = np.array([0.5, 1, 0.8]).reshape((1, 3))
    print(ctrl0.output(x0))
    # ctrl0.debug()
    print(ctrl1.output(x1))
    # ctrl1.debug()
    # ctrl1.set_setpt(x2) # will throw assertion error
