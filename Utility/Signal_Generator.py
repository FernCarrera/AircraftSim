from abc import abstractmethod
from numpy import vectorize, float64, sin, pi

def vectorize_float(method):
    vect_method = vectorize(method,otypes=[float64])
    def wrapper(self,*args,**kwargs):
        return vect_method(self,*args,**kwargs)

    return wrapper

class Control:

    @abstractmethod
    def _fun(self, t):
        raise NotImplementedError

    def __call__(self, t):
        r = self._fun(t)
        if r.size == 1:
            return float(r)
        else:
            return r

    def __add__(self, other):
        control = Control()
        #control._fun = lambda t: self(t) + other(t)
        control._vec_fun = vectorize(control._fun, otypes=[float64])
        return control

    def __sub__(self, other):
        control = Control()
        #control._fun = lambda t: self(t) - other(t)
        control._vec_fun = vectorize(control._fun, otypes=[float64])
        return control

    def __mul__(self, other):
        control = Control()
        #control._fun = lambda t: self(t) * other(t)
        control._vec_fun = vectorize(control._fun, otypes=[float64])
        return control

class Constant(Control):

    def __init__(self, offset=0):
        self.offset = offset

    @vectorize_float
    def _fun(self, t):
        return self.offset

class Doublet(Control):

    def __init__(self, t_init, T, A, offset=0):
        self.t_init = t_init
        self.T = T
        self.A = A
        self.offset = offset

        self.t_fin1 = self.t_init + self.T / 2
        self.t_fin2 = self.t_init + self.T

    @vectorize_float
    def _fun(self, t):
        value = self.offset

        if self.t_init <= t < self.t_fin1:
            value += self.A / 2
        elif self.t_fin1 < t <= self.t_fin2:
            value -= self.A / 2
        return value