import numpy as np
from numpy.linalg import det, inv

class KDEstimator():
    def __init__(self, bump:str='Gauss', bandwidth:str='Silverman') -> None:
        self.bump = bump
        self.bandwidth = bandwidth

        # set during fit
        self.M, self.N = None, None         # number of observations and features in each
        self.x_train = None                 # store training data to use in inference
        self.h = None                       # the actual value of the bandwidth

        # validation
        assert bump in ['Gauss', 'Rect'], f"Only Gauss and Rect bumps are supported but you passed {bump} to KDEstimator"

        bandwidth_is_valid = self.bandwidth in ['Silverman', 'Scott'] or isinstance(self.bandwidth, (int, float))
        assert bandwidth_is_valid, f"bandwidth must be either 'Silverman' or 'Scott' or a number but you passed {self.bandwidth} to KDEstimator"


    def fit(self, x_train):
        self.M, self.N = x_train.shape
        self.x_train = x_train

        M, N = self.M, self.N               # to avoid corrupting eqns with self

        avg_σ = np.mean(np.std(x_train, axis=0))

        if self.bandwidth == 'Silverman':
            self.h = avg_σ * (4/(M*(N+2)))**(1/(N+4))
        elif self.bandwidth =='Scott':
            self.h = avg_σ * M**(-1/(N+4))
        else:
            # it must be an int or float in this case so we use it directly
            self.h = self.bandwidth

        return self

    def g(self, x):
        if self.bump == 'Gauss':
            N = self.N
            π = np.pi
            scale = (2*π)**(N/2)
            Numerator = np.exp(-0.5 * np.einsum('ij,ij->i', x, x))
            return Numerator/scale

        elif self.bump == 'Rect':
            return np.all(np.abs(x) <= 0.5,axis=1).astype(float)


    def ϕ(self, x):
        h, N = self.h, self.N
        return 1/h**N * self.g(x/h)

    def P(self, x):
        scale = 1/(self.M)
        xₘ = self.x_train
        return scale * np.sum(self.ϕ(x - xₘ), axis=0)

    def transform(self, x_val):
        # if x_val is 1D apply P to x_val directly (same line of code)

        return np.apply_along_axis(self.P, axis=1, arr=x_val) if x_val.ndim >= 2 else self.P(x_val)

    def fit_transform(self, x_data):
        # fit on x_data then transform
        return self.fit(x_data).transform(x_data)
