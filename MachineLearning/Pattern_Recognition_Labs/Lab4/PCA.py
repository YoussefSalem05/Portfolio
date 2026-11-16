import numpy as np

class PCA():
    def __init__(self,  new_dim:int) -> None:
        # hyperparameter representing the number of dimensions after reduction
        self.new_dim = new_dim
        # for standardization
        self.μ:np.ndarray
        self.σ:np.ndarray
        # for PCA
        self.A:np.ndarray

    # x_train is (m,n) matrix where each row is an n-dimensional vector of features
    def fit(self, x_train):
        self.μ = None
        self.σ = None
        # if a column has zero std (useless constant) set σ=1 (skip their standardization)
        self.σ = np.where(self.σ == 0, 1, self.σ)

        z_train = None

        Σ = None

        λs, U = None
        λs, U = λs.real, U.real           # sometimes a zero imaginary part can appear due to approximations

        sorting_inds = None
        λs = None
        U = None

        L = self.new_dim
        self.A = None

        return self

    # x_val is (m,n) matrix where each row is an n-dimensional vector of features
    def transform(self, x_val):
        z_val = (x_val - self.μ) / self.σ
        return None

    def inverse_transform(self, z_val):
        return None

    def fit_transform(self, x_train):
        return self.fit(x_train).transform(x_train)
