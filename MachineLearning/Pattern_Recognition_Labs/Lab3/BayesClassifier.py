import numpy as np # type: ignore
from typing import Callable, Optional
from numpy.linalg import det, inv # type: ignore
import copy

class BayesClassifier():
    def __init__(self, mode:str='QDA', *, kde_config=None, density=None, estimate_params=None):
        # the following will be set upon fit
        self.labels: np.ndarray

        # model learnable parameters will be set upon fit
        self.means: np.ndarray
        self.covs: np.ndarray
        self.priors: np.ndarray
        self.weighted_cov: np.ndarray         # LDA only

        # for KDE
        self.kde_config = kde_config
        self.kde_list: list = []     # contain config for each class

        # for custom
        self.density: Optional[Callable] = density
        self.estimate_params: Optional[Callable] = estimate_params
        self.density_params_list: list[dict] = []

        # mode
        assert mode in ['QDA', 'LDA', 'GNB', 'KDE', 'GMM', 'Custom'],\
        f"Mode must be one of 'QDA', 'LDA', 'Naive', KDE', 'GMM', 'Custom' but you passed {mode}"
        self.mode: str = mode

    # fit calls the appropriate fit method depending on mode [important function]
    def fit(self, x_train: np.ndarray, y_train: np.ndarray):
        self.labels = np.unique(y_train)                # common among all fit methods
        self.priors = np.empty((len(self.labels),))     # will be set within the fit methods
        match self.mode:
            case 'QDA'| 'LDA'| 'GNB':
               self.fit_qda_and_friends(x_train, y_train)
            case 'KDE':
                self.fit_kde(x_train, y_train)
            case 'GMM':
                self.fit_gmm(x_train, y_train)
            case _:
                self.fit_custom_density(x_train, y_train)
        return self

    # fit using Kernel Density Estimation
    def fit_kde(self, x_train: np.ndarray, y_train: np.ndarray):
        # make identical copies of the passed kde instance
        self.kde_list = [copy.deepcopy(self.kde_config) for _ in range(len(self.labels))]
        for k, label in enumerate(self.labels):
            x_train_given_class = x_train[y_train == label]
            self.priors[k] = x_train_given_class.shape[0] / x_train.shape[0]
            self.kde_list[k].fit(x_train_given_class)

    # fit using Gaussian Mixture Models
    def fit_gmm(self, x_train: np.ndarray, y_train: np.ndarray):
        raise NotImplementedError("Gaussian mixture mode is unavailable")

    # fit using custom density
    def fit_custom_density(self, x_train: np.ndarray, y_train: np.ndarray):
        for k, label in enumerate(self.labels):
            x_train_given_class = x_train[y_train == label]
            self.priors[k] = x_train_given_class.shape[0] / x_train.shape[0]
            assert self.estimate_params is not None, "Please provide an estimate_params function for your custom density."
            estimated_params = self.estimate_params(x_train_given_class)
            self.density_params_list.append(estimated_params)


    # fit using QDA, LDA and GNB
    def fit_qda_and_friends(self, x_train: np.ndarray, y_train: np.ndarray):
        # setup empty numpy structures
        N = x_train.shape[1]
        self.means = np.empty((len(self.labels), N))
        self.covs = np.empty((len(self.labels), N, N))

        # compute model parameters
        for k, label in enumerate(self.labels):
            x_train_given_class = x_train[y_train == label]
            self.means[k] = np.mean(x_train_given_class, axis=0)
            self.covs[k] = np.cov(x_train_given_class.T, bias=(self.mode != 'QDA'))
            self.priors[k] = x_train_given_class.shape[0] / x_train.shape[0]

        # handle special cases
        if self.mode == 'LDA':
            self.weighted_cov = np.sum(self.covs * self.priors.reshape(-1, 1, 1), axis=0)
            del self.covs
        elif self.mode == 'GNB':
            self.covs = np.array([cov * np.eye(cov.shape[0]) for cov in self.covs])
        else:
            return

    # Computes P(X|C=k) where k is the index of the class and is passed in predict_proba_x as seen below
    def P(self, x:np.ndarray, k:int)->float:
        match self.mode:
            case 'QDA'| 'LDA'| 'GNB':
                μ = self.means[k]
                Σ = self.covs[k] if self.mode!="LDA" else self.weighted_cov
                π, N = np.pi, len(μ)
                scale = (2 * π)**(N/2) * det(Σ)**(0.5)
                P = np.exp(-0.5 * ( (x - μ).T @ inv(Σ) @  (x- μ))) / scale
            case 'KDE':
                P = self.kde_list[k].transform(x.reshape(1,-1))[0]
            case 'GMM':
                P = None
                raise NotImplementedError("Gaussian mixture mode is unavailable")
            case 'Custom':
                assert self.density is not None, "Please provide an density function for your custom density."
                params = self.density_params_list[k]
                P = self.density(x,params)
        return P

    # Computes [P(X|C=0) P(X|C=1) ... P(X|C=K)]
    def predict_proba_x(self, x:np.ndarray)->np.ndarray:
        prob_product = np.array([self.P(x, k) * prior for k, prior in enumerate(self.priors)])
        return prob_product/np.sum(prob_product)

    # Applies predict_proba_x to each row in x_val
    def predict_proba(self, x_val:np.ndarray)->np.ndarray:
        return np.apply_along_axis(self.predict_proba_x, axis=1, arr=x_val)

    # Transforms probabilities into predicted labels by maximizing probability of correctness
    def predict(self, x_val:np.ndarray)->np.ndarray:
        y_pred_prob = self.predict_proba(x_val)
        y_pred_inds = np.argmax(y_pred_prob, axis=1)
        # replace each prediction with label in self.labels
        y_pred = self.labels[y_pred_inds]
        return y_pred

    # Computes accuracy
    def score(self, x_val:np.ndarray, y_val:np.ndarray)->float:
        y_pred = self.predict(x_val)
        acc = np.sum(y_pred == y_val)/len(y_val)
        return round(acc, 4)
