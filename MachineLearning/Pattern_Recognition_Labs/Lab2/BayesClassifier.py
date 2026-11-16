import numpy as np

class BayesClassifier():
    def __init__(self, mode:str='QDA'):
        # the class labels (will be set upon fit)
        self.labels = []
        # model learnable parameters will be set upon fit
        self.means = []
        self.covs = []
        self.priors = []
        self.weighted_cov = None        # in case of LDA only

        # assert goes here
        assert mode in ['QDA', 'LDA', 'Naive'], "mode must be one of ['QDA', 'LDA', 'Naive']"
        self.mode = mode

    def fit(self, x_train: np.ndarray, y_train: np.ndarray):
        self.labels = np.unique(y_train)
        for label in self.labels:
            x_train_given_class = x_train[y_train == label]
            class_mean = np.mean(x_train_given_class, axis=0) #One big row, if one then one big column
            self.means.append(class_mean)
            # Check the documentation of np.cov and what layout it assumes about the input
            # Set bias=False in case of QDA only and use a one-line if for that
            class_cov = np.cov(x_train_given_class, rowvar=False, bias=(self.mode!='QDA'))
            self.covs.append(class_cov)
            class_prior = len(x_train_given_class) / len(x_train)
            self.priors.append(class_prior)

        self.means = np.array(self.means)
        self.covs = np.array(self.covs)
        self.priors = np.array(self.priors)

        # In one line, sum the covariance matrices for each class weighted (multiplied) by their priors
        if self.mode == 'LDA':
            self.weighted_cov = np.sum([cov * prior for cov, prior in zip(self.covs, self.priors)], axis=0)
            # reset self.covs to an empty list to save memory as we no longer need it
            self.covs = []
        elif self.mode == 'Naive':
            # diagonalize the covariance matrices in one line
            self.covs = np.array([np.diag(np.diag(c)) for c in self.covs])
        # In case of QDA, we maintain the general setup and just return
        else:
             return

    @staticmethod
    def N(x:np.ndarray, μ:np.ndarray, Σ:np.ndarray)->float:
        n=len(x)
        scale = 1/np.sqrt((2*np.pi)**n * np.linalg.det(Σ))                              # start by defining the denominator
        prob = np.exp(-0.5*(x-μ).T @ np.linalg.inv(Σ) @ (x-μ)) * scale     # then compute the exponent and multiply by the scale
        return prob

    def predict_proba_x(self, x:np.ndarray)->np.ndarray:
        if self.mode != 'LDA':
            prob_product = np.array([self.N(x, mean, cov) * prior for mean, cov, prior in zip(self.means, self.covs, self.priors)])
        else:
            prob_product = np.array([self.N(x, mean, self.weighted_cov) * prior for mean, prior in zip(self.means, self.priors)])
        return prob_product/np.sum(prob_product)

    def predict_proba(self, x_val:np.ndarray)->np.ndarray:
        # given x_val of dimensions (m,n) apply predict_proba_x to each row (point x) to return array of probabilities (m, k)
        return np.apply_along_axis(self.predict_proba_x, axis=1, arr=x_val)


    def predict(self, x_val:np.ndarray)->np.ndarray:
        y_pred_prob = self.predict_proba(x_val)
        y_pred_inds = np.argmax(y_pred_prob, axis=1)
        # replace each prediction with label in self.labels
        y_pred = self.labels[y_pred_inds]
        return y_pred

    def score(self, x_val:np.ndarray, y_val:np.ndarray)->float:
        y_pred = self.predict(x_val)
        acc = np.mean(y_pred == y_val)
        return acc
