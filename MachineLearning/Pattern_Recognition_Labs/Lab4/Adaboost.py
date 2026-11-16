import numpy as np
from sklearn.tree import DecisionTreeClassifier

class Adaboost:
    def __init__(self, T, random_state=42):
        self.T = T
        self.weak_clfs = [DecisionTreeClassifier(max_depth=1, random_state=random_state) for _ in range(T)]
        self.αs = []

    def fit(self, x_train, y_train):

        m = x_train.shape[0]

        W = None                            # should have shape (m,)

        # loop over the boosting iterations
        for t, weak_clf in enumerate(self.weak_clfs):

            # read the docs of the fit method in sklearn.tree.DecisionTreeClassifier to see how the weights can be passed

            hₜ = weak_clf.predict
            Iₜ = None

            errₜ =  None

            αₜ = None
            self.αs.append(αₜ)

            W = None

            W = None

        return self

    def predict(self, x_val):
        weighted_opinions = np.array(None)     # Use zip
        # Now have T evaluations of x_val each weighted (multiplied) by the corresponding alpha,
        # so as per the formula we only need to take the sign of the sum of the different evaluations
        return np.sign(np.sum(weighted_opinions, axis=0))

    def score(self, x_val, y_val):
        y_pred = self.predict(x_val)
        return np.mean(y_pred == y_val)
