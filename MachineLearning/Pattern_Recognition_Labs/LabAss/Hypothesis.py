import numpy as np

class HypothesisFunction:
    def __init__(self, l, m, k):
        self.l = None
        self.m = None
        self.k = None

        self.Wh = None
        self.Wo = None

        self.bo = None
        self.bh = None

    def forward(self, x):
        # Ensure input shape matches input size
        assert x.shape[0] == self.l, f"Your input must be consistent the value l={self.l}"

        a = None

        y = None


        return y, a

    def double_forward(self, x1, x2):
        # Perform forward function for the two inputs
        y1, _ = self.forward(x1)
        y2, _ = self.forward(x2)

        z = None

        z_bar = None

        return z_bar


    def count_params(self):
        num_params = None
        total_params = None
        return total_params
