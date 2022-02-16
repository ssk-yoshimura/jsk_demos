# -*- coding: utf-8 -*-

import numpy as np

class layer:
    # W,b,f,u,x_output,delta,eta # node  input: n output: m,  batch l
    def __init__(self, W, b, f, df_y, eta):
        self.W = W # (m,n)
        self.b = b # (m)
        self.f = f 
        self.df_y = df_y
        self.eta = eta
        self.m = self.W.shape[0]
        self.n = self.W.shape[1]

    def forward_old(self, x_input): # x_input: (n,l), x_output,u: (m,l)
        self.x_input = x_input
        self.l = self.x_input.shape[1]
        self.u = (np.dot(self.W, x_input).T + self.b).T
        self.x_output = self.f(self.u)
        return self.x_output

    def forward(self, x_input, dropout_p = 0.0): # x_input: (n,l), x_output,u: (m,l)
        self.x_input = x_input
        self.l = self.x_input.shape[1]
        self.u = (np.dot(self.W, x_input).T + self.b).T
        self.x_output = self.f(self.u)
        if dropout_p != 0.0:
            dropout_mask = np.random.random_sample(self.x_output.shape)
            dropout_mask[dropout_mask < dropout_p] = 0.0
            dropout_mask[dropout_mask > 0.0] = 1.0
            self.x_output = self.x_output * dropout_mask
        return self.x_output

    def bp(self, c): # c: (m,l)
        self.delta = c * self.df_y(self.x_output); # (m,l)
        return np.dot(self.W.T, self.delta) # (n,l)

    def update(self):
        self.W = self.W - self.eta * np.dot(self.delta, self.x_input.T) / self.l
        self.b = self.b - self.eta * np.mean(self.delta, axis=1)
