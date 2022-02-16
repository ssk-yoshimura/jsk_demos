# -*- coding: utf-8 -*-

import numpy as np
from Layer import *

class dqn:
    def __init__(self, node_list, out_func=lambda x: x):
        self.out_func = out_func
        self.layerlist = []
        self.node_list = node_list
        self.state_n = node_list[0]
        self.action_n = node_list[-1]
        self.dropout_list = np.zeros(len(node_list)-1)
        for i in range(len(node_list)-1):
            self.layerlist.append(layer(np.random.randn(node_list[i+1], node_list[i])*0.03,
                                        np.random.randn(node_list[i+1])*0.03, 
                                        lambda x : 1/(1 + np.exp(-x)),
                                        lambda y : y*(1-y),
                                        0.5))
        self.set_W_n()

    def set_dropout(self, drop_list):
        self.dropout_list = drop_list
  
    def set_W(self, dev_list):
        for i in range(len(dev_list)):
            self.layerlist[i].W = np.random.standard_normal(self.layerlist[i].W.shape)*dev_list[i]
            self.layerlist[i].b = np.random.standard_normal(self.layerlist[i].b.shape)*dev_list[i]
  
    def set_W_n(self):
        dev_list = []
        for i in range(len(self.layerlist)):
            dev_list.append(1/np.sqrt(self.layerlist[i].W.shape[1]))
            self.set_W(dev_list)
            # print(dev_list)

    def set_func(self, func_list):
        for i in range(len(func_list)):
            self.layerlist[i].f = func_list[i][0]
            self.layerlist[i].df_y = func_list[i][1]
  
    def set_func_relu(self):
        for i in range(len(self.layerlist)-1):
            self.layerlist[i].f = relu()[0]
            self.layerlist[i].df_y = relu()[1]
        self.layerlist[-1].f = identify()[0]
        self.layerlist[-1].df_y = identify()[1]

    def set_func_sigmoid(self):
        for i in range(len(self.layerlist)):
            self.layerlist[i].f = sigmoid()[0]
            self.layerlist[i].df_y = sigmoid()[1]

    def set_l_rate(self, l_list):
        for i in range(len(l_list)):
            self.layerlist[i].eta = l_list[i] 
    
    def set_rate_all(self, l_rate):
        for i in range(len(self.layerlist)):
            self.layerlist[i].eta = l_rate
  
    def forward(self, state): # state:(n,1), n:state size = NN input size
        x = state
        x_record = [] # record x for learning
        for l in self.layerlist:
            x_record.append(x)
            x = l.forward(x)
        x = self.out_func(x) # use out_func
        x_record.append(x)
        action = np.argmax(x, axis=0)
        return x_record, action
    
    def forward_old(self, state):
        x = state
        for l in self.layerlist:
            x = l.forward(x)
        x = self.out_func(x)
        maxQ = np.max(x, axis=0)
        return maxQ

    def collect_init(self):
        self.x_collect = [[] for _ in range(len(self.layerlist)+1)]

    def play(self, state, epsilon):
        self.x_record, self.action_best = self.forward(state)
        if epsilon > np.random.uniform(0,1):
            next_action = np.random.choice(self.action_n, self.action_best.size)
        else:
            next_action = self.action_best
        return next_action
    
    def collect(self):
        for i in range(len(self.x_collect)):
            self.x_collect[i].append(self.x_record[i][:,0])
        # print(self.x_collect)
        return self.x_collect
    
    def collect_end(self):
        self.x_collect_arr = []
        for i in range(len(self.x_collect)):
            self.x_collect_arr.append(np.array(self.x_collect[i]).T)
        # print(self.x_collect_arr)
        return self.x_collect_arr
    
    def update(self, label):
        # layerのxをx_recで書き換える
        for i in range(len(self.layerlist)):
            self.layerlist[i].x_input = self.x_collect_arr[i]
            self.layerlist[i].x_output = self.x_collect_arr[i+1]
        # bp
        c = self.x_collect_arr[-1] - label
        for l in reversed(self.layerlist):
            c = l.bp(c)
        for l in self.layerlist:
            l.update()

def relu(k=1.0):
    def relu_func(x):
        mask = (x<0)
        x[mask] = 0
        return k*x
    return (relu_func, lambda y:k*(y>0))

def identify():
    return (lambda x:x, lambda y:1)

def sigmoid(alpha=1.0):
    return (lambda x:1/(1+np.exp(-alpha*x)), lambda y:alpha*y*(1-y))

def tanh():
    return (lambda x:np.tanh(x), lambda y:(1-y*y))

def softmax(x):
    x_max = np.max(x, axis=0)
    exp_x = np.exp(x - x_max)
    sum_e = np.sum(exp_x, axis=0)
    return exp_x / sum_e
