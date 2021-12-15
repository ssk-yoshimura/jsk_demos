#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np

# k近傍法で、立ったままのポーズか、ものを指しているポーズか判別する

def k_nearest(k,test_data):
    data = np.loadtxt('testpose.csv')
    print(data)

k_nearest(10, 0)
