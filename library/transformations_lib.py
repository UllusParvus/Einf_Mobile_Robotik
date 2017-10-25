#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 24 16:51:26 2017

@author: christoph
"""

import numpy as np


def rot(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]])


def rotx(theta):
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])


def roty(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])


def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])


def rot2trans(r):
    trans = np.vstack((r, np.zeros(r.shape[1])))
    trans = np.hstack((trans, np.zeros((trans.shape[0], 1))))
    trans[trans.shape[0] - 1, trans.shape[1] - 1] = 1
    return trans


def trans(t):
    tl_vec = np.vstack((np.reshape(np.array(t), (np.array(t).shape[0], 1)), 1))
    trans = np.identity(tl_vec.shape[0])
    trans[:, trans.shape[1] - 1:] = tl_vec
    return trans