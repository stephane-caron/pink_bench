#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

import numpy as np


class RunningMeanStd(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.n = 0
        self.x = 0.0
        self.x2 = 0.0

    def append(self, x):
        self.n += 1
        self.x += x
        self.x2 += x**2

    def average(self):
        assert self.n >= 1
        return self.x / self.n

    def std(self):
        assert self.n > 1
        mu = self.average()
        unbiased = np.sqrt(1.0 + 1.0 / (self.n - 1))
        return unbiased * np.sqrt(self.x2 / self.n - mu**2)
