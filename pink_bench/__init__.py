#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Library of robot motions with inverse kinematics and predictive control."""

from .library import scenarios
from .scenario import Scenario
from .scene import Scene

__version__ = "0.1.0"

__all__ = [
    "Scenario",
    "Scene",
    "scenarios",
]
