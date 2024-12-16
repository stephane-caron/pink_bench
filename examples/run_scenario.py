#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Run a given scenario from the library."""

import argparse

import qpsolvers

import pink_bench

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scenario",
        help="scenario to run from the pink_bench library",
        choices=list([name for name in pink_bench.SCENARIOS.keys()]),
    )
    parser.add_argument(
        "--dt",
        help="Timestep in seconds between differential IK problems",
        type=float,
        default=0.005,
    )
    parser.add_argument(
        "--qpsolver",
        help="solver for the QP-based approach",
        default="proxqp",
        choices=qpsolvers.available_solvers,
    )
    args = parser.parse_args()
    pink_bench.play_scenario(
        name=args.scenario,
        dt=args.dt,
        qpsolver=args.qpsolver,
    )
