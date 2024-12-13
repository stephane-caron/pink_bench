#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

import uuid
from typing import Optional, Tuple

import meshcat_shapes
import numpy as np
import pink
import pinocchio as pin
from meshcat import Visualizer
from numpy.typing import NDArray
from qpmpc import MPCProblem, solve_mpc
from qpmpc.exceptions import ProblemDefinitionError
from qpmpc.live_plots.live_plot import LivePlot

from .trajectory import Trajectory

GRAVITY = 9.81  # [m] / [s]²
MAX_ZMP_DIST = 100.0  # [m]


def display_foot_area(
    viewer: Visualizer, transform_contact_to_world: pin.SE3, foot_size
):
    prefix = str(uuid.uuid4())
    if viewer is not None:
        meshcat_shapes.point(
            viewer[f"{prefix}_center"],
            radius=0.02,
            color=0x00FF00,
        )
        viewer[f"{prefix}_center"].set_transform(transform_contact_to_world.np)
    for i in range(4):
        u_x = -1 if i < 2 else +1
        u_y = -1 if i % 2 == 0 else +1
        transform_vertex_to_contact = pin.SE3(
            rotation=np.eye(3),
            translation=np.array(
                [u_x * foot_size[0], u_y * foot_size[1], 0.0]
            ),
        )
        transform_vertex_to_world = (
            transform_contact_to_world * transform_vertex_to_contact
        )
        if viewer is not None:
            meshcat_shapes.point(
                viewer[f"{prefix}_{i}"],
                radius=0.01,
                color=0xFF0000,
            )
            viewer[f"{prefix}_{i}"].set_transform(transform_vertex_to_world.np)


def plot_plan(
    live_plot, sampling_period, nb_timesteps, omega, mpc_problem, plan
) -> None:
    horizon_duration = sampling_period * nb_timesteps
    trange = np.linspace(0.0, 0.0 + horizon_duration, nb_timesteps + 1)
    X = plan.states
    zmp_from_state = np.array([1.0, 0.0, -1.0 / omega**2])
    zmp = X.dot(zmp_from_state)
    pos = X[:, 0]
    zmp_min = [
        x[0] if abs(x[0]) < MAX_ZMP_DIST else None
        for x in mpc_problem.ineq_vector
    ]
    zmp_max = [
        -x[1] if abs(x[1]) < MAX_ZMP_DIST else None
        for x in mpc_problem.ineq_vector
    ]
    zmp_min.append(zmp_min[-1])
    zmp_max.append(zmp_max[-1])
    live_plot.update_line("pos", trange, pos)
    live_plot.update_line("zmp", trange, zmp)
    live_plot.update_line("zmp_min", trange, zmp_min)
    live_plot.update_line("zmp_max", trange, zmp_max)


def build_mpc_problem(
    nb_timesteps: int,
    omega: float,
    sampling_period: float,
):
    """Build the model predictive control problem.

    For details on this problem and how open-loop model predictive control
    was used in the LIPM walking controller, see "Stair Climbing
    Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance
    Control" (Caron et al., 2019).
    """
    T = sampling_period
    state_matrix = np.array(
        [
            [1.0, T, T**2 / 2.0],
            [0.0, 1.0, T],
            [0.0, 0.0, 1.0],
        ]
    )
    input_matrix = np.array(
        [
            T**3 / 6.0,
            T**2 / 2.0,
            T,
        ]
    ).reshape((3, 1))
    zmp_from_state = np.array([1.0, 0.0, -1.0 / omega**2])
    ineq_matrix = np.array([+zmp_from_state, -zmp_from_state])
    return MPCProblem(
        transition_state_matrix=state_matrix,
        transition_input_matrix=input_matrix,
        ineq_state_matrix=ineq_matrix,
        ineq_input_matrix=None,
        ineq_vector=None,
        initial_state=None,
        goal_state=None,
        nb_timesteps=nb_timesteps,
        terminal_cost_weight=1.0,
        stage_state_cost_weight=None,
        stage_input_cost_weight=1e-3,
    )


class PhaseStepper:
    def __init__(
        self,
        dsp_duration: float,
        nb_timesteps: int,
        sampling_period: float,
        ssp_duration: float,
        strides: NDArray[float],
    ):
        nb_dsp_steps = int(round(dsp_duration / sampling_period))
        nb_ssp_steps = int(round(ssp_duration / sampling_period))
        if 2 * (nb_dsp_steps + nb_ssp_steps) < nb_timesteps:
            raise ProblemDefinitionError(
                "there are more than two steps in the receding horizon"
            )

        # Skip edge cases of separate initial and final DSP durations (1/2):
        # here we set the initial index in the middle of the first SSP phase in
        # the receding horizon, thus creating a short first SSP phase.
        self.__initial_index = 5

        self.index = self.__initial_index
        self.nb_dsp_steps = nb_dsp_steps
        self.nb_ssp_steps = nb_ssp_steps
        self.nb_timesteps = nb_timesteps
        self.stride_index = 0
        self.strides = strides

    def reset(self):
        self.index = self.__initial_index
        self.stride_index = 0

    def advance(self):
        self.index += 1
        if self.index >= self.nb_dsp_steps + self.nb_ssp_steps:
            self.index = 0

    def advance_stride(self):
        self.stride_index = (self.stride_index + 1) % len(self.strides)

    def get_nb_steps(self):
        offset = self.index
        nb_init_dsp_steps = max(0, self.nb_dsp_steps - offset)
        offset = max(0, offset - self.nb_dsp_steps)
        nb_init_ssp_steps = max(0, self.nb_ssp_steps - offset)
        offset = max(0, offset - self.nb_ssp_steps)

        remaining = self.nb_timesteps - nb_init_dsp_steps - nb_init_ssp_steps
        nb_next_dsp_steps = min(self.nb_dsp_steps, remaining)
        remaining = max(0, remaining - self.nb_dsp_steps)
        nb_next_ssp_steps = min(self.nb_ssp_steps, remaining)
        remaining = max(0, remaining - self.nb_ssp_steps)
        nb_last_dsp_steps = min(self.nb_dsp_steps, remaining)
        remaining = max(0, remaining - self.nb_dsp_steps)
        nb_last_ssp_steps = min(self.nb_ssp_steps, remaining)
        remaining = max(0, remaining - self.nb_ssp_steps)
        if remaining > 0:
            raise ProblemDefinitionError(
                "there are more than two steps in the receding horizon"
            )

        return (
            nb_init_dsp_steps,
            nb_init_ssp_steps,
            nb_next_dsp_steps,
            nb_next_ssp_steps,
            nb_last_dsp_steps,
            nb_last_ssp_steps,
        )

    def get_next_foot_pose(self, transform_foot_to_world: pin.SE3) -> float:
        transform_stride_to_foot = pin.SE3(
            rotation=np.eye(3),
            translation=self.strides[self.stride_index],
        )
        return transform_foot_to_world * transform_stride_to_foot

    def get_next_foot_on_axis(self, foot_pos: float, axis: int) -> float:
        return foot_pos + self.strides[self.stride_index][axis]

    def get_last_foot_on_axis(self, foot_pos: float, axis: int) -> float:
        upcoming_stride = (self.stride_index + 1) % len(self.strides)
        next_foot_pos = self.get_next_foot_on_axis(foot_pos, axis)
        return next_foot_pos + self.strides[upcoming_stride][axis]


class LIPMWalkingTrajectory(Trajectory):
    def __init__(
        self,
        task: pink.tasks.FrameTask,
        dsp_duration: float,
        foot_size: float,
        foot_spacing: float,
        lateral_offset: float,
        left_foot_frame: str,
        nb_timesteps: int,
        sampling_period: float,
        ssp_duration: float,
        stride: float,
        transform_target_to_root: pin.SE3,
        annotate_viz: bool = True,
        init_vel_scale: Tuple[float, float] = (0.2, 0.2),
        plot_axis: Optional[int] = None,
    ):
        """Prepare center-of-mass trajectory.

        Args:
            task: Frame task targetting the root joint.
            dsp_duration: Duration of double-support phases, in seconds.
            foot_size: Foot dimensions (half-length, half-width), in meters.
            init_vel_scale: Initial velocity scaling (no unit).
            lateral_offset: Lateral offset between initial foot frames and foot
                targets (see SwingFootTrajectory).
            left_foot_frame: Name of left-foot frame in the robot model.
            nb_timesteps: Number of discrete time steps in the receding-horizon
                optimal control problem.
            sampling_period: Discrete time steps of the optimal control
                problem, in seconds.
            foot_spacing: Distance between left and right footholds, in meters.
            ssp_duration: Duration of single-support phases, in seconds.
            stride: Forward stepping distance, in meters.
            transform_target_to_root: Offset applied between the root frame and
                the task target frame.
            plot_axis: If set, plot open-loop MPC trajectories.
        """
        super().__init__(task)
        self.phase = PhaseStepper(
            dsp_duration=dsp_duration,
            nb_timesteps=nb_timesteps,
            sampling_period=sampling_period,
            ssp_duration=ssp_duration,
            strides=np.array(
                [
                    [stride / 2, -foot_spacing, 0.0],
                    [stride / 2, +foot_spacing, 0.0],
                ]
            ),
        )

        self.__goal_pos = None
        self.__live_plot = None
        self.annotate_viz = annotate_viz
        self.dsp_duration = dsp_duration
        self.foot_size = foot_size
        self.foot_spacing = foot_spacing
        self.init_vel_scale = init_vel_scale
        self.lateral_offset = lateral_offset
        self.left_foot_frame = left_foot_frame
        self.nb_timesteps = nb_timesteps
        self.plot_axis = None
        self.sampling_period = sampling_period
        self.ssp_duration = ssp_duration
        self.stride = stride
        self.transform_target_to_root = transform_target_to_root

    def init_plot(self, plot_axis: int) -> None:
        horizon_duration = self.sampling_period * self.nb_timesteps
        self.__live_plot = LivePlot(
            xlim=(0, horizon_duration + self.sampling_period),
            ylim=(-1, 1),
        )
        self.__live_plot.add_line("pos", "b-")
        self.__live_plot.add_line("cur_pos", "bo", lw=2)
        self.__live_plot.add_line("cur_zmp", "ro", lw=2)
        self.__live_plot.add_line("goal_pos", "ko", lw=2)
        self.__live_plot.add_line("zmp", "r-")
        self.__live_plot.add_line("zmp_min", "g:")
        self.__live_plot.add_line("zmp_max", "b:")
        self.__goal_pos = np.array([0.0, 0.0])
        self.plot_axis = plot_axis

    def reset(self, configuration: pink.Configuration, viewer: Visualizer):
        super().reset(configuration, viewer)
        self.phase.reset()

        transform_root_to_world = configuration.get_transform_frame_to_world(
            self.task.frame
        )
        transform_lateral_offset_to_foot = pin.SE3(
            rotation=np.eye(3),
            translation=np.array([0.0, self.lateral_offset, 0.0]),
        )
        transform_left_foot_orig_to_world = (
            configuration.get_transform_frame_to_world(self.left_foot_frame)
            * transform_lateral_offset_to_foot
        )
        transform_left_foot_to_world = pin.SE3(  # world-aligned frame
            rotation=np.eye(3),
            translation=transform_left_foot_orig_to_world.translation,
        )
        transform_root_to_foot = transform_left_foot_to_world.actInv(
            transform_root_to_world
        )
        com_height = transform_root_to_foot.translation[2]
        assert com_height > 0.0
        omega = np.sqrt(GRAVITY / com_height)

        if self.annotate_viz:
            display_foot_area(
                viewer, transform_left_foot_to_world, self.foot_size
            )

        # Skip edge cases of separate initial and final DSP durations (2/2):
        # here we set the initial ZMP at the center of the initial foothold,
        # and the initial DCM halfway. See the LIPM walking controller and its
        # configuration for details on initial/final DSP phases.
        foot_pos = transform_left_foot_to_world.translation
        root_pos = transform_root_to_world.translation
        init_pos = root_pos[:2]
        init_vel = omega * np.array(
            [
                self.init_vel_scale[0] * self.stride,
                self.init_vel_scale[1] * self.foot_spacing,
            ]
        )
        init_accel = -(omega**2) * (foot_pos - root_pos)[:2]
        state = np.vstack([init_pos, init_vel, init_accel]).T  # (2, 3)

        mpc_problem = build_mpc_problem(
            self.nb_timesteps, omega, self.sampling_period
        )

        self.mpc_problem = mpc_problem
        self.nb_steps = 0
        self.omega = omega
        self.plan = [None, None]
        self.state = state
        self.substep_duration = -1.0
        self.transform_contact_to_world = transform_left_foot_to_world
        self.transform_root_to_world = transform_root_to_world
        self.viewer = viewer

    def step(self, dt: float):
        """Update target frame (in place)."""
        if self.substep_duration <= 0.0:
            self.replan()
        for axis in (0, 1):
            jerk = self.plan[axis].inputs[0][0]
            self.integrate_state(axis, jerk, dt)
        self.substep_duration -= dt
        self.transform_root_to_world.translation[:2] = self.state[:, 0]
        transform_target_to_world = self.transform_root_to_world * (
            self.transform_target_to_root
        )
        self.task.set_target(transform_target_to_world)

        if self.plot_axis is not None:
            self.update_plot()
        self.nb_steps += 1
        if self.nb_steps % 30 == 0 and self.annotate_viz and self.viewer:
            meshcat_shapes.point(
                self.viewer[f"target_{self.nb_steps}"],
                radius=0.015,
                color=0xFF0000,
            )
            self.viewer[f"target_{self.nb_steps}"].set_transform(
                transform_target_to_world.np
            )

    def update_plot(self):
        horizon_duration = self.sampling_period * self.nb_timesteps
        cur_pos = self.state[self.plot_axis][0]
        cur_acc = self.state[self.plot_axis][2]
        cur_zmp = cur_pos - cur_acc / self.omega**2
        goal_pos = self.__goal_pos[self.plot_axis]
        subt = self.sampling_period - self.substep_duration
        self.__live_plot.update_line("cur_pos", [subt], [cur_pos])
        self.__live_plot.update_line("cur_zmp", [subt], [cur_zmp])
        self.__live_plot.update_line(
            "goal_pos", [horizon_duration], [goal_pos]
        )
        self.__live_plot.update()

    def replan(self):
        for axis in (0, 1):
            self.mpc_problem.update_initial_state(self.state[axis])
            self.update_goal_and_constraints(axis)
            self.plan[axis] = solve_mpc(self.mpc_problem, solver="clarabel")
            if axis == self.plot_axis:
                plot_plan(
                    self.__live_plot,
                    self.sampling_period,
                    self.nb_timesteps,
                    self.omega,
                    self.mpc_problem,
                    self.plan[self.plot_axis],
                )
        self.phase.advance()
        if self.phase.index == 0:
            self.transform_contact_to_world = self.phase.get_next_foot_pose(
                self.transform_contact_to_world
            )
            if self.annotate_viz:
                display_foot_area(
                    self.viewer,
                    self.transform_contact_to_world,
                    self.foot_size,
                )
            self.phase.advance_stride()
        self.substep_duration = self.sampling_period

    def update_goal_and_constraints(self, axis: int):
        (
            nb_init_dsp_steps,
            nb_init_ssp_steps,
            nb_next_dsp_steps,
            nb_next_ssp_steps,
            nb_last_dsp_steps,
            nb_last_ssp_steps,
        ) = self.phase.get_nb_steps()
        cur_foot_pos = self.transform_contact_to_world.translation[axis]
        next_foot_pos = self.phase.get_next_foot_on_axis(cur_foot_pos, axis)
        last_foot_pos = self.phase.get_last_foot_on_axis(cur_foot_pos, axis)
        cur_max = cur_foot_pos + 0.5 * self.foot_size[axis]
        cur_min = cur_foot_pos - 0.5 * self.foot_size[axis]
        next_max = next_foot_pos + 0.5 * self.foot_size[axis]
        next_min = next_foot_pos - 0.5 * self.foot_size[axis]
        last_max = last_foot_pos + 0.5 * self.foot_size[axis]
        last_min = last_foot_pos - 0.5 * self.foot_size[axis]
        self.mpc_problem.ineq_vector = (
            [np.array([+MAX_ZMP_DIST, +MAX_ZMP_DIST])] * nb_init_dsp_steps
            + [np.array([+cur_max, -cur_min])] * nb_init_ssp_steps
            + [np.array([+MAX_ZMP_DIST, +MAX_ZMP_DIST])] * nb_next_dsp_steps
            + [np.array([+next_max, -next_min])] * nb_next_ssp_steps
            + [np.array([+MAX_ZMP_DIST, +MAX_ZMP_DIST])] * nb_last_dsp_steps
            + [np.array([+last_max, -last_min])] * nb_last_ssp_steps
        )
        goal_pos = last_foot_pos if nb_last_dsp_steps > 0 else next_foot_pos
        self.mpc_problem.update_goal_state(np.array([goal_pos, 0.0, 0.0]))
        if self.plot_axis is not None:
            self.__goal_pos[axis] = goal_pos

    def integrate_state(self, axis: int, jerk: float, dt: float) -> np.ndarray:
        """Integrate state (pos, vel, accel) with constant jerk.

        Args:
            axis: Index of the axis (0 for x, 1 for y) to integrate.
            jerk: Constant jerk to integrate.
            dt: Duration to integrate for, in seconds.

        Returns:
            State after integration.
        """
        p_0, v_0, a_0 = self.state[axis]
        self.state[axis, 0] = p_0 + dt * (v_0 + dt * (a_0 / 2 + dt * jerk / 6))
        self.state[axis, 1] = v_0 + dt * (a_0 + dt * (jerk / 2))
        self.state[axis, 2] = a_0 + dt * jerk
