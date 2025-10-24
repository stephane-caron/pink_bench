#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Tests for utility functions."""

import numpy as np
import pytest

from pink_bench.utils import RunningMeanStd


class TestRunningMeanStd:
    """Tests for the RunningMeanStd class."""

    def test_initialization(self):
        """Test that RunningMeanStd initializes correctly."""
        rms = RunningMeanStd()
        assert rms.n == 0
        assert rms.x == 0.0
        assert rms.x2 == 0.0

    def test_reset(self):
        """Test that reset() clears the estimator."""
        rms = RunningMeanStd()
        rms.append(1.0)
        rms.append(2.0)
        assert rms.n == 2
        rms.reset()
        assert rms.n == 0
        assert rms.x == 0.0
        assert rms.x2 == 0.0

    def test_single_value(self):
        """Test average with a single value."""
        rms = RunningMeanStd()
        rms.append(5.0)
        assert rms.average() == 5.0

    def test_average_multiple_values(self):
        """Test average calculation with multiple values."""
        rms = RunningMeanStd()
        values = [1.0, 2.0, 3.0, 4.0, 5.0]
        for v in values:
            rms.append(v)
        assert rms.average() == 3.0

    def test_average_empty_raises(self):
        """Test that average() raises assertion error when empty."""
        rms = RunningMeanStd()
        with pytest.raises(AssertionError):
            rms.average()

    def test_std_single_value_raises(self):
        """Test that std() raises assertion error with single value."""
        rms = RunningMeanStd()
        rms.append(5.0)
        with pytest.raises(AssertionError):
            rms.std()

    def test_std_calculation(self):
        """Test standard deviation calculation."""
        rms = RunningMeanStd()
        values = [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0]
        for v in values:
            rms.append(v)

        # Calculate expected std
        expected_mean = np.mean(values)
        expected_std = np.std(values, ddof=0)
        unbiased = np.sqrt(1.0 + 1.0 / (len(values) - 1))
        expected_std *= unbiased

        assert np.isclose(rms.average(), expected_mean)
        assert np.isclose(rms.std(), expected_std)

    def test_negative_values(self):
        """Test that the estimator works with negative values."""
        rms = RunningMeanStd()
        values = [-5.0, -3.0, -1.0, 1.0, 3.0, 5.0]
        for v in values:
            rms.append(v)
        assert np.isclose(rms.average(), 0.0)
        assert rms.std() > 0

    def test_identical_values(self):
        """Test with all identical values (zero std)."""
        rms = RunningMeanStd()
        values = [7.0, 7.0, 7.0, 7.0]
        for v in values:
            rms.append(v)
        assert rms.average() == 7.0
        # With identical values, std should be close to zero
        assert np.isclose(rms.std(), 0.0, atol=1e-10)
