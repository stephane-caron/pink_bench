# pink\_bench

[![PyPI version](https://img.shields.io/pypi/v/pink_bench)](https://pypi.org/project/pink_bench/)

A library of robot motions performed by inverse kinematics and model predictive control:

<p align="center">
    <img src="https://github.com/user-attachments/assets/5b169c2b-3c84-47b8-96d0-5b9095255f52" alt="Sample scenarios from the pink bench" />
</p>

## Installation

For best compatibility we recommended installing the bench from Conda:

```console
conda install -c conda-forge pink_bench
```

You can also install it from PyPI:

```console
pip install pink_bench
```

## Usage

To play a scenario from the library, simply call:

```py
pink_bench.play_scenario(
    name="jaxon",
    dt=0.005,  # seconds
    qpsolver="proxqp",
)
```

This will open a MeshCat tab in your web browser playing the scenario for its prescribed duration. You can also do the same from the command-line:

```console
$ ./examples/run_scenario.py jaxon --dt 0.005 --qpsolver proxqp
```

## See also

The pink bench is a combination of many open-source libraries:

- [robot\_descriptions.py](https://github.com/robot-descriptions/robot_descriptions.py) to load robot descriptions
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio/) for forward kinematics functions: Jacobians, Jlog6, ...
- [MeshCat](https://github.com/meshcat-dev/meshcat-python) for (optional) visualization
- [Pink](https://github.com/stephane-caron/pink) for differential inverse kinematics
- [qpmpc](https://github.com/stephane-caron/qpmpc) for linear model predictive control, used in biped and humanoid scenarios

It is also related to the following projects:

- [ik\_qpbenchmark](https://github.com/qpsolvers/ik_qpbenchmark): A test set for QP solvers with differential IK problems generated on the pink bench.
