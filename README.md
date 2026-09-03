# Adaptive_Loss_of_Effectiveness_Control
Numerical implementation and simulation of an adaptive control scheme for a two-degree-of-freedom robotic manipulator.

## Overview

This repository contains the MATLAB implementation developed during
a sabbatical research stay. The code implements and simulates an
adaptive control scheme for a 2-DOF robotic manipulator.

The simulation considers:

- Adaptive parameter estimation.
- Actuator input saturation.
- Saturation in the adaptive subsystem.
- Trajectory tracking.
- Parameter estimation errors.
- Comparison between the nominal and saturated control inputs.

## Requirements

The code is intended to be compatible with both MATLAB and GNU Octave.

### MATLAB

- MATLAB R2018 or later
- Standard MATLAB numerical and plotting functionality

### GNU Octave

- GNU Octave 8.x or later

## MATLAB / GNU Octave Compatibility

This repository is developed using MATLAB-compatible syntax and is
intended to run in both MATLAB and GNU Octave.

The numerical simulations use standard ODE solvers and basic
matrix and plotting operations.

## Acknowledgments

This work was developed during a sabbatical research stay at
Laboratoire des Signaux et Systemes, supported by SECIHTI - CentroGeo.

## Citation

If you use this code in academic work, please cite:

> López-Araujo DJ, "Adaptive control for systems under input failure: trajectory tracking and
output feedback approaches", MATLAB/OCTAVE implementation,
> GitHub repository, 2026. https://github.com/LoadING-718/Adaptive_Loss_of_Effectiveness_Control



Compatibility with GNU Octave should be verified for the complete
set of auxiliary functions included in the repository.
