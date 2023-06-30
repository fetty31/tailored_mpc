<a name="readme-top"></a>
<br />
<div align="center">
<a> <img src="common/logo/cat15x_cupra.png" alt="Logo" width="355"> </a>
<h3 align="center">Tailored MPC</h3>
<p align="center">
        This pkg is thought to be the main AS controller for CAT15x. Within this repo you will find 3 different controllers (time-variant, spatial-variant and lateral (also time-variant)). The one used for the 2022-2023 season is the lateral approach, leaving the others for a future implementation.
    <br />
  </p>
</div>

<details>
    <summary>Table of Contents</summary>
    <ol>
        <li>
        <a href="#getting-started">Getting Started</a>
        <ul>
            <li><a href="#usage">Usage</a></li>
            <li><a href="#branches">Branches</a></li>
        </ul>
        </li>
        <li><a href="#prerequisites">Prerequisites</a>
        <ul>
            <li><a href="#msg-dependencies">Msg dependencies</a></li>
            <li><a href="#solver-dependencies">Solver dependencies</a></li>
        </ul>
        </li>
        <li>
        <a href="#approach">Approach</a>
        <ul>
            <li><a href="#lateral-mpc">Lateral MPC</a></li>
            <li><a href="#time-dependant-mpc">Curvature MPC</a></li>
            <li><a href="#space-dependant-mpc">Spatial MPC</a></li>
        </ul>
        </li>
    </ol>
</details>

<br />

# Getting Started

## Usage
In order to execute this pkg a [ForcesPro](https://forces.embotech.com/Documentation/index.html) software/hardware license is needed. If you don't have any license available read the __Solvers__ section in [AS README.md](https://bcnemotorsport.upc.edu:81/autonomous-systems-group/autonomous_systems).

This pkg has a launch file for each dynamic event (or should have) because of the specific parametres used depending on the event. 

An example for executing the autocross controller:

```sh
roslaunch tailored_mpc autoX.launch
```

## Branches
Here's a quick explanation of this project branches:

* `master` : time-dependant Non Linear MPC. Here a coupled (longitudinal+lateral dynamics) autonomous controller is specified. The main characteristics of this controller are:
    * High computational load (30ms)
    * Curvature based.
    * Handles throttle + steering commands.
    * Complex/Time consuming tuning.
    * High specialization (different parameters for each dynamic event).
    * No need for velocity profile input.
* `lateral` : time-dependant Non Linear MPC. Here a decoupled (lateral dynamics) autonomous controller is specified. The main characteristics of this controller are:
    * Lower computational load (10ms).
    * Curvature based.
    * Handles steering commands.
    * Lower tuning complexity.
    * Must have a velocity profile as input for the whole horizon length (prediction horizon).
    * A longitudinal controller must run in paralel, following as close as possible the given velocity profile.
* `spatial` : space-dependant Non Linear MPC. Here a coupled (lateral+longitudinal dynamics) autonomous controller is specified. The main characteristics of this controller are:
    * High computational load (30ms)
    * Curvature based.
    * Handles throttle + steering commands.
    * Complex/Time consuming tuning
    * No need for velocity profile input
    * High specialization (different parameters for each dynamic event).
    * Space dependant output, must be extrapolated into the time domain afterwards.
    * Adaptive integration step. Depending on the meters covered by the planner the spatial integration step can be bigger/lower.

__NOTE:__ The other branches are still in an early development process. They're not ready for deployment!

# Prerequisites

## Msg dependencies
This pkg depends on the self-defined [as_msgs](https://bcnemotorsport.upc.edu:81/autonomous-systems-group/autonomous_systems). Make sure to have this msgs on your current workspace.

## Solver dependecies
As said before, this pkg depends on [Embotech ForcesPro]((https://www.embotech.com/products/forcespro/overview/)) solver so a Hardware/Software license is needed. See the __Solvers__ section from [AS README.md](https://bcnemotorsport.upc.edu:81/autonomous-systems-group/autonomous_systems) for more information.

# Approach

Here the theoretical approach of each branch will be summarized. For more information read [TRO](docs/TRO.pdf), [Lateral MPC]() and [Curvature MPC](https://drive.google.com/file/d/1rntZJFIQ_4R1oglZBTvtA23OXXG3SWAU/view?usp=drive_link) papers.

For the sake of simplicity the different controllers are named after their more important characteristic. However, all the specified MPC controllers are curvature-based and follow a simplified non linear bicycle model.

## Lateral MPC

## Time dependant MPC

## Space dependant MPC
