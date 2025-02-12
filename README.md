# Omnicopter Firmware Adaptation
## UA LAGER specifics
### Pre-requisite
Docker engine

### Setup
Clone this repository and submodules
```
git clone git@github.com:TL-4319/omnicopter.git --recursive
```

### Begin gazebo simulator
Spin up a docker
```
./Tools/px4_docker_setup.sh
```

Within the docker container, navigate to the omnicopter directory

```
cd workspaces/omnicopter
```

Begin gazebo sim with iris drone
```
make px4_sitl gazebo_iris
```

This starts a SITL simulation with a IRIS drone in Gazebo. The drone mavlink telemetry is exposed over \<IP-address>:18570 UDP port

## Virginia Tech addition
The omnicopter is a 6DOF controllable multirotor in the shape of a cube actuated by 8 motors. It was originally designed at ETH Zurich by Dario Brescianini and Rafael D'Andrea. The original paper can be found [here](https://doi.org/10.1016/j.mechatronics.2018.08.005).

This vehicle has been recreated in the [Spacedrones lab](https://spacedrones.aoe.vt.edu) at Virginia Tech using COTS hardware as well as 3d printed parts. Links to the CAD files and parts list can be found here. It is intended to emulate in-space free flyer dynamics.

Controller development has been performed for the [position](https://doi.org/10.2514/6.2021-4100) and [attitude](https://doi.org/10.2514/6.2022-0358) control of the omnicopter, particularly focusing on adaptive controllers.

Videos of the omnicopter and other Spacedrones projects can be found [here](https://www.youtube.com/channel/UCmOpHMDR7gplivKW3OsD2Xg).

Hardware and software development was performed by Patrick Thomas (thomaspj1017@vt.edu).
Controller development was peformed by Theresa Furgiuele (btheresa@vt.edu).

## Files Modified/Added Compared to the original Firmware

- Create Controller Code: [src/modules/](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/src/modules) (control allocation occurs within each new control module)
  - [omni_amrac_aback](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/src/modules/omni_amrac_aback): augmented MRAC position controller, adaptive backstepping attitude controller
  - [omni_amrac_amrac](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/src/modules/omni_amrac_amrac): augmented MRAC position controller, augmented MRAC attitude controller
  - [omni_amrac_pifl](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/src/modules/omni_amrac_pifl): augmented MRAC position controller, PI/feedback linearization attitude controller
  - [omni_pid_aback](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/src/modules/omni_pid_aback): PID position controller, adaptive backstepping attitude controller
  - [omnicopter_controller](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/src/modules/omnicopter_controller): PID position controller, PI/feedback linearization attitude controller
- Create Mixer and Vehicle Setup: [ROMFS/px4fmu_common](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/ROMFS/px4fmu_common)
  - [/mixers](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/ROMFS/px4fmu_common/mixers)
    - [omni.main.mix](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/ROMFS/px4fmu_common/mixers/omni.main.mix)
  - [/init.d](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/ROMFS/px4fmu_common/init.d)
    - [rc.omni_apps](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/ROMFS/px4fmu_common/init.d/rc.omni_apps): start the controllers
    - [rc.omni_defaults](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/ROMFS/px4fmu_common/init.d/rc.omni_defaults): set parameter defaults
    - [rc.vehicle_setup](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/ROMFS/px4fmu_common/init.d/rc.vehicle_setup): specify mixer and vehicle type
    - [CMakeLists.txt](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/ROMFS/px4fmu_common/init.d/CMakeLists.txt): add new files to compile
  - [/init.d/airframes](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/ROMFS/px4fmu_common/init.d/airframes)
    - [90002_omnicopter](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/ROMFS/px4fmu_common/init.d/airframes/90002_omnicopter): create the vehicle
    - [CMakeLists.txt](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt)
- Launch/Trajectory Publishing: [launch/quad_traj/scripts](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/launch/quad_traj/scripts)
  - [omni_offboard_control.py](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/launch/quad_traj/scripts/omni_offboard_control.py): Pulls in .csv file that is specified within the folder and launches/lands omnicopter with that trajectory
  - This folder also contains some matlab scripts for creating some trajectories. [create_csv_V2.m](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/launch/quad_traj/scripts/create_csv_V2.m) is currently the main script for creating trajectories.
- New uORB messages: [msg](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/msg)
  - [omni_debug.msg](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/msg/omni_debug.msg)
  - [omni_adap_debug.msg](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/msg/omni_adap_debug.msg)
  - [CMakeLists.txt](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/msg/CMakeLists.txt)
  - For debugging: [src/modules/logger/logged_topics.cpp](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/src/modules/logger/logged_topics.cpp)
    - Add name of new debugging message under **void LoggedTopics::add_debug_topics()**
    - Set **SDLOG\_PROFILE** in [rc.omni_defaults](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/ROMFS/px4fmu_common/init.d/rc.omni_defaults) to 33
  - For issues with MAVLINK, look in: [src/modules/mavlink/mavlink_receiver](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/src/modules/mavlink/mavlink_receiver.cpp)
    - e.g. **void MavlinkReceiver::handle\_message\_set\_attitude\_target(mavlink\_message\_t msg)**
- Compile Controller: [boards/px4/fmu-v5/default.px4board](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/tree/omnicopter/boards/px4/fmu-v5)
  - Option A: add **CONFIG\_MODULES\_\<moduleName\>=y** to the list
  - Option B: Create **boards/px4/fmu-v5/\<name\>.px4board**
    - Within new file, add **CONFIG_MODULES_\<modelueName\>=y**
    - To compile, type **make px4_fmu-v5_\<name\> upload**
    - Options:
      - [omni-adap.px4board](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/boards/px4/fmu-v5/omni-adap.px4board): aMRAC/aBack control
      - [omni-amrac.px4board](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/boards/px4/fmu-v5/omni-amrac.px4board): aMRAC/aMRAC control
      - [omni.px4board](https://code.vt.edu/space_at_vt/space_drones/04---rpo-drone-to-space-simulator/omnicopter/-/blob/omnicopter/boards/px4/fmu-v5/omni.px4board): PID/PIFL control


***********ALL INFO BELOW THIS POINT IS INCLUDED FROM THE ORIGINAL PX4 FIRMWARE REPOSITORY'S README***********


# PX4 Drone Autopilot

[![Releases](https://img.shields.io/github/release/PX4/PX4-Autopilot.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![DOI](https://zenodo.org/badge/22634/PX4/PX4-Autopilot.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/PX4-Autopilot)

[![Nuttx Targets](https://github.com/PX4/PX4-Autopilot/workflows/Nuttx%20Targets/badge.svg)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22Nuttx+Targets%22?branch=master) [![SITL Tests](https://github.com/PX4/PX4-Autopilot/workflows/SITL%20Tests/badge.svg?branch=master)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22SITL+Tests%22)

[![Slack](/.github/slack.svg)](https://join.slack.com/t/px4/shared_invite/zt-si4xo5qs-R4baYFmMjlrT4rQK5yUnaA)

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/master/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

PX4 is highly portable, OS-independent and supports Linux, NuttX and MacOS out of the box.

* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/PX4-Autopilot/blob/master/LICENSE))
* [Supported airframes](https://docs.px4.io/master/en/airframes/airframe_reference.html) ([portfolio](http://px4.io/#airframes)):
  * [Multicopters](https://docs.px4.io/master/en/frames_multicopter/)
  * [Fixed wing](https://docs.px4.io/master/en/frames_plane/)
  * [VTOL](https://docs.px4.io/master/en/frames_vtol/)
  * [Autogyro](https://docs.px4.io/master/en/frames_autogyro/)
  * [Rover](https://docs.px4.io/master/en/frames_rover/)
  * many more experimental types (Blimps, Boats, Submarines, High altitude balloons, etc)
* Releases: [Downloads](https://github.com/PX4/PX4-Autopilot/releases)


## Building a PX4 based drone, rover, boat or robot

The [PX4 User Guide](https://docs.px4.io/master/en/) explains how to assemble [supported vehicles](https://docs.px4.io/master/en/airframes/airframe_reference.html) and fly drones with PX4.
See the [forum and chat](https://docs.px4.io/master/en/#support) if you need help!


## Changing code and contributing

This [Developer Guide](https://docs.px4.io/master/en/development/development.html) is for software developers who want to modify the flight stack and middleware (e.g. to add new flight modes), hardware integrators who want to support new flight controller boards and peripherals, and anyone who wants to get PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the [Guide for Contributions](https://docs.px4.io/master/en/contribute/).
See the [forum and chat](https://dev.px4.io/master/en/#support) if you need help!


### Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://dev.px4.io/master/en/contribute/#dev_call).

> **Note** The dev call is open to all interested developers (not just the core dev team). This is a great opportunity to meet the team and contribute to the ongoing development of the platform. It includes a QA session for newcomers. All regular calls are listed in the [Dronecode calendar](https://www.dronecode.org/calendar/).


## Maintenance Team

  * Project: Founder
    * [Lorenz Meier](https://github.com/LorenzMeier)
  * Architecture
    * [Daniel Agar](https://github.com/dagar)
  * [Dev Call](https://github.com/PX4/PX4-Autopilot/labels/devcall)
    * [Ramon Roche](https://github.com/mrpollo)
  * Communication Architecture
    * [Beat Kueng](https://github.com/bkueng)
    * [Julian Oes](https://github.com/JulianOes)
  * UI in QGroundControl
    * [Gus Grubba](https://github.com/dogmaphobic)
  * [Multicopter Flight Control](https://github.com/PX4/PX4-Autopilot/labels/multicopter)
    * [Mathieu Bresciani](https://github.com/bresch)
  * [Multicopter Software Architecture](https://github.com/PX4/PX4-Autopilot/labels/multicopter)
    * [Matthias Grob](https://github.com/MaEtUgR)
  * [VTOL Flight Control](https://github.com/PX4/PX4-Autopilot/labels/vtol)
    * [Roman Bapst](https://github.com/RomanBapst)
  * [Fixed Wing Flight Control](https://github.com/PX4/PX4-Autopilot/labels/fixedwing)
    * [Roman Bapst](https://github.com/RomanBapst)
  * OS / NuttX
    * [David Sidrane](https://github.com/davids5)
  * Driver Architecture
    * [Daniel Agar](https://github.com/dagar)
  * Commander Architecture
    * [Julian Oes](https://github.com/julianoes)
  * [UAVCAN](https://github.com/PX4/PX4-Autopilot/labels/uavcan)
    * [Daniel Agar](https://github.com/dagar)
  * [State Estimation](https://github.com/PX4/PX4-Autopilot/issues?q=is%3Aopen+is%3Aissue+label%3A%22state+estimation%22)
    * [Paul Riseborough](https://github.com/priseborough)
  * Vision based navigation and Obstacle Avoidance
    * [Markus Achtelik](https://github.com/markusachtelik)
  * RTPS/ROS2 Interface
    * [Nuno Marques](https://github.com/TSC21)

See also [maintainers list](https://px4.io/community/maintainers/) (px4.io) and the [contributors list](https://github.com/PX4/PX4-Autopilot/graphs/contributors) (Github).

## Supported Hardware

This repository contains code supporting Pixhawk standard boards (best supported, best tested, recommended choice) and proprietary boards.

### Pixhawk Standard Boards
  * FMUv6X and FMUv6U (STM32H7, 2021)
    * Various vendors will provide FMUv6X and FMUv6U based designs Q3/2021
  * FMUv5 and FMUv5X (STM32F7, 2019/20)
    * [Pixhawk 4 (FMUv5)](https://docs.px4.io/master/en/flight_controller/pixhawk4.html)
    * [Pixhawk 4 mini (FMUv5)](https://docs.px4.io/master/en/flight_controller/pixhawk4_mini.html)
    * [CUAV V5+ (FMUv5)](https://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html)
    * [CUAV V5 nano (FMUv5)](https://docs.px4.io/master/en/flight_controller/cuav_v5_nano.html)
    * [Auterion Skynode (FMUv5X)](https://docs.px4.io/master/en/flight_controller/auterion_skynode.html)
  * FMUv4 (STM32F4, 2015)
    * [Pixracer](https://docs.px4.io/master/en/flight_controller/pixracer.html)
    * [Pixhawk 3 Pro](https://docs.px4.io/master/en/flight_controller/pixhawk3_pro.html)
  * FMUv3 (STM32F4, 2014)
    * [Pixhawk 2](https://docs.px4.io/master/en/flight_controller/pixhawk-2.html)
    * [Pixhawk Mini](https://docs.px4.io/master/en/flight_controller/pixhawk_mini.html)
    * [CUAV Pixhack v3](https://docs.px4.io/master/en/flight_controller/pixhack_v3.html)
  * FMUv2 (STM32F4, 2013)
    * [Pixhawk](https://docs.px4.io/master/en/flight_controller/pixhawk.html)
    * [Pixfalcon](https://docs.px4.io/master/en/flight_controller/pixfalcon.html)

### Manufacturer and Community supported
  * [Holybro Durandal](https://docs.px4.io/master/en/flight_controller/durandal.html)
  * [Hex Cube Orange](https://docs.px4.io/master/en/flight_controller/cubepilot_cube_orange.html)
  * [Hex Cube Yellow](https://docs.px4.io/master/en/flight_controller/cubepilot_cube_yellow.html)
  * [Airmind MindPX V2.8](http://www.mindpx.net/assets/accessories/UserGuide_MindPX.pdf)
  * [Airmind MindRacer V1.2](http://mindpx.net/assets/accessories/mindracer_user_guide_v1.2.pdf)
  * [Bitcraze Crazyflie 2.0](https://docs.px4.io/master/en/complete_vehicles/crazyflie2.html)
  * [Omnibus F4 SD](https://docs.px4.io/master/en/flight_controller/omnibus_f4_sd.html)
  * [Holybro Kakute F7](https://docs.px4.io/master/en/flight_controller/kakutef7.html)
  * [Raspberry PI with Navio 2](https://docs.px4.io/master/en/flight_controller/raspberry_pi_navio2.html)

Additional information about supported hardware can be found in [PX4 user Guide > Autopilot Hardware](https://docs.px4.io/master/en/flight_controller/).

## Project Roadmap

A high level project roadmap is available [here](https://github.com/orgs/PX4/projects/25).
