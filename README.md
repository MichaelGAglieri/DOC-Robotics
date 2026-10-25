# D.O.C. — Desktop Operations Companion

> **Work in progress.** This project is under active development. Hardware, software, interfaces and configuration can change. See the [roadmap](ROADMAP.md).

D.O.C. is my five-axis desktop robotics project, built to connect mechanical assembly, simulation and software control. This repository contains the current ROS 2 laboratory scenario, its Webots model, reproducible tests and the reference hardware documentation for macOS Apple Silicon.

The physical arm uses the **EB-15 mechanical design by ToolBox Robotics**. My work covers the five-axis integration, PETG printing and assembly, wiring, firmware/app iteration, simulation setup, ROS 2 coordination and bench tests. Development used AI assistance; the external designs and dependencies are credited separately in [AUTHORS.md](AUTHORS.md) and [THIRD_PARTY.md](THIRD_PARTY.md).

## What is included

- ROS 2 Jazzy through RoboStack/Pixi: local knowledge, explicit commands, scene validation, mission planning and a local execution journal.
- A Webots R2025a laboratory: two demonstration containers, four supports, five-axis motion and a kinematic gripper.
- Geometry, protocol and mission tests.
- Reference wiring and configuration for Mega 2560, RAMPS 1.4, A4988 and an external DM542.

The companion macOS control application and its stable firmware are in [Robot-Console](https://github.com/MichaelGAglieri/Robot-Console). It records, edits, enables/disables and replays poses; it also offers an optional local Ollama assistant and a ROS 2 observer.

## Start the simulation

Requirements: macOS Apple Silicon, [Pixi](https://pixi.sh), and [Webots R2025a](https://github.com/cyberbotics/webots/releases/tag/R2025a). Install the environment and fetch the pinned upstream assets once:

```sh
pixi install
pixi run assets
pixi run test
pixi run webots
```

Press Play in Webots. In another terminal, from this directory:

```sh
pixi run doc
```

Use [QUICKSTART.md](QUICKSTART.md) for commands, mission planning and shutdown. `pixi run finestre` opens the macOS working windows; it does not start physical robot control. Voice and a locally installed Ollama model are optional.

## Scope of the evidence

DocArm5 is derived from **Cyberbotics’ UR3e model**, with the sixth joint removed. It is a demonstration model, not a calibrated digital twin of the physical EB-15-based arm. The gripper uses a kinematic attachment in simulation; simulation outcomes are not measurements of real grip force, payload capacity or collision safety.

The physical configuration has no automatic homing, encoders or force sensing in this release. Never send the simulated UR3e-derived waypoints to the physical arm. The firmware/app count commanded pulses; the operator establishes HOME.

## Project layout

| Path | Purpose |
| --- | --- |
| `architetture/laboratorio/ros2_ws/src/doc_lab` | Current ROS 2 package and its tests |
| `webots_sim` | Current world, controller, parameters and model |
| `scripts` | Generators, upstream asset setup and simulation client |
| `test` | Scene/geometry tests |
| `hardware` | Reference configuration and original design links |
| `docs` | Architecture and validation notes |

The earlier drawer scenario, old firmware experiments, installed environments, private runtime data and development archives are intentionally outside this release. Windows support is not provided in this version.

## License

Original code in this release is GPL-3.0-only. Cyberbotics-derived material retains Apache-2.0 and its notices. The original ToolBox Robotics CAD is linked rather than redistributed. See [LICENSE](LICENSE), [THIRD_PARTY.md](THIRD_PARTY.md) and [AUTHORS.md](AUTHORS.md).
