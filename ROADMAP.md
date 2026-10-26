# Work in progress and roadmap

D.O.C. and Robot Console are active development projects. The current release is a foundation for further hardware, sensing and software work. This list describes planned directions and options to investigate; it does not mean these features are installed or validated, and it does not promise delivery dates.

## Robot sensing

- Position references, home switches and travel-limit sensing across the robot.
- Joint/motor encoders to compare commanded motion with measured position and investigate missed steps and closed-loop control.
- Cameras for observation, scene understanding and future vision-assisted interaction.
- Proximity and distance sensors around the arm and work area.
- LiDAR or other ranging systems to explore workspace mapping and obstacle detection where appropriate.
- Additional sensors distributed through the robot as the mechanical/electrical design evolves; evaluate gripper state, contact and force sensing.

The sensor selection, number, placement, electrical interfaces and integration architecture remain to be designed and tested. Adding sensors alone does not create a safety-rated collision avoidance or emergency-stop system.

## Software evolution

- Expose measured feedback and sensor health in the ROS 2 architecture and control app.
- Revisit calibration, state estimation, limits and playback when measured feedback becomes available.
- Improve hardware configuration, reproducibility and validation evidence as the robot changes.
- Evaluate a Windows & Linux version after the current macOS release is consolidated. No Windows or Linux build is shipped in this release.

The present configuration still relies on an operator-established HOME and commanded pulse counts. See the current hardware and validation documentation for implemented capabilities.
