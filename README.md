# frc-2022
Team 2485 in-development code for the 2022 FRC game, Rapid React. 
Includes code for:
- Drive: SDS MK3/MK4 swerve modules running 2 Falcon 500s. 
- Intake: geared arm with lower wheels and centering rollers for over-bumper intake (NEO)
- Indexer: star wheel with passive spring (NEO 550)
- Feeder: belts actuated by servos (NEO 550, Rev Smart Servo)
- Shooter: Lower 6 in AndyMark shooter wheel, upper 2 in sushi roller wheels (2 Falcon 500s)
- Climber:
  - Elevator: Sliding hook (Falcon 500, Rev Smart Servo on ratchet)
  - Arm: Rotating/translating rack and pinion mechanism (Falcon 500)
- Turret (not used after SDR): Armabot 240 turret, analog potentiometer (775 Pro)
- Hood (not used after SDR): Neo 550 chain drive

Controls approach:
- Model based, robust control 
- Feedforward + feedback (P or PD) control, both model-deterimined directly or with LQR
- Trapezoidal motion profiling on position subsystems

Other cool stuff:
- Odometry and vision (LL 2+ running PhotonVision) fused pose estimation
- Trajectory folowing via PathPlanner
- Heavy use of trigger binding, state machines, and command factories
