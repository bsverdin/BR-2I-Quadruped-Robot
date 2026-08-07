# 2DOF Quadruped MATLAB IK Simulator

This project contains a MATLAB simulation for a four-leg quadruped with two pitch joints per leg: hip and knee. It animates the robot in 3D, plots joint movement, estimates torque and motor current during different gaits, and lets you import motor KT/KV constants.

## Run

Open MATLAB in this folder and run:

```matlab
quadrupedIKSimulator
```

## Motor Controller Bench Testing

Bench-test materials for the moteus-c1 motor controllers live in
`moteus_testing/`. Start with the lab runbook at
`docs/moteus_lab_bldc_setup_calibration.md`, then use
`docs/moteus_lab_troubleshooting.md` for common failures and fixes. The toolkit
also includes a measurement checklist, Python CSV telemetry logger, plot
generator, and an Excel test log/dashboard in
`outputs/moteus_motor_testing/moteus_motor_test_log.xlsx`.

The simulator includes:

- 3D animation of all four legs and each hip/knee joint.
- Built-in Walk, Trot, Pace, Bound, and Crawl gaits.
- Editable gait period, duty factor, stride length, clearance, body height, and leg phase offsets.
- Save/load gait files as `.mat` or `.json`.
- Editable link lengths, body dimensions, robot/payload mass, link masses, KT/KV, gear ratio, current limit, and voltage.
- Torque and current graphs with time on the x-axis.
- Live graph traces that fill during gait playback, with a moving time cursor that matches the robot pose.
- A corner metrics panel showing current and peak torque/current, joint speed, foot speed, and warning counts.
- Single-pose torque/current analysis for a specific hip/knee orientation.
- Performance controls for simulation timestep, displayed frame stride, and max animation FPS.

## Motor Import Format

Use `motor_presets.csv` as a template. The importer accepts CSV, XLSX, XLS, or TXT tables with flexible column names, but this exact header is recommended:

```csv
MotorName,Kt_Nm_per_A,Kv_RPM_per_V,NoLoadCurrent_A,MaxCurrent_A,Voltage_V,WindingResistance_Ohm,GearRatio,GearEfficiency,RobotMass_kg,PayloadMass_kg
```

`RobotMass_kg` is the body mass carried by the stance legs. The modeled leg-link masses are still handled separately by the Lagrangian link dynamics, so do not double-count those link masses here unless that is intentional for your sizing margin.

If only `Kv_RPM_per_V` is provided, the code estimates ideal `Kt_Nm_per_A` as:

```matlab
Kt = 60 / (2*pi*Kv_RPM_per_V)
```

## Command-Line Examples

Run a gait without the GUI:

```matlab
config = defaultQuadrupedConfig();
motor = importMotorConstants('motor_presets.csv');
results = simulateQuadrupedGait(config, config.gaits(2), motor(1));
```

Evaluate torque/current for one specific pose:

```matlab
config = defaultQuadrupedConfig();
qDeg = [-70, -75];
out = evaluate2DOFLegTorqueCurrent(qDeg*pi/180, [0, 0], [0, 0], ...
    config.links, config.motor, config.simulation.gravity);
disp(out.torqueNm)
disp(out.currentA)
```

Load example gaits:

```matlab
quadrupedIKSimulator
```

Then click `Load Gait` and select `gaits/example_gaits.json`.

## Model Notes

The inverse kinematics model treats each leg as a planar two-link mechanism in the sagittal plane. The dynamic torque estimate uses the standard two-link Lagrangian formulation plus a stance foot-load term from robot/payload weight:

```matlab
tau = M(q)*qdd + C(q,qd) + G(q) + J(q)'*Ffoot
```

During stance, `Ffoot = [0; -((RobotMass_kg + PayloadMass_kg)*g / stanceLegCount)]`. Swing legs use zero body-support force, but still include their link inertia and link gravity terms.

Current is estimated from required output torque, gear ratio, gear efficiency, and motor torque constant:

```matlab
motorTorque = abs(jointTorque)/(gearRatio*gearEfficiency);
current = motorTorque/Kt + noLoadCurrent;
```

The estimate is only as accurate as the supplied link mass, center-of-mass, inertia, motor, and gearbox values. Add real measured parameters for final actuator sizing.

## Performance Tips

Open `Edit Robot/Motor` and adjust:

- `Simulation timestep dt (s)`: larger values produce fewer simulation samples.
- `Display every N simulation frames`: larger values skip more visual frames while keeping computed data.
- `Max animation FPS`: lower values reduce graphics load during playback.
