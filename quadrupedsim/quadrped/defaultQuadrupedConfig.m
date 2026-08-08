function config = defaultQuadrupedConfig()
%DEFAULTQUADRUPEDCONFIG Create editable defaults for the 2-DOF quadruped model.
%
% Units are SI: meters, kilograms, seconds, radians, Newton-meters, Amps.

config = struct();

config.robot.name = '2DOF Quadruped IK Simulator';
config.robot.bodyLength = 0.30;
config.robot.bodyWidth = 0.18;
config.robot.bodyHeight = 0.16;
config.robot.bodyMassKg = 1.20;
config.robot.payloadMassKg = 0.00;

config.links.L1 = 0.12;
config.links.L2 = 0.12;
config.links.m1 = 0.080;
config.links.m2 = 0.070;
config.links.lc1 = config.links.L1 / 2;
config.links.lc2 = config.links.L2 / 2;
config.links.I1 = config.links.m1 * config.links.L1^2 / 12;
config.links.I2 = config.links.m2 * config.links.L2^2 / 12;

% q1 is hip pitch measured from forward-horizontal; q2 is knee pitch.
config.joints.hipLimitsDeg = [-125, 55];
config.joints.kneeLimitsDeg = [-170, -5];

config.motor.name = 'Example geared servo';
config.motor.Kt_Nm_per_A = 0.080;
config.motor.Kv_RPM_per_V = 120;
config.motor.NoLoadCurrent_A = 0.20;
config.motor.MaxCurrent_A = 12;
config.motor.Voltage_V = 12;
config.motor.WindingResistance_Ohm = 0.25;
config.motor.GearRatio = 9;
config.motor.GearEfficiency = 0.85;

config.simulation.dt = 0.02;
config.simulation.cycles = 3;
config.simulation.gravity = 9.80665;
config.simulation.animateEveryNFrames = 2;
config.simulation.maxAnimationFps = 30;

config.battery.nominalVoltageV = 24;
config.battery.capacityAh = 5;
config.battery.usableCapacityPct = 85;
config.battery.auxiliaryDrawW = 10;
config.battery.auxiliaryDrawA = 0;
config.battery.currentSafetyFactor = 1.5;
config.battery.internalResistanceOhm = 0;
config.battery.cutoffVoltageV = 0;

config = refreshQuadrupedConfig(config);
config.gaits = defaultQuadrupedGaits(config);
end
