function config = mevius2Config()
%MEVIUS2CONFIG Create editable defaults for the 2-DOF quadruped model.
%
% Units are SI: meters, kilograms, seconds, radians, Newton-meters, Amps.

config = struct();

config.robot.name = 'Mevius2';
config.robot.bodyLength = 0.73;
config.robot.bodyWidth = 0.38;
config.robot.bodyHeight = 0.30;
config.robot.bodyMassKg = 22.9;
config.robot.payloadMassKg = 0.00;

config.links.L1 = 0.3;
config.links.L2 = 0.3;
config.links.m1 = 1.152;
config.links.m2 = 0.154;
config.links.lc1 = config.links.L1 / 2;
config.links.lc2 = config.links.L2 / 2;
config.links.I1 = config.links.m1 * config.links.L1^2 / 12;
config.links.I2 = config.links.m2 * config.links.L2^2 / 12;

% q1 is hip pitch measured from forward-horizontal; q2 is knee pitch.
config.joints.hipLimitsDeg = [-125, 55];
config.joints.kneeLimitsDeg = [-170, -5];

config.motor.name = 'RobStride 03';
config.motor.Kt_Nm_per_A = 2.36;
config.motor.Kv_RPM_per_V = 2.08333333333;
config.motor.NoLoadCurrent_A = 0.60;
config.motor.MaxCurrent_A = 43;
config.motor.Voltage_V = 48;
config.motor.WindingResistance_Ohm = 0.39;
config.motor.GearRatio = 9;
config.motor.GearEfficiency = 0.85;

config.simulation.dt = 0.02;
config.simulation.cycles = 3;
config.simulation.gravity = 9.80665;
config.simulation.animateEveryNFrames = 2;
config.simulation.maxAnimationFps = 30;

config.battery.nominalVoltageV = 48;
config.battery.capacityAh = 3.3;
config.battery.usableCapacityPct = 85;
config.battery.auxiliaryDrawW = 10;
config.battery.auxiliaryDrawA = 0;
config.battery.currentSafetyFactor = 1.5;
config.battery.internalResistanceOhm = 0;
config.battery.cutoffVoltageV = 0;

config = refreshQuadrupedConfig(config);
config.gaits = defaultQuadrupedGaits(config);
end
