function config = openDynamicRobotInitConfig()
%OPENDYNAMICROBOTINITCONFIG Create editable defaults for the 2-DOF quadruped model.
%
% Units are SI: meters, kilograms, seconds, radians, Newton-meters, Amps.

config = struct();

config.robot.name = 'ODRI Solo8 V2';
config.robot.bodyLength = 0.38;
config.robot.bodyWidth = 0.209;
config.robot.bodyHeight = 0.21586;
config.robot.bodyMassKg = 1.22195485;
config.robot.payloadMassKg = 0.00;

config.links.L1 = 0.16;
config.links.L2 = 0.16;
config.links.m1 = 0.339022575;
config.links.m2 = 0.339022575;
config.links.lc1 = config.links.L1 / 2;
config.links.lc2 = config.links.L2 / 2;
config.links.I1 = config.links.m1 * config.links.L1^2 / 12;
config.links.I2 = config.links.m2 * config.links.L2^2 / 12;

% q1 is hip pitch measured from forward-horizontal; q2 is knee pitch.
config.joints.hipLimitsDeg = [-125, 55];
config.joints.kneeLimitsDeg = [-170, -5];

config.hipMountRoll= [0,0,0,0];
config.hipMountYaw = [0,0,-pi,pi];

config.motor.name = 'MN4004 Antigravity Type 4-6S UAV Motor KV300';
config.motor.Kt_Nm_per_A = 0.0318;
config.motor.Kv_RPM_per_V = 300;
config.motor.NoLoadCurrent_A = 0.20;
config.motor.MaxCurrent_A = 9;
config.motor.Voltage_V = 22.2;
config.motor.WindingResistance_Ohm = 0.452;
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
