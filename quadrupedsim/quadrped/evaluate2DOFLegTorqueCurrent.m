function out = evaluate2DOFLegTorqueCurrent(qRad, qdRad, qddRad, links, motor, gravity, footForceXZ)
%EVALUATE2DOFLEGTORQUECURRENT Torque/current for one 2R leg pose.
%
% qRad  = [hipPitch, kneePitch] in radians. q1=0 is forward-horizontal;
%         negative q angles place the foot below the hip in this convention.
% qdRad = joint velocity [rad/s], optional, defaults to zero.
% qddRad = joint acceleration [rad/s^2], optional, defaults to zero.
% footForceXZ = [Fx, Fz] force that the foot must apply to the ground/load
%               in the leg plane. For body support, Fz is usually negative
%               because the foot pushes down on the ground.
%
% The dynamics come from the planar two-link Lagrangian:
% tau = d/dt(dL/dqd) - dL/dq + J(q)'*F
%     = M(q)qdd + C(q,qd) + G(q) + J(q)'*F.
% Current is estimated at the motor side using Kt, gear ratio, and
% efficiency.

if nargin < 2 || isempty(qdRad)
    qdRad = [0, 0];
end
if nargin < 3 || isempty(qddRad)
    qddRad = [0, 0];
end
if nargin < 4 || isempty(links)
    links = defaultQuadrupedConfig().links;
end
if nargin < 5 || isempty(motor)
    motor = defaultQuadrupedConfig().motor;
end
if nargin < 6 || isempty(gravity)
    gravity = 9.80665;
end
if nargin < 7 || isempty(footForceXZ)
    footForceXZ = [0, 0];
end

links = normalizeLinks(links);
motor = normalizeMotor(motor);

q = qRad(:);
qd = qdRad(:);
qdd = qddRad(:);
footForceXZ = footForceXZ(:);
if numel(q) ~= 2 || numel(qd) ~= 2 || numel(qdd) ~= 2
    error('qRad, qdRad, and qddRad must each contain exactly two values.');
end
if numel(footForceXZ) ~= 2
    error('footForceXZ must contain exactly two values: [Fx, Fz].');
end

q1 = q(1);
q2 = q(2);
dq1 = qd(1);
dq2 = qd(2);

L1 = links.L1;
lc1 = links.lc1;
lc2 = links.lc2;
m1 = links.m1;
m2 = links.m2;
I1 = links.I1;
I2 = links.I2;
g = gravity;

M11 = I1 + I2 + m1 * lc1^2 + m2 * (L1^2 + lc2^2 + 2 * L1 * lc2 * cos(q2));
M12 = I2 + m2 * (lc2^2 + L1 * lc2 * cos(q2));
M22 = I2 + m2 * lc2^2;
M = [M11, M12; M12, M22];

C = [
    -m2 * L1 * lc2 * sin(q2) * (2 * dq1 * dq2 + dq2^2);
     m2 * L1 * lc2 * sin(q2) * dq1^2];

G = [
    (m1 * lc1 + m2 * L1) * g * cos(q1) + m2 * lc2 * g * cos(q1 + q2);
     m2 * lc2 * g * cos(q1 + q2)];

J = [
    -L1 * sin(q1) - links.L2 * sin(q1 + q2), -links.L2 * sin(q1 + q2);
     L1 * cos(q1) + links.L2 * cos(q1 + q2),  links.L2 * cos(q1 + q2)];
loadTorque = J.' * footForceXZ;
lagrangianTorque = M * qdd + C + G;
tau = lagrangianTorque + loadTorque;

gearRatio = motor.GearRatio;
gearEfficiency = motor.GearEfficiency;
Kt = motor.Kt_Nm_per_A;
noLoadCurrent = motor.NoLoadCurrent_A;

motorTorqueNm = abs(tau) ./ (gearRatio * gearEfficiency);
torqueCurrentA = motorTorqueNm ./ Kt;
currentA = torqueCurrentA + noLoadCurrent;
signedCurrentA = tau ./ (gearRatio * gearEfficiency * Kt) + sign(tau) .* noLoadCurrent;

kvRadPerSecPerVolt = motor.Kv_RPM_per_V * 2 * pi / 60;
motorSpeedRadSec = abs(qd) .* gearRatio;
backEmfV = motorSpeedRadSec ./ kvRadPerSecPerVolt;
if isfinite(motor.WindingResistance_Ohm)
    copperVoltageV = torqueCurrentA .* motor.WindingResistance_Ohm;
    voltageEstimateV = backEmfV + copperVoltageV;
else
    copperVoltageV = nan(2, 1);
    voltageEstimateV = backEmfV;
end

if isfinite(motor.MaxCurrent_A)
    currentLimitExceeded = currentA > motor.MaxCurrent_A;
else
    currentLimitExceeded = false(2, 1);
end
if isfinite(motor.Voltage_V)
    voltageLimitExceeded = voltageEstimateV > motor.Voltage_V;
else
    voltageLimitExceeded = false(2, 1);
end

out = struct();
out.qRad = q.';
out.qDeg = q.' * 180 / pi;
out.qdRadSec = qd.';
out.qddRadSec2 = qdd.';
out.massMatrix = M;
out.coriolisNm = C.';
out.gravityNm = G.';
out.jacobianXZ = J;
out.footForceXZ_N = footForceXZ.';
out.loadTorqueNm = loadTorque.';
out.lagrangianTorqueNm = lagrangianTorque.';
out.torqueNm = tau.';
out.motorTorqueNm = motorTorqueNm.';
out.currentA = currentA.';
out.signedCurrentA = signedCurrentA.';
out.torqueCurrentA = torqueCurrentA.';
out.backEmfV = backEmfV.';
out.copperVoltageV = copperVoltageV.';
out.voltageEstimateV = voltageEstimateV.';
out.currentLimitExceeded = currentLimitExceeded.';
out.voltageLimitExceeded = voltageLimitExceeded.';
out.motor = motor;
out.links = links;
end

function links = normalizeLinks(links)
links.L1 = positiveField(links, 'L1', 0.12);
links.L2 = positiveField(links, 'L2', 0.12);
links.m1 = positiveField(links, 'm1', 0.080);
links.m2 = positiveField(links, 'm2', 0.070);
links.lc1 = positiveField(links, 'lc1', links.L1 / 2);
links.lc2 = positiveField(links, 'lc2', links.L2 / 2);
links.lc1 = min(links.lc1, links.L1);
links.lc2 = min(links.lc2, links.L2);
links.I1 = positiveField(links, 'I1', links.m1 * links.L1^2 / 12);
links.I2 = positiveField(links, 'I2', links.m2 * links.L2^2 / 12);
end

function motor = normalizeMotor(motor)
motor.name = textField(motor, 'name', 'Motor');
motor.Kv_RPM_per_V = positiveField(motor, 'Kv_RPM_per_V', nan);
motor.Kt_Nm_per_A = positiveField(motor, 'Kt_Nm_per_A', nan);
if ~isfinite(motor.Kt_Nm_per_A) && isfinite(motor.Kv_RPM_per_V)
    motor.Kt_Nm_per_A = 60 / (2 * pi * motor.Kv_RPM_per_V);
end
if ~isfinite(motor.Kv_RPM_per_V) && isfinite(motor.Kt_Nm_per_A)
    motor.Kv_RPM_per_V = 60 / (2 * pi * motor.Kt_Nm_per_A);
end
motor.Kt_Nm_per_A = positiveScalarOrDefault(motor.Kt_Nm_per_A, 0.080);
motor.Kv_RPM_per_V = positiveScalarOrDefault(motor.Kv_RPM_per_V, 120);
motor.NoLoadCurrent_A = nonnegativeField(motor, 'NoLoadCurrent_A', 0);
motor.MaxCurrent_A = positiveField(motor, 'MaxCurrent_A', inf);
motor.Voltage_V = positiveField(motor, 'Voltage_V', inf);
motor.WindingResistance_Ohm = positiveField(motor, 'WindingResistance_Ohm', nan);
motor.GearRatio = positiveField(motor, 'GearRatio', 1);
motor.GearEfficiency = positiveField(motor, 'GearEfficiency', 1);
motor.GearEfficiency = min(max(motor.GearEfficiency, 0.01), 1);
end

function value = positiveField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = positiveScalarOrDefault(s.(fieldName), defaultValue);
else
    value = defaultValue;
end
end

function value = nonnegativeField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
if ~isnumeric(value) || isempty(value) || ~isfinite(value(1)) || value(1) < 0
    value = defaultValue;
else
    value = value(1);
end
end

function value = positiveScalarOrDefault(value, defaultValue)
if ~isnumeric(value) || isempty(value) || ~isfinite(value(1)) || value(1) <= 0
    value = defaultValue;
else
    value = value(1);
end
end

function value = textField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = char(s.(fieldName));
else
    value = defaultValue;
end
end
