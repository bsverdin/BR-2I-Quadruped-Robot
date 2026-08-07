function results = simulateQuadrupedGait(config, gait, motor)
%SIMULATEQUADRUPEDGAIT Run IK, dynamics, and current estimates for one gait.
%
% results = simulateQuadrupedGait(config, gait, motor)
%
% Leg order is FL, FR, RL, RR. Each leg is a planar 2R mechanism in the
% robot sagittal plane, attached to a fixed body corner for visualization.

if nargin < 1 || isempty(config)
    config = defaultQuadrupedConfig();
end
config = refreshQuadrupedConfig(config);

if nargin < 2 || isempty(gait)
    if isfield(config, 'gaits') && ~isempty(config.gaits)
        gait = config.gaits(1);
    else
        gait = defaultQuadrupedGaits(config);
        gait = gait(1);
    end
end

if nargin < 3 || isempty(motor)
    motor = config.motor;
end

config = applyPresetOverrides(config, motor);
gait = normalizeGait(gait, config);
dt = config.simulation.dt;
duration = max(config.simulation.cycles, 0) * gait.period;
time = (0:dt:duration).';
if isempty(time)
    time = 0;
end

nFrames = numel(time);
nLegs = 4;
nJoints = 2;

jointAnglesRad = zeros(nFrames, nLegs, nJoints);
jointVelocityRadSec = zeros(nFrames, nLegs, nJoints);
jointAccelerationRadSec2 = zeros(nFrames, nLegs, nJoints);
torqueNm = zeros(nFrames, nLegs, nJoints);
currentA = zeros(nFrames, nLegs, nJoints);
signedCurrentA = zeros(nFrames, nLegs, nJoints);
voltageEstimateV = zeros(nFrames, nLegs, nJoints);
motorTorqueNm = zeros(nFrames, nLegs, nJoints);
loadTorqueNm = zeros(nFrames, nLegs, nJoints);
lagrangianTorqueNm = zeros(nFrames, nLegs, nJoints);
supportForceN = zeros(nFrames, nLegs);

hipPosition = zeros(nFrames, nLegs, 3);
kneePosition = zeros(nFrames, nLegs, 3);
footPosition = zeros(nFrames, nLegs, 3);
targetFootPosition = zeros(nFrames, nLegs, 3);
footSpeedMps = zeros(nFrames, nLegs);

reachable = true(nFrames, nLegs);
swingMask = false(nFrames, nLegs);
phase = zeros(nFrames, nLegs);
currentLimitExceeded = false(nFrames, nLegs, nJoints);
voltageLimitExceeded = false(nFrames, nLegs, nJoints);
jointLimitExceeded = false(nFrames, nLegs, nJoints);

for k = 1:nFrames
    for leg = 1:nLegs
        hip = config.hipPositions(leg, :);
        [targetLocal, phase(k, leg), swingMask(k, leg)] = footTarget(time(k), gait, leg);
        [q, kneeWorld, footWorld, isReachable] = planarLegIK(targetLocal, hip, config.links);

        hipPosition(k, leg, :) = hip;
        kneePosition(k, leg, :) = kneeWorld;
        footPosition(k, leg, :) = footWorld;
        targetFootPosition(k, leg, :) = hip + targetLocal;
        jointAnglesRad(k, leg, :) = q;
        reachable(k, leg) = isReachable;
    end
end

for leg = 1:nLegs
    for joint = 1:nJoints
        qSeries = unwrap(jointAnglesRad(:, leg, joint));
        jointAnglesRad(:, leg, joint) = qSeries;
        if nFrames > 1
            jointVelocityRadSec(:, leg, joint) = gradient(qSeries, dt);
            jointAccelerationRadSec2(:, leg, joint) = gradient(jointVelocityRadSec(:, leg, joint), dt);
        end
    end
    if nFrames > 1
        xVel = gradient(squeeze(footPosition(:, leg, 1)), dt);
        yVel = gradient(squeeze(footPosition(:, leg, 2)), dt);
        zVel = gradient(squeeze(footPosition(:, leg, 3)), dt);
        footSpeedMps(:, leg) = sqrt(xVel.^2 + yVel.^2 + zVel.^2);
    end
end

for k = 1:nFrames
    stanceLegs = ~swingMask(k, :);
    stanceCount = sum(stanceLegs);
    if stanceCount == 0
        stanceLegs(:) = true;
        stanceCount = nLegs;
    end
    bodyLoadPerLegN = (config.robot.bodyMassKg + config.robot.payloadMassKg) * ...
        config.simulation.gravity / stanceCount;
    for leg = 1:nLegs
        q = squeeze(jointAnglesRad(k, leg, :)).';
        qd = squeeze(jointVelocityRadSec(k, leg, :)).';
        qdd = squeeze(jointAccelerationRadSec2(k, leg, :)).';
        if stanceLegs(leg)
            supportForceN(k, leg) = bodyLoadPerLegN;
        end
        footForceXZ = [0, -supportForceN(k, leg)];
        evalOut = evaluate2DOFLegTorqueCurrent(q, qd, qdd, config.links, motor, ...
            config.simulation.gravity, footForceXZ);

        torqueNm(k, leg, :) = evalOut.torqueNm;
        loadTorqueNm(k, leg, :) = evalOut.loadTorqueNm;
        lagrangianTorqueNm(k, leg, :) = evalOut.lagrangianTorqueNm;
        motorTorqueNm(k, leg, :) = evalOut.motorTorqueNm;
        currentA(k, leg, :) = evalOut.currentA;
        signedCurrentA(k, leg, :) = evalOut.signedCurrentA;
        voltageEstimateV(k, leg, :) = evalOut.voltageEstimateV;
        currentLimitExceeded(k, leg, :) = evalOut.currentLimitExceeded;
        voltageLimitExceeded(k, leg, :) = evalOut.voltageLimitExceeded;

        qDeg = q * 180 / pi;
        jointLimitExceeded(k, leg, 1) = qDeg(1) < config.joints.hipLimitsDeg(1) || ...
            qDeg(1) > config.joints.hipLimitsDeg(2);
        jointLimitExceeded(k, leg, 2) = qDeg(2) < config.joints.kneeLimitsDeg(1) || ...
            qDeg(2) > config.joints.kneeLimitsDeg(2);
    end
end

results = struct();
results.time = time;
results.gait = gait;
results.configSnapshot = config;
results.motorSnapshot = motor;
results.legNames = config.legNames;
results.jointNames = {'Hip', 'Knee'};
results.jointAnglesRad = jointAnglesRad;
results.jointAnglesDeg = jointAnglesRad * 180 / pi;
results.jointVelocityRadSec = jointVelocityRadSec;
results.jointAccelerationRadSec2 = jointAccelerationRadSec2;
results.torqueNm = torqueNm;
results.loadTorqueNm = loadTorqueNm;
results.lagrangianTorqueNm = lagrangianTorqueNm;
results.motorTorqueNm = motorTorqueNm;
results.currentA = currentA;
results.signedCurrentA = signedCurrentA;
results.voltageEstimateV = voltageEstimateV;
results.hipPosition = hipPosition;
results.kneePosition = kneePosition;
results.footPosition = footPosition;
results.footSpeedMps = footSpeedMps;
results.targetFootPosition = targetFootPosition;
results.reachable = reachable;
results.swingMask = swingMask;
results.phase = phase;
results.supportForceN = supportForceN;
results.currentLimitExceeded = currentLimitExceeded;
results.voltageLimitExceeded = voltageLimitExceeded;
results.jointLimitExceeded = jointLimitExceeded;
results.frameMaxTorqueNm = max(reshape(abs(torqueNm), nFrames, []), [], 2);
results.frameMaxLoadTorqueNm = max(reshape(abs(loadTorqueNm), nFrames, []), [], 2);
results.frameMaxCurrentA = max(reshape(currentA, nFrames, []), [], 2);
results.frameMaxJointSpeedRadSec = max(reshape(abs(jointVelocityRadSec), nFrames, []), [], 2);
results.frameMaxFootSpeedMps = max(footSpeedMps, [], 2);
results.peakTorqueNm = cummax(results.frameMaxTorqueNm);
results.peakCurrentA = cummax(results.frameMaxCurrentA);
results.peakJointSpeedRadSec = cummax(results.frameMaxJointSpeedRadSec);
results.peakFootSpeedMps = cummax(results.frameMaxFootSpeedMps);
results.summary = summarizeResults(results);
end

function gait = normalizeGait(gait, config)
if ~isfield(gait, 'name') || isempty(gait.name)
    gait.name = 'Custom gait';
end
gait.period = positiveField(gait, 'period', 1.0);
gait.dutyFactor = positiveField(gait, 'dutyFactor', 0.65);
gait.dutyFactor = min(max(gait.dutyFactor, 0.05), 0.95);
gait.stepLength = positiveField(gait, 'stepLength', 0.075);
gait.clearance = positiveField(gait, 'clearance', 0.035);
gait.bodyHeight = positiveField(gait, 'bodyHeight', config.robot.bodyHeight);
if ~isfield(gait, 'phaseOffsets') || numel(gait.phaseOffsets) ~= 4
    gait.phaseOffsets = [0, 0.5, 0.75, 0.25];
else
    gait.phaseOffsets = reshape(gait.phaseOffsets, 1, 4);
end
if ~isfield(gait, 'description')
    gait.description = '';
end
end

function config = applyPresetOverrides(config, motor)
if isstruct(motor) && isfield(motor, 'RobotMass_kg') && ...
        isnumeric(motor.RobotMass_kg) && isfinite(motor.RobotMass_kg) && motor.RobotMass_kg >= 0
    config.robot.bodyMassKg = motor.RobotMass_kg;
end
if isstruct(motor) && isfield(motor, 'PayloadMass_kg') && ...
        isnumeric(motor.PayloadMass_kg) && isfinite(motor.PayloadMass_kg) && motor.PayloadMass_kg >= 0
    config.robot.payloadMassKg = motor.PayloadMass_kg;
end
config = refreshQuadrupedConfig(config);
end

function value = positiveField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName)) && ...
        isnumeric(s.(fieldName)) && isfinite(s.(fieldName)(1)) && s.(fieldName)(1) > 0
    value = s.(fieldName)(1);
else
    value = defaultValue;
end
end

function [targetLocal, phase, inSwing] = footTarget(t, gait, legIndex)
phase = mod(t / gait.period + gait.phaseOffsets(legIndex), 1);
stepLength = gait.stepLength;
height = gait.bodyHeight;
clearance = gait.clearance;
duty = gait.dutyFactor;

if phase < duty
    s = phase / duty;
    x = stepLength / 2 - stepLength * s;
    z = -height;
    inSwing = false;
else
    s = (phase - duty) / (1 - duty);
    smoothS = s * s * (3 - 2 * s);
    x = -stepLength / 2 + stepLength * smoothS;
    z = -height + clearance * sin(pi * s);
    inSwing = true;
end

targetLocal = [x, 0, z];
end

function [q, kneeWorld, footWorld, reachable] = planarLegIK(targetLocal, hipWorld, links)
L1 = links.L1;
L2 = links.L2;
x = targetLocal(1);
z = targetLocal(3);
r = max(hypot(x, z), eps);

c2Raw = (r^2 - L1^2 - L2^2) / (2 * L1 * L2);
c2 = min(1, max(-1, c2Raw));
q2 = -acos(c2);
q1 = atan2(z, x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));

kneeLocal = [L1 * cos(q1), 0, L1 * sin(q1)];
footLocal = kneeLocal + [L2 * cos(q1 + q2), 0, L2 * sin(q1 + q2)];
kneeWorld = hipWorld + kneeLocal;
footWorld = hipWorld + footLocal;

reachable = abs(c2Raw) <= 1 && r <= (L1 + L2) && r >= abs(L1 - L2);
q = [q1, q2];
end

function summary = summarizeResults(results)
summary = struct();
summary.maxAbsTorqueNm = squeeze(max(max(abs(results.torqueNm), [], 1), [], 2)).';
summary.maxAbsLoadTorqueNm = squeeze(max(max(abs(results.loadTorqueNm), [], 1), [], 2)).';
summary.maxCurrentA = squeeze(max(max(results.currentA, [], 1), [], 2)).';
summary.maxVoltageEstimateV = squeeze(max(max(results.voltageEstimateV, [], 1), [], 2)).';
summary.maxAbsTorqueAllNm = max(abs(results.torqueNm(:)));
summary.maxAbsLoadTorqueAllNm = max(abs(results.loadTorqueNm(:)));
summary.maxCurrentAllA = max(results.currentA(:));
summary.maxSupportForceN = max(results.supportForceN(:));
summary.maxAbsJointSpeedRadSec = squeeze(max(max(abs(results.jointVelocityRadSec), [], 1), [], 2)).';
summary.maxAbsJointSpeedAllRadSec = max(abs(results.jointVelocityRadSec(:)));
summary.maxFootSpeedMps = max(results.footSpeedMps(:));
summary.unreachableSamples = sum(~results.reachable(:));
summary.jointLimitSamples = sum(results.jointLimitExceeded(:));
summary.currentLimitSamples = sum(results.currentLimitExceeded(:));
summary.voltageLimitSamples = sum(results.voltageLimitExceeded(:));
end
