function config = refreshQuadrupedConfig(config)
%REFRESHQUADRUPEDCONFIG Recompute derived geometry and fill missing defaults.

if ~isfield(config, 'robot')
    config.robot = struct();
end
if ~isfield(config, 'links')
    config.links = struct();
end
if ~isfield(config, 'joints')
    config.joints = struct();
end
if ~isfield(config, 'simulation')
    config.simulation = struct();
end

config.robot.name = getOrDefault(config.robot, 'name', '2DOF Quadruped IK Simulator');
config.robot.bodyLength = positiveOrDefault(getOrDefault(config.robot, 'bodyLength', 0.30), 0.30);
config.robot.bodyWidth = positiveOrDefault(getOrDefault(config.robot, 'bodyWidth', 0.18), 0.18);
config.robot.bodyHeight = positiveOrDefault(getOrDefault(config.robot, 'bodyHeight', 0.16), 0.16);
config.robot.bodyMassKg = nonnegativeOrDefault(getOrDefault(config.robot, 'bodyMassKg', 1.20), 1.20);
config.robot.payloadMassKg = nonnegativeOrDefault(getOrDefault(config.robot, 'payloadMassKg', 0.00), 0.00);

config.links.L1 = positiveOrDefault(getOrDefault(config.links, 'L1', 0.12), 0.12);
config.links.L2 = positiveOrDefault(getOrDefault(config.links, 'L2', 0.12), 0.12);
config.links.m1 = positiveOrDefault(getOrDefault(config.links, 'm1', 0.080), 0.080);
config.links.m2 = positiveOrDefault(getOrDefault(config.links, 'm2', 0.070), 0.070);

config.links.lc1 = positiveOrDefault(getOrDefault(config.links, 'lc1', config.links.L1 / 2), config.links.L1 / 2);
config.links.lc2 = positiveOrDefault(getOrDefault(config.links, 'lc2', config.links.L2 / 2), config.links.L2 / 2);
config.links.lc1 = min(config.links.lc1, config.links.L1);
config.links.lc2 = min(config.links.lc2, config.links.L2);

config.links.I1 = positiveOrDefault(getOrDefault(config.links, 'I1', config.links.m1 * config.links.L1^2 / 12), ...
    config.links.m1 * config.links.L1^2 / 12);
config.links.I2 = positiveOrDefault(getOrDefault(config.links, 'I2', config.links.m2 * config.links.L2^2 / 12), ...
    config.links.m2 * config.links.L2^2 / 12);

config.joints.hipLimitsDeg = getOrDefault(config.joints, 'hipLimitsDeg', [-125, 55]);
config.joints.kneeLimitsDeg = getOrDefault(config.joints, 'kneeLimitsDeg', [-170, -5]);

config.simulation.dt = positiveOrDefault(getOrDefault(config.simulation, 'dt', 0.02), 0.02);
config.simulation.cycles = nonnegativeOrDefault(getOrDefault(config.simulation, 'cycles', 3), 3);
config.simulation.gravity = positiveOrDefault(getOrDefault(config.simulation, 'gravity', 9.80665), 9.80665);
config.simulation.animateEveryNFrames = max(1, round(positiveOrDefault( ...
    getOrDefault(config.simulation, 'animateEveryNFrames', 2), 2)));
config.simulation.maxAnimationFps = positiveOrDefault(getOrDefault(config.simulation, 'maxAnimationFps', 30), 30);

config.legNames = {'FL', 'FR', 'RL', 'RR'};
halfLength = config.robot.bodyLength / 2;
halfWidth = config.robot.bodyWidth / 2;
config.hipPositions = [
     halfLength,  halfWidth, 0;
     halfLength, -halfWidth, 0;
    -halfLength,  halfWidth, 0;
    -halfLength, -halfWidth, 0];
end

function value = getOrDefault(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function value = positiveOrDefault(value, defaultValue)
if ~isnumeric(value) || isempty(value) || any(~isfinite(value(:))) || any(value(:) <= 0)
    value = defaultValue;
end
end

function value = nonnegativeOrDefault(value, defaultValue)
if ~isnumeric(value) || isempty(value) || any(~isfinite(value(:))) || any(value(:) < 0)
    value = defaultValue;
end
end
