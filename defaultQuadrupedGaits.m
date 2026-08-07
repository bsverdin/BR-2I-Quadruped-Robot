function gaits = defaultQuadrupedGaits(config)
%DEFAULTQUADRUPEDGAITS Built-in gait presets for FL, FR, RL, RR leg order.

height = config.robot.bodyHeight;

gaits(1) = makeGait('Walk', 1.20, 0.70, 0.075, 0.035, [0.00, 0.50, 0.75, 0.25], height, ...
    'Four-beat walking gait.');
gaits(2) = makeGait('Trot', 0.70, 0.58, 0.090, 0.040, [0.00, 0.50, 0.50, 0.00], height, ...
    'Diagonal pairs move together.');
gaits(3) = makeGait('Pace', 0.80, 0.60, 0.085, 0.040, [0.00, 0.50, 0.00, 0.50], height, ...
    'Same-side legs move together.');
gaits(4) = makeGait('Bound', 0.62, 0.55, 0.095, 0.045, [0.00, 0.00, 0.50, 0.50], height, ...
    'Front pair and rear pair alternate.');
gaits(5) = makeGait('Crawl', 1.70, 0.78, 0.060, 0.030, [0.00, 0.25, 0.50, 0.75], height, ...
    'Slow high-duty-factor crawl.');
end

function gait = makeGait(name, period, dutyFactor, stepLength, clearance, phaseOffsets, bodyHeight, description)
gait = struct();
gait.name = name;
gait.period = period;
gait.dutyFactor = dutyFactor;
gait.stepLength = stepLength;
gait.clearance = clearance;
gait.phaseOffsets = phaseOffsets;
gait.bodyHeight = bodyHeight;
gait.description = description;
end
