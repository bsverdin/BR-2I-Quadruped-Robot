function estimate = estimateBatteryRuntime(results, battery)
%ESTIMATEBATTERYRUNTIME First-pass runtime estimate from simulated current.
%
% The simulator current profile is a torque-derived motor/controller current
% estimate. Treat this as a sizing estimate until it is calibrated against
% real pack current and voltage logs.

if nargin < 2 || isempty(battery)
    battery = defaultQuadrupedConfig().battery;
end
battery = normalizeBatteryConfig(battery);

time = results.time(:);
if isempty(time)
    time = 0;
end
nFrames = numel(time);

motorCurrentA = sum(reshape(results.currentA, nFrames, []), 2);
correctedMotorCurrentA = motorCurrentA * battery.currentSafetyFactor;
auxiliaryCurrentA = battery.auxiliaryDrawA + battery.auxiliaryDrawW / battery.nominalVoltageV;
inputCurrentA = correctedMotorCurrentA + auxiliaryCurrentA;

if isfield(results, 'voltageEstimateV') && ~isempty(results.voltageEstimateV)
    motorPowerW = sum(reshape(results.currentA .* max(results.voltageEstimateV, 0), nFrames, []), 2);
    powerEquivalentCurrentA = motorPowerW / battery.nominalVoltageV;
else
    motorPowerW = nan(nFrames, 1);
    powerEquivalentCurrentA = nan(nFrames, 1);
end

usableAh = battery.capacityAh * battery.usableCapacityPct / 100;
usableWh = usableAh * battery.nominalVoltageV;
usedAh = cumtrapz(time, inputCurrentA) / 3600;
usedWh = cumtrapz(time, inputCurrentA * battery.nominalVoltageV) / 3600;
remainingAh = max(0, usableAh - usedAh);
remainingSocPct = 100 * remainingAh / max(usableAh, eps);

durationS = max(time(end) - time(1), 0);
if durationS > 0
    averageCurrentA = trapz(time, inputCurrentA) / durationS;
    averageMotorCurrentA = trapz(time, motorCurrentA) / durationS;
else
    averageCurrentA = mean(inputCurrentA);
    averageMotorCurrentA = mean(motorCurrentA);
end

peakCurrentA = max(inputCurrentA);
percentile95CurrentA = percentileValue(inputCurrentA, 95);
percentile99CurrentA = percentileValue(inputCurrentA, 99);

if averageCurrentA > 0
    runtimeMinutes = usableAh / averageCurrentA * 60;
else
    runtimeMinutes = inf;
end

crossIndex = find(usedAh >= usableAh, 1, 'first');
if isempty(crossIndex)
    runtimeWithinProfileMinutes = nan;
else
    runtimeWithinProfileMinutes = time(crossIndex) / 60;
end

voltageSagV = inputCurrentA * battery.internalResistanceOhm;
loadedVoltageV = battery.nominalVoltageV - voltageSagV;
if battery.cutoffVoltageV > 0
    cutoffIndex = find(loadedVoltageV <= battery.cutoffVoltageV, 1, 'first');
else
    cutoffIndex = [];
end
if isempty(cutoffIndex)
    cutoffTimeMinutes = nan;
else
    cutoffTimeMinutes = time(cutoffIndex) / 60;
end

estimate = struct();
estimate.battery = battery;
estimate.time = time;
estimate.motorCurrentA = motorCurrentA;
estimate.correctedMotorCurrentA = correctedMotorCurrentA;
estimate.auxiliaryCurrentA = auxiliaryCurrentA;
estimate.inputCurrentA = inputCurrentA;
estimate.motorPowerW = motorPowerW;
estimate.powerEquivalentCurrentA = powerEquivalentCurrentA;
estimate.usedAh = usedAh;
estimate.usedWh = usedWh;
estimate.remainingAh = remainingAh;
estimate.remainingSocPct = remainingSocPct;
estimate.usableAh = usableAh;
estimate.usableWh = usableWh;
estimate.averageCurrentA = averageCurrentA;
estimate.averageMotorCurrentA = averageMotorCurrentA;
estimate.peakCurrentA = peakCurrentA;
estimate.percentile95CurrentA = percentile95CurrentA;
estimate.percentile99CurrentA = percentile99CurrentA;
estimate.runtimeMinutes = runtimeMinutes;
estimate.runtimeWithinProfileMinutes = runtimeWithinProfileMinutes;
estimate.averagePowerW = averageCurrentA * battery.nominalVoltageV;
estimate.voltageSagV = voltageSagV;
estimate.loadedVoltageV = loadedVoltageV;
estimate.cutoffTimeMinutes = cutoffTimeMinutes;
estimate.note = ['First-pass estimate from torque-derived motor current. ', ...
    'Calibrate with real battery current and voltage logs before final pack sizing.'];
end

function battery = normalizeBatteryConfig(battery)
battery.nominalVoltageV = positiveField(battery, 'nominalVoltageV', 24);
battery.capacityAh = positiveField(battery, 'capacityAh', 5);
battery.usableCapacityPct = min(100, max(1, positiveField(battery, 'usableCapacityPct', 85)));
battery.auxiliaryDrawW = nonnegativeField(battery, 'auxiliaryDrawW', 10);
battery.auxiliaryDrawA = nonnegativeField(battery, 'auxiliaryDrawA', 0);
battery.currentSafetyFactor = positiveField(battery, 'currentSafetyFactor', 1.5);
battery.internalResistanceOhm = nonnegativeField(battery, 'internalResistanceOhm', 0);
battery.cutoffVoltageV = nonnegativeField(battery, 'cutoffVoltageV', 0);
end

function value = positiveField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName)) && ...
        isnumeric(s.(fieldName)) && isfinite(s.(fieldName)(1)) && s.(fieldName)(1) > 0
    value = s.(fieldName)(1);
else
    value = defaultValue;
end
end

function value = nonnegativeField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName)) && ...
        isnumeric(s.(fieldName)) && isfinite(s.(fieldName)(1)) && s.(fieldName)(1) >= 0
    value = s.(fieldName)(1);
else
    value = defaultValue;
end
end

function value = percentileValue(values, percentile)
values = sort(values(isfinite(values)));
if isempty(values)
    value = nan;
    return;
end
if isscalar(values)
    value = values(1);
    return;
end
rank = 1 + (numel(values) - 1) * percentile / 100;
lowIndex = floor(rank);
highIndex = ceil(rank);
if lowIndex == highIndex
    value = values(lowIndex);
else
    blend = rank - lowIndex;
    value = values(lowIndex) * (1 - blend) + values(highIndex) * blend;
end
end
