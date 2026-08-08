function motors = importMotorConstants(fileName)
%IMPORTMOTORCONSTANTS Import KT/KV and electrical constants from CSV/XLSX.
%
% Expected columns are flexible and case-insensitive. The example file uses:
% MotorName,Kt_Nm_per_A,Kv_RPM_per_V,NoLoadCurrent_A,MaxCurrent_A,
% Voltage_V,WindingResistance_Ohm,GearRatio,GearEfficiency,
% RobotMass_kg,PayloadMass_kg

if nargin < 1 || isempty(fileName)
    [name, path] = uigetfile({'*.csv;*.xlsx;*.xls;*.txt', 'Motor tables (*.csv, *.xlsx, *.xls, *.txt)'}, ...
        'Import motor constants');
    if isequal(name, 0)
        motors = struct([]);
        return;
    end
    fileName = fullfile(path, name);
end

if ~exist(fileName, 'file')
    error('Motor constants file not found: %s', fileName);
end

[~, ~, ext] = fileparts(fileName);
if any(strcmpi(ext, {'.xlsx', '.xls'}))
    tableData = readtable(fileName);
else
    tableData = readtable(fileName, 'Delimiter', ',');
end

if height(tableData) == 0
    motors = struct([]);
    return;
end

columnKeys = normalizeColumnNames(tableData.Properties.VariableNames);
motors(height(tableData), 1) = defaultMotor();

for row = 1:height(tableData)
    motor = defaultMotor();
    motor.name = char(readText(tableData, columnKeys, row, {'motorname', 'name', 'motor'}, sprintf('Motor %d', row)));
    motor.Kt_Nm_per_A = readNumber(tableData, columnKeys, row, {'ktnmpera', 'kt', 'torqueconstant', 'torqueconstantnmpera'}, nan);
    motor.Kv_RPM_per_V = readNumber(tableData, columnKeys, row, {'kvrpmperv', 'kv', 'speedconstant', 'speedconstantrpmperv'}, nan);
    motor.NoLoadCurrent_A = readNumber(tableData, columnKeys, row, {'noloadcurrenta', 'noloadcurrent', 'io', 'i0'}, 0);
    motor.MaxCurrent_A = readNumber(tableData, columnKeys, row, {'maxcurrenta', 'maxcurrent', 'currentlimita', 'stallcurrenta'}, inf);
    motor.Voltage_V = readNumber(tableData, columnKeys, row, {'voltagev', 'voltage', 'nominalvoltagev', 'supplyv'}, inf);
    motor.WindingResistance_Ohm = readNumber(tableData, columnKeys, row, {'windingresistanceohm', 'resistanceohm', 'phaseohm', 'terminalresistanceohm'}, nan);
    motor.GearRatio = readNumber(tableData, columnKeys, row, {'gearratio', 'reduction', 'gear'}, 9);
    motor.GearEfficiency = readNumber(tableData, columnKeys, row, {'gearefficiency', 'efficiency', 'eta'}, 1);
    motor.RobotMass_kg = readNumber(tableData, columnKeys, row, {'robotmasskg', 'robotmass', 'bodymasskg', 'bodymass'}, nan);
    motor.PayloadMass_kg = readNumber(tableData, columnKeys, row, {'payloadmasskg', 'payloadmass', 'payloadkg', 'payload'}, nan);

    if ~(isfinite(motor.Kt_Nm_per_A) && motor.Kt_Nm_per_A > 0) && ...
            isfinite(motor.Kv_RPM_per_V) && motor.Kv_RPM_per_V > 0
        motor.Kt_Nm_per_A = 60 / (2 * pi * motor.Kv_RPM_per_V);
    end
    if ~(isfinite(motor.Kv_RPM_per_V) && motor.Kv_RPM_per_V > 0) && ...
            isfinite(motor.Kt_Nm_per_A) && motor.Kt_Nm_per_A > 0
        motor.Kv_RPM_per_V = 60 / (2 * pi * motor.Kt_Nm_per_A);
    end
    if ~(isfinite(motor.Kt_Nm_per_A) && motor.Kt_Nm_per_A > 0)
        error('Motor row %d must provide a positive Kt_Nm_per_A or Kv_RPM_per_V.', row);
    end
    if ~(isfinite(motor.Kv_RPM_per_V) && motor.Kv_RPM_per_V > 0)
        motor.Kv_RPM_per_V = 60 / (2 * pi * motor.Kt_Nm_per_A);
    end

    motor.GearRatio = positiveOrDefault(motor.GearRatio, 9);
    motor.GearEfficiency = min(max(positiveOrDefault(motor.GearEfficiency, 1), 0.01), 1);
    motors(row) = motor;
end
end

function motor = defaultMotor()
motor = struct();
motor.name = 'Motor';
motor.Kt_Nm_per_A = 0.080;
motor.Kv_RPM_per_V = 120;
motor.NoLoadCurrent_A = 0;
motor.MaxCurrent_A = inf;
motor.Voltage_V = inf;
motor.WindingResistance_Ohm = nan;
motor.GearRatio = 9;
motor.GearEfficiency = 1;
motor.RobotMass_kg = nan;
motor.PayloadMass_kg = nan;
end

function keys = normalizeColumnNames(names)
keys = cell(size(names));
for i = 1:numel(names)
    keys{i} = lower(regexprep(names{i}, '[^a-zA-Z0-9]', ''));
end
end

function value = readNumber(tableData, keys, row, candidates, defaultValue)
value = defaultValue;
idx = findColumn(keys, candidates);
if isempty(idx)
    return;
end
raw = tableData{row, idx};
if iscell(raw)
    raw = raw{1};
end
if isstring(raw) || ischar(raw)
    raw = str2double(raw);
end
if isnumeric(raw) && ~isempty(raw)
    value = raw(1);
end
if isempty(value) || (~isfinite(value) && isfinite(defaultValue))
    value = defaultValue;
end
end

function value = readText(tableData, keys, row, candidates, defaultValue)
value = defaultValue;
idx = findColumn(keys, candidates);
if isempty(idx)
    return;
end
raw = tableData{row, idx};
if iscell(raw)
    raw = raw{1};
end
if isstring(raw)
    raw = char(raw);
elseif isnumeric(raw)
    raw = num2str(raw(1));
end
if ischar(raw) && ~isempty(raw)
    value = raw;
end
end

function idx = findColumn(keys, candidates)
idx = [];
for c = 1:numel(candidates)
    hit = find(strcmp(keys, candidates{c}), 1, 'first');
    if ~isempty(hit)
        idx = hit;
        return;
    end
end
end

function value = positiveOrDefault(value, defaultValue)
if ~isnumeric(value) || isempty(value) || ~isfinite(value(1)) || value(1) <= 0
    value = defaultValue;
else
    value = value(1);
end
end
