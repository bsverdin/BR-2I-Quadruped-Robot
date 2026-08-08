function quadrupedIKSimulator()
%QUADRUPEDIKSIMULATOR Interactive 3D IK, torque, and current simulation.
%
% Run from MATLAB with:
%   quadrupedIKSimulator

config = bevelGearConceptConfig();

app = struct();
app.config = config;
app.gaits = config.gaits;
app.motor = config.motor;
app.battery = config.battery;
app.batteryEstimate = [];
app.results = [];
app.stopRequested = false;
app.robotGraphics = [];
app.plotGraphics = [];

fig = figure( ...
    'Name', '2DOF Quadruped IK Simulator', ...
    'NumberTitle', 'off', ...
    'Color', [0.94, 0.96, 0.98], ...
    'Units', 'normalized', ...
    'Position', [0.05, 0.07, 0.90, 0.84]);

app.handles = buildControls(fig, app);
guidata(fig, app);
updatePreview(fig);
setStatus(fig, 'Ready. Choose a gait, edit dimensions or motor constants, then run the simulation.');
end

function handles = buildControls(fig, app)
handles = struct();
uiColor = get(fig, 'Color');
panelColor = [0.985, 0.990, 0.995];
panelEdgeColor = [0.62, 0.69, 0.76];
textColor = [0.06, 0.09, 0.12];
mutedTextColor = [0.22, 0.27, 0.32];
buttonColor = [0.98, 0.99, 1.00];
buttonAccent = [0.86, 0.93, 1.00];
runColor = [0.76, 0.92, 0.77];
stopColor = [0.98, 0.80, 0.78];

handles.titleText = uicontrol(fig, 'Style', 'text', ...
    'String', '2DOF Quadruped IK, Torque, Current, and Battery Simulator', ...
    'Units', 'normalized', 'Position', [0.035, 0.952, 0.930, 0.035], ...
    'BackgroundColor', uiColor, 'ForegroundColor', textColor, ...
    'HorizontalAlignment', 'left', 'FontSize', 17, 'FontWeight', 'bold');

handles.robotPanel = uipanel(fig, 'Title', '3D Robot View', ...
    'Units', 'normalized', 'Position', [0.035, 0.285, 0.555, 0.645], ...
    'BackgroundColor', panelColor, 'ForegroundColor', textColor, ...
    'BorderType', 'line', 'BorderWidth', 1, 'BorderColor', panelEdgeColor, ...
    'FontSize', 11, 'FontWeight', 'bold');
handles.anglesPanel = uipanel(fig, 'Title', 'Joint Angles', ...
    'Units', 'normalized', 'Position', [0.615, 0.742, 0.350, 0.288], ...
    'BackgroundColor', panelColor, 'ForegroundColor', textColor, ...
    'BorderType', 'line', 'BorderWidth', 1, 'BorderColor', panelEdgeColor, ...
    'FontSize', 11, 'FontWeight', 'bold');
handles.torquePanel = uipanel(fig, 'Title', 'Joint Torque', ...
    'Units', 'normalized', 'Position', [0.615, 0.515, 0.350, 0.288], ...
    'BackgroundColor', panelColor, 'ForegroundColor', textColor, ...
    'BorderType', 'line', 'BorderWidth', 1, 'BorderColor', panelEdgeColor, ...
    'FontSize', 11, 'FontWeight', 'bold');
handles.currentPanel = uipanel(fig, 'Title', 'Motor Current Draw', ...
    'Units', 'normalized', 'Position', [0.615, 0.288, 0.350, 0.288], ...
    'BackgroundColor', panelColor, 'ForegroundColor', textColor, ...
    'BorderType', 'line', 'BorderWidth', 1, 'BorderColor', panelEdgeColor, ...
    'FontSize', 11, 'FontWeight', 'bold');

handles.axRobot = axes('Parent', handles.robotPanel, 'Units', 'normalized', 'Position', [0.055, 0.075, 0.900, 0.860]);
handles.axAngles = axes('Parent', handles.anglesPanel, 'Units', 'normalized', 'Position', [0.100, 0.185, 0.865, 0.635]);
handles.axTorque = axes('Parent', handles.torquePanel, 'Units', 'normalized', 'Position', [0.100, 0.185, 0.865, 0.635]);
handles.axCurrent = axes('Parent', handles.currentPanel, 'Units', 'normalized', 'Position', [0.100, 0.185, 0.865, 0.635]);
set([handles.axRobot, handles.axAngles, handles.axTorque, handles.axCurrent], ...
    'FontSize', 10.5, 'LineWidth', 1.1, 'Box', 'on', 'Color', [1, 1, 1]);

handles.metricsPanel = uipanel(fig, 'Title', 'Live Metrics', ...
    'Units', 'normalized', 'Position', [0.055, 0.705, 0.275, 0.205], ...
    'BackgroundColor', [1, 1, 1], 'ForegroundColor', textColor, ...
    'BorderType', 'line', 'BorderWidth', 1.2, 'BorderColor', [0.28, 0.36, 0.44], ...
    'FontSize', 10.5, 'FontWeight', 'bold');
handles.metricsText = uicontrol(handles.metricsPanel, 'Style', 'text', ...
    'String', sprintf('Gait      --\nTime      --\nTorque    --\nCurrent   --\nLoad      --\nSpeed     --\nLimits    --'), ...
    'Units', 'normalized', 'Position', [0.035, 0.035, 0.930, 0.860], ...
    'BackgroundColor', [1, 1, 1], 'ForegroundColor', textColor, ...
    'HorizontalAlignment', 'left', 'FontName', 'Consolas', 'FontSize', 9.5);

handles.controlPanel = uipanel(fig, 'Title', 'Controls and Status', ...
    'Units', 'normalized', 'Position', [0.035, 0.035, 0.930, 0.225], ...
    'BackgroundColor', panelColor, 'ForegroundColor', textColor, ...
    'BorderType', 'line', 'BorderWidth', 1, 'BorderColor', panelEdgeColor, ...
    'FontSize', 11, 'FontWeight', 'bold');

uicontrol(handles.controlPanel, 'Style', 'text', 'String', 'Gait', 'Units', 'normalized', ...
    'Position', [0.015, 0.720, 0.045, 0.160], 'BackgroundColor', panelColor, ...
    'ForegroundColor', mutedTextColor, 'HorizontalAlignment', 'left', ...
    'FontWeight', 'bold', 'FontSize', 10.5);
handles.gaitPopup = uicontrol(handles.controlPanel, 'Style', 'popupmenu', 'String', {app.gaits.name}, ...
    'Units', 'normalized', 'Position', [0.060, 0.705, 0.145, 0.200], ...
    'FontSize', 10.5, 'BackgroundColor', [1, 1, 1], ...
    'Callback', @(~, ~) updatePreview(fig));

handles.runButton = uicontrol(handles.controlPanel, 'Style', 'pushbutton', 'String', 'Run', ...
    'Units', 'normalized', 'Position', [0.220, 0.700, 0.070, 0.215], ...
    'BackgroundColor', runColor, 'ForegroundColor', textColor, ...
    'FontSize', 10.5, 'FontWeight', 'bold', ...
    'Callback', @(~, ~) runSimulation(fig));
handles.stopButton = uicontrol(handles.controlPanel, 'Style', 'pushbutton', 'String', 'Stop', ...
    'Units', 'normalized', 'Position', [0.298, 0.700, 0.070, 0.215], ...
    'BackgroundColor', stopColor, 'ForegroundColor', textColor, ...
    'FontSize', 10.5, 'FontWeight', 'bold', ...
    'Callback', @(~, ~) requestStop(fig));
handles.editGaitButton = uicontrol(handles.controlPanel, 'Style', 'pushbutton', 'String', 'Edit Gait', ...
    'Units', 'normalized', 'Position', [0.015, 0.470, 0.090, 0.190], ...
    'BackgroundColor', buttonAccent, 'ForegroundColor', textColor, 'FontSize', 10.5, ...
    'Callback', @(~, ~) editGait(fig));
handles.editRobotButton = uicontrol(handles.controlPanel, 'Style', 'pushbutton', 'String', 'Edit Robot/Motor', ...
    'Units', 'normalized', 'Position', [0.112, 0.470, 0.135, 0.190], ...
    'BackgroundColor', buttonAccent, 'ForegroundColor', textColor, 'FontSize', 10.5, ...
    'Callback', @(~, ~) editRobotMotor(fig));
handles.importMotorButton = uicontrol(handles.controlPanel, 'Style', 'pushbutton', 'String', 'Import Motor', ...
    'Units', 'normalized', 'Position', [0.254, 0.470, 0.110, 0.190], ...
    'BackgroundColor', buttonAccent, 'ForegroundColor', textColor, 'FontSize', 10.5, ...
    'Callback', @(~, ~) importMotor(fig));
handles.poseButton = uicontrol(handles.controlPanel, 'Style', 'pushbutton', 'String', 'Analyze Pose', ...
    'Units', 'normalized', 'Position', [0.015, 0.240, 0.115, 0.190], ...
    'BackgroundColor', buttonColor, 'ForegroundColor', textColor, 'FontSize', 10.5, ...
    'Callback', @(~, ~) analyzePose(fig));
handles.batteryButton = uicontrol(handles.controlPanel, 'Style', 'pushbutton', 'String', 'Battery', ...
    'Units', 'normalized', 'Position', [0.137, 0.240, 0.085, 0.190], ...
    'BackgroundColor', buttonColor, 'ForegroundColor', textColor, 'FontSize', 10.5, ...
    'Callback', @(~, ~) editBatteryEstimator(fig));

handles.saveGaitButton = uicontrol(handles.controlPanel, 'Style', 'pushbutton', 'String', 'Save Gait', ...
    'Units', 'normalized', 'Position', [0.230, 0.240, 0.085, 0.190], ...
    'BackgroundColor', buttonColor, 'ForegroundColor', textColor, 'FontSize', 10.5, ...
    'Callback', @(~, ~) saveGait(fig));
handles.loadGaitButton = uicontrol(handles.controlPanel, 'Style', 'pushbutton', 'String', 'Load Gait', ...
    'Units', 'normalized', 'Position', [0.322, 0.240, 0.085, 0.190], ...
    'BackgroundColor', buttonColor, 'ForegroundColor', textColor, 'FontSize', 10.5, ...
    'Callback', @(~, ~) loadGait(fig));
handles.exportResultsButton = uicontrol(handles.controlPanel, 'Style', 'pushbutton', 'String', 'Export Results', ...
    'Units', 'normalized', 'Position', [0.230, 0.020, 0.177, 0.180], ...
    'BackgroundColor', buttonColor, 'ForegroundColor', textColor, 'FontSize', 10.5, ...
    'Callback', @(~, ~) exportResults(fig));

handles.infoPanel = uipanel(handles.controlPanel, 'Title', 'Robot, Motor, and Battery', ...
    'Units', 'normalized', 'Position', [0.425, 0.270, 0.370, 0.640], ...
    'BackgroundColor', [1, 1, 1], 'ForegroundColor', textColor, ...
    'BorderType', 'line', 'BorderWidth', 1, 'BorderColor', panelEdgeColor, ...
    'FontSize', 10.5, 'FontWeight', 'bold');
handles.motorText = uicontrol(handles.infoPanel, 'Style', 'text', 'String', motorLabel(app.motor, app.config), ...
    'Units', 'normalized', 'Position', [0.030, 0.485, 0.940, 0.420], ...
    'BackgroundColor', [1, 1, 1], 'HorizontalAlignment', 'left', ...
    'FontSize', 9.5, 'ForegroundColor', textColor);
handles.batteryText = uicontrol(handles.infoPanel, 'Style', 'text', 'String', batteryLabel([], app.battery), ...
    'Units', 'normalized', 'Position', [0.030, 0.075, 0.940, 0.365], ...
    'BackgroundColor', [1, 1, 1], 'HorizontalAlignment', 'left', ...
    'FontSize', 9.5, 'ForegroundColor', textColor);

handles.limitPanel = uipanel(handles.controlPanel, 'Title', 'Motor Limit Check', ...
    'Units', 'normalized', 'Position', [0.810, 0.270, 0.170, 0.640], ...
    'BackgroundColor', [1, 1, 1], 'ForegroundColor', textColor, ...
    'BorderType', 'line', 'BorderWidth', 1, 'BorderColor', panelEdgeColor, ...
    'FontSize', 10.5, 'FontWeight', 'bold');
handles.motorStatusText = uicontrol(handles.limitPanel, 'Style', 'text', 'String', sprintf('CHECK\nRun gait'), ...
    'Units', 'normalized', 'Position', [0.055, 0.080, 0.890, 0.800], ...
    'BackgroundColor', [0.88, 0.90, 0.93], 'ForegroundColor', textColor, ...
    'HorizontalAlignment', 'center', 'FontName', 'Consolas', 'FontSize', 9.5, ...
    'FontWeight', 'bold');

handles.statusPanel = uipanel(handles.controlPanel, 'Title', 'Status', ...
    'Units', 'normalized', 'Position', [0.425, 0.020, 0.555, 0.220], ...
    'BackgroundColor', [0.91, 0.95, 0.99], 'ForegroundColor', textColor, ...
    'BorderType', 'line', 'BorderWidth', 1, 'BorderColor', panelEdgeColor, ...
    'FontSize', 9.5, 'FontWeight', 'bold');
handles.statusText = uicontrol(handles.statusPanel, 'Style', 'text', 'String', '', ...
    'Units', 'normalized', 'Position', [0.015, 0.080, 0.970, 0.760], ...
    'BackgroundColor', [0.91, 0.95, 0.99], 'ForegroundColor', textColor, ...
    'HorizontalAlignment', 'left', 'FontSize', 9.5);
end

function runSimulation(fig)
app = guidata(fig);
app.stopRequested = false;
guidata(fig, app);

gait = activeGait(app);
setStatus(fig, sprintf('Simulating %s gait...', gait.name));
drawnow;

try
    results = simulateQuadrupedGait(app.config, gait, app.motor);
catch err
    setStatus(fig, ['Simulation failed: ', err.message]);
    rethrow(err);
end

app = guidata(fig);
app.results = results;
app.batteryEstimate = estimateBatteryRuntime(results, app.battery);
app.motorCapability = evaluateMotorCapability(results, app.motor);
app.robotGraphics = initializeRobotScene(app.handles.axRobot, app.config, results);
app.handles.livePlots = initializeLivePlots(app, results);
set(app.handles.batteryText, 'String', batteryLabel(app.batteryEstimate, app.battery));
updateMotorStatusLight(app.handles.motorStatusText, app.motorCapability);
guidata(fig, app);

updateMetricsPanel(fig, results, 1);
setStatus(fig, summarizeStatus(results));
animateResults(fig, results);
end

function requestStop(fig)
app = guidata(fig);
app.stopRequested = true;
guidata(fig, app);
setStatus(fig, 'Stop requested. Animation will pause at the next frame.');
end

function updatePreview(fig)
if ~ishandle(fig)
    return;
end
app = guidata(fig);
if isempty(app)
    return;
end
previewConfig = app.config;
previewConfig.simulation.cycles = 0;
gait = activeGait(app);
try
    results = simulateQuadrupedGait(previewConfig, gait, app.motor);
    drawRobotFrame(app.handles.axRobot, previewConfig, results, 1);
    updateMetricsPanel(fig, results, 1);
catch err
    cla(app.handles.axRobot);
    title(app.handles.axRobot, ['Preview failed: ', err.message]);
    set(app.handles.metricsText, 'String', 'Preview failed. Check gait and geometry settings.');
end
end

function animateResults(fig, results)
app = guidata(fig);
frameStep = app.config.simulation.animateEveryNFrames;
maxFps = app.config.simulation.maxAnimationFps;
nFrames = numel(results.time);
lastPlotIndex = 0;
if nFrames > 1
    framePause = min(1 / maxFps, max(0.001, frameStep * median(diff(results.time))));
else
    framePause = 0.001;
end

for k = 1:frameStep:nFrames
    if ~ishandle(fig)
        return;
    end
    app = guidata(fig);
    if app.stopRequested
        app.stopRequested = false;
        guidata(fig, app);
        setStatus(fig, 'Animation stopped. Results and plots are still available.');
        return;
    end
    updateRobotFrame(app.robotGraphics, results, k);
    updateLivePlots(app.handles.livePlots, results, lastPlotIndex + 1, k);
    updateMetricsPanel(fig, results, k);
    lastPlotIndex = k;
    drawnow limitrate;
    pause(framePause);
end
if lastPlotIndex < nFrames
    app = guidata(fig);
    updateRobotFrame(app.robotGraphics, results, nFrames);
    updateLivePlots(app.handles.livePlots, results, lastPlotIndex + 1, nFrames);
    updateMetricsPanel(fig, results, nFrames);
    drawnow limitrate;
end
setStatus(fig, [summarizeStatus(results), ' Animation complete.']);
end

function robotGraphics = initializeRobotScene(ax, config, results)
cla(ax);
hold(ax, 'on');
grid(ax, 'on');
set(ax, 'Color', [0.985, 0.990, 0.995], 'GridColor', [0.70, 0.75, 0.80], ...
    'MinorGridColor', [0.84, 0.87, 0.90], 'FontSize', 10, 'LineWidth', 1.0, ...
    'XColor', [0.08, 0.11, 0.14], 'YColor', [0.08, 0.11, 0.14], ...
    'ZColor', [0.08, 0.11, 0.14]);
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';
ax.ZMinorGrid = 'on';

colors = legColorMap();
hips = config.hipPositions;
bodyLoop = hips([1, 2, 4, 3, 1], :);
reach = config.links.L1 + config.links.L2;
margin = 0.15;
xLimits = [-config.robot.bodyLength / 2 - reach - margin, config.robot.bodyLength / 2 + reach + margin];
yLimits = [-config.robot.bodyWidth / 2 - margin, config.robot.bodyWidth / 2 + margin];
zLimits = [-config.robot.bodyHeight - config.links.L1 - margin, reach * 0.35];

floorZ = -config.robot.bodyHeight;
floorPatch = patch('Parent', ax, ...
    'XData', xLimits([1, 2, 2, 1]), 'YData', yLimits([1, 1, 2, 2]), ...
    'ZData', [floorZ, floorZ, floorZ, floorZ], ...
    'FaceColor', [0.58, 0.66, 0.72], 'FaceAlpha', 0.10, ...
    'EdgeColor', [0.68, 0.74, 0.80], 'LineStyle', ':');
bodyPatch = patch('Parent', ax, ...
    'XData', bodyLoop(:, 1), 'YData', bodyLoop(:, 2), 'ZData', bodyLoop(:, 3), ...
    'FaceColor', [0.74, 0.80, 0.86], 'FaceAlpha', 0.55, ...
    'EdgeColor', [0.16, 0.20, 0.24], 'LineWidth', 1.8);

legLines = gobjects(4, 1);
targetMarkers = gobjects(4, 1);
targetLinks = gobjects(4, 1);
legLabels = gobjects(4, 1);
for leg = 1:4
    legLines(leg) = plot3(ax, nan, nan, nan, '-o', ...
        'Color', colors(leg, :), 'MarkerFaceColor', colors(leg, :), ...
        'MarkerEdgeColor', [0.04, 0.04, 0.04], 'LineWidth', 3.0, 'MarkerSize', 6);
    targetMarkers(leg) = plot3(ax, nan, nan, nan, 'x', ...
        'Color', [0.82, 0.05, 0.05], 'LineWidth', 1.8, 'MarkerSize', 8, ...
        'Visible', 'off');
    targetLinks(leg) = plot3(ax, nan, nan, nan, ':', ...
        'Color', [0.82, 0.05, 0.05], 'LineWidth', 1.2, 'Visible', 'off');
    legLabels(leg) = text(ax, nan, nan, nan, results.legNames{leg}, ...
        'Color', colors(leg, :), 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'FontSize', 10);
end

titleHandle = title(ax, '', 'FontSize', 12, 'FontWeight', 'bold', ...
    'Color', [0.07, 0.09, 0.12]);
xlabel(ax, 'X forward (m)', 'FontWeight', 'bold', 'Color', [0.08, 0.11, 0.14]);
ylabel(ax, 'Y left (m)', 'FontWeight', 'bold', 'Color', [0.08, 0.11, 0.14]);
zlabel(ax, 'Z up (m)', 'FontWeight', 'bold', 'Color', [0.08, 0.11, 0.14]);
set(ax, 'XLim', xLimits, 'YLim', yLimits, 'ZLim', zLimits);
daspect(ax, [1, 1, 1]);
axis(ax, 'vis3d');
view(ax, 38, 22);

robotGraphics = struct();
robotGraphics.floorPatch = floorPatch;
robotGraphics.bodyPatch = bodyPatch;
robotGraphics.legLines = legLines;
robotGraphics.targetMarkers = targetMarkers;
robotGraphics.targetLinks = targetLinks;
robotGraphics.legLabels = legLabels;
robotGraphics.titleHandle = titleHandle;

updateRobotFrame(robotGraphics, results, 1);
hold(ax, 'off');
end

function updateRobotFrame(robotGraphics, results, frameIndex)
frameIndex = min(max(frameIndex, 1), numel(results.time));
for leg = 1:4
    hip = squeeze(results.hipPosition(frameIndex, leg, :)).';
    knee = squeeze(results.kneePosition(frameIndex, leg, :)).';
    foot = squeeze(results.footPosition(frameIndex, leg, :)).';
    target = squeeze(results.targetFootPosition(frameIndex, leg, :)).';

    set(robotGraphics.legLines(leg), ...
        'XData', [hip(1), knee(1), foot(1)], ...
        'YData', [hip(2), knee(2), foot(2)], ...
        'ZData', [hip(3), knee(3), foot(3)]);
    set(robotGraphics.legLabels(leg), ...
        'Position', [foot(1), foot(2), foot(3) - 0.012]);

    if ~results.reachable(frameIndex, leg)
        set(robotGraphics.targetMarkers(leg), ...
            'XData', target(1), 'YData', target(2), 'ZData', target(3), 'Visible', 'on');
        set(robotGraphics.targetLinks(leg), ...
            'XData', [foot(1), target(1)], ...
            'YData', [foot(2), target(2)], ...
            'ZData', [foot(3), target(3)], 'Visible', 'on');
    else
        set(robotGraphics.targetMarkers(leg), 'Visible', 'off');
        set(robotGraphics.targetLinks(leg), 'Visible', 'off');
    end
end

set(robotGraphics.titleHandle, 'String', sprintf('%s | t %.2f s | now %.3f Nm, %.3f A', ...
    results.gait.name, results.time(frameIndex), ...
    results.frameMaxTorqueNm(frameIndex), results.frameMaxCurrentA(frameIndex)));
end

function plotGraphics = initializeLivePlots(app, results)
colors = legColorMap();
lineStyles = {'-', '--'};
lineLabels = cell(1, 8);

plotGraphics = struct();
plotGraphics.angleLines = gobjects(4, 2);
plotGraphics.torqueLines = gobjects(4, 2);
plotGraphics.currentLines = gobjects(4, 2);

timeLimits = [0, max(results.time(end), eps)];
angleLimits = paddedLimits(results.jointAnglesDeg(:), 5);
torqueLimits = paddedLimits(results.torqueNm(:), 0.05);
currentLimits = paddedLimits(results.currentA(:), 0.05);

configureLiveAxis(app.handles.axAngles, timeLimits, angleLimits, 'Live Joint Movement', 'Degrees');
configureLiveAxis(app.handles.axTorque, timeLimits, torqueLimits, 'Live Joint Torque', 'Torque (Nm)');
configureLiveAxis(app.handles.axCurrent, timeLimits, currentLimits, 'Live Motor Current Draw', 'Current (A)');

angleLegend = gobjects(1, 8);
torqueLegend = gobjects(1, 8);
currentLegend = gobjects(1, 8);
legendIndex = 0;
for leg = 1:4
    for joint = 1:2
        legendIndex = legendIndex + 1;
        lineLabels{legendIndex} = [results.legNames{leg}, ' ', lower(results.jointNames{joint})];
        plotGraphics.angleLines(leg, joint) = animatedline(app.handles.axAngles, ...
            'Color', colors(leg, :), 'LineStyle', lineStyles{joint}, 'LineWidth', 2.0);
        plotGraphics.torqueLines(leg, joint) = animatedline(app.handles.axTorque, ...
            'Color', colors(leg, :), 'LineStyle', lineStyles{joint}, 'LineWidth', 2.0);
        plotGraphics.currentLines(leg, joint) = animatedline(app.handles.axCurrent, ...
            'Color', colors(leg, :), 'LineStyle', lineStyles{joint}, 'LineWidth', 2.0);
        angleLegend(legendIndex) = plotGraphics.angleLines(leg, joint);
        torqueLegend(legendIndex) = plotGraphics.torqueLines(leg, joint);
        currentLegend(legendIndex) = plotGraphics.currentLines(leg, joint);
    end
end

plotGraphics.angleCursor = line(app.handles.axAngles, [0, 0], angleLimits, ...
    'Color', [0.05, 0.08, 0.10], 'LineStyle', ':', 'LineWidth', 1.4, 'HandleVisibility', 'off');
plotGraphics.torqueCursor = line(app.handles.axTorque, [0, 0], torqueLimits, ...
    'Color', [0.05, 0.08, 0.10], 'LineStyle', ':', 'LineWidth', 1.4, 'HandleVisibility', 'off');
plotGraphics.currentCursor = line(app.handles.axCurrent, [0, 0], currentLimits, ...
    'Color', [0.05, 0.08, 0.10], 'LineStyle', ':', 'LineWidth', 1.4, 'HandleVisibility', 'off');

styleLegend(legend(app.handles.axAngles, angleLegend, lineLabels, 'Location', 'northoutside', ...
    'NumColumns', 4, 'FontSize', 8.5));
styleLegend(legend(app.handles.axTorque, torqueLegend, lineLabels, 'Location', 'northoutside', ...
    'NumColumns', 4, 'FontSize', 8.5));
styleLegend(legend(app.handles.axCurrent, currentLegend, lineLabels, 'Location', 'northoutside', ...
    'NumColumns', 4, 'FontSize', 8.5));
end

function updateLivePlots(plotGraphics, results, startIndex, endIndex)
if isempty(plotGraphics) || startIndex > endIndex
    return;
end
startIndex = max(1, startIndex);
endIndex = min(numel(results.time), endIndex);
timeSegment = results.time(startIndex:endIndex).';

for leg = 1:4
    for joint = 1:2
        addpoints(plotGraphics.angleLines(leg, joint), timeSegment, ...
            reshape(results.jointAnglesDeg(startIndex:endIndex, leg, joint), 1, []));
        addpoints(plotGraphics.torqueLines(leg, joint), timeSegment, ...
            reshape(results.torqueNm(startIndex:endIndex, leg, joint), 1, []));
        addpoints(plotGraphics.currentLines(leg, joint), timeSegment, ...
            reshape(results.currentA(startIndex:endIndex, leg, joint), 1, []));
    end
end

cursorTime = results.time(endIndex);
set(plotGraphics.angleCursor, 'XData', [cursorTime, cursorTime]);
set(plotGraphics.torqueCursor, 'XData', [cursorTime, cursorTime]);
set(plotGraphics.currentCursor, 'XData', [cursorTime, cursorTime]);
end

function configureLiveAxis(ax, timeLimits, yLimits, titleText, yLabelText)
cla(ax);
hold(ax, 'on');
grid(ax, 'on');
set(ax, 'XLim', timeLimits, 'YLim', yLimits, 'FontSize', 9, ...
    'LineWidth', 1.1, 'GridColor', [0.68, 0.74, 0.80], ...
    'MinorGridColor', [0.84, 0.88, 0.92], ...
    'XColor', [0.08, 0.11, 0.14], 'YColor', [0.08, 0.11, 0.14]);
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';
xlabel(ax, 'Seconds', 'FontWeight', 'bold', 'Color', [0.08, 0.11, 0.14]);
ylabel(ax, yLabelText, 'FontWeight', 'bold', 'Color', [0.08, 0.11, 0.14]);
title(ax, titleText, 'FontSize', 11.5, 'FontWeight', 'bold', 'Color', [0.07, 0.09, 0.12]);
end

function colors = legColorMap()
colors = [
    0.000, 0.300, 0.950
    0.900, 0.270, 0.000
    0.000, 0.560, 0.220
    0.620, 0.120, 0.920];
end

function styleLegend(legendHandle)
set(legendHandle, 'Color', [1, 1, 1], 'TextColor', [0.06, 0.08, 0.10], ...
    'EdgeColor', [0.25, 0.28, 0.32]);
end

function limits = paddedLimits(values, fallbackPadding)
values = values(isfinite(values));
if isempty(values)
    limits = [-fallbackPadding, fallbackPadding];
    return;
end
lo = min(values);
hi = max(values);
if abs(hi - lo) < eps
    pad = max(abs(hi) * 0.10, fallbackPadding);
else
    pad = max((hi - lo) * 0.08, fallbackPadding);
end
limits = [lo - pad, hi + pad];
end

function drawRobotFrame(ax, config, results, frameIndex)
cla(ax);
hold(ax, 'on');
grid(ax, 'on');
set(ax, 'Color', [0.985, 0.990, 0.995], 'GridColor', [0.70, 0.75, 0.80], ...
    'MinorGridColor', [0.84, 0.87, 0.90], 'FontSize', 10, 'LineWidth', 1.0, ...
    'XColor', [0.08, 0.11, 0.14], 'YColor', [0.08, 0.11, 0.14], ...
    'ZColor', [0.08, 0.11, 0.14]);
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';
ax.ZMinorGrid = 'on';
colors = legColorMap();

hips = config.hipPositions;
bodyLoop = hips([1, 2, 4, 3, 1], :);
patch('Parent', ax, ...
    'XData', bodyLoop(:, 1), 'YData', bodyLoop(:, 2), 'ZData', bodyLoop(:, 3), ...
    'FaceColor', [0.82, 0.86, 0.90], 'FaceAlpha', 0.35, ...
    'EdgeColor', [0.20, 0.24, 0.28], 'LineWidth', 1.5);

for leg = 1:4
    hip = squeeze(results.hipPosition(frameIndex, leg, :)).';
    knee = squeeze(results.kneePosition(frameIndex, leg, :)).';
    foot = squeeze(results.footPosition(frameIndex, leg, :)).';
    target = squeeze(results.targetFootPosition(frameIndex, leg, :)).';

    plot3(ax, [hip(1), knee(1), foot(1)], [hip(2), knee(2), foot(2)], [hip(3), knee(3), foot(3)], ...
        '-o', 'Color', colors(leg, :), 'MarkerFaceColor', colors(leg, :), ...
        'MarkerEdgeColor', [0.05, 0.05, 0.05], 'LineWidth', 2.2, 'MarkerSize', 5);
    if ~results.reachable(frameIndex, leg)
        plot3(ax, target(1), target(2), target(3), 'x', 'Color', [0.80, 0.05, 0.05], ...
            'LineWidth', 1.8, 'MarkerSize', 8);
        plot3(ax, [foot(1), target(1)], [foot(2), target(2)], [foot(3), target(3)], ':', ...
            'Color', [0.80, 0.05, 0.05], 'LineWidth', 1.0);
    end
    text(ax, foot(1), foot(2), foot(3) - 0.012, results.legNames{leg}, ...
        'Color', colors(leg, :), 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
end

maxTorque = max(abs(reshape(results.torqueNm(frameIndex, :, :), 1, [])));
maxCurrent = max(reshape(results.currentA(frameIndex, :, :), 1, []));
title(ax, sprintf('%s | t = %.2f s | %.2f Nm max | %.2f A max', ...
    results.gait.name, results.time(frameIndex), maxTorque, maxCurrent), ...
    'Color', [0.07, 0.09, 0.12], 'FontWeight', 'bold');
xlabel(ax, 'X forward (m)', 'Color', [0.08, 0.11, 0.14], 'FontWeight', 'bold');
ylabel(ax, 'Y left (m)', 'Color', [0.08, 0.11, 0.14], 'FontWeight', 'bold');
zlabel(ax, 'Z up (m)', 'Color', [0.08, 0.11, 0.14], 'FontWeight', 'bold');

reach = config.links.L1 + config.links.L2;
margin = 0.06;
xlim(ax, [-config.robot.bodyLength / 2 - reach - margin, config.robot.bodyLength / 2 + reach + margin]);
ylim(ax, [-config.robot.bodyWidth / 2 - margin, config.robot.bodyWidth / 2 + margin]);
zlim(ax, [-config.robot.bodyHeight - config.links.L1 - margin, reach * 0.35]);
axis(ax, 'equal');
view(ax, 38, 22);
hold(ax, 'off');
end

function editGait(fig)
app = guidata(fig);
idx = get(app.handles.gaitPopup, 'Value');
gait = app.gaits(idx);

defaults = {
    char(gait.name)
    num2str(gait.period)
    num2str(gait.dutyFactor)
    num2str(gait.stepLength)
    num2str(gait.clearance)
    num2str(gait.bodyHeight)
    num2str(gait.phaseOffsets)
    char(gait.description)
    };
prompt = {
    'Gait name'
    'Period (s)'
    'Duty factor, 0.05 to 0.95'
    'Step length (m)'
    'Foot clearance (m)'
    'Body height / neutral foot drop (m)'
    'Phase offsets for FL FR RL RR, 0 to 1'
    'Description'
    };
answer = settingsDialog(prompt, 'Edit gait', defaults);
if isempty(answer)
    return;
end

gait.name = answer{1};
gait.period = scalarOrDefault(answer{2}, gait.period);
gait.dutyFactor = min(max(scalarOrDefault(answer{3}, gait.dutyFactor), 0.05), 0.95);
gait.stepLength = scalarOrDefault(answer{4}, gait.stepLength);
gait.clearance = scalarOrDefault(answer{5}, gait.clearance);
gait.bodyHeight = scalarOrDefault(answer{6}, gait.bodyHeight);
gait.phaseOffsets = vectorOrDefault(answer{7}, 4, gait.phaseOffsets);
gait.description = answer{8};

app.gaits(idx) = gait;
set(app.handles.gaitPopup, 'String', {app.gaits.name}, 'Value', idx);
guidata(fig, app);
updatePreview(fig);
setStatus(fig, sprintf('Updated %s gait. Use Save Gait to store it.', gait.name));
end

function editRobotMotor(fig)
app = guidata(fig);
config = app.config;
motor = app.motor;
oldHeight = config.robot.bodyHeight;

defaults = {
    num2str(config.robot.bodyLength)
    num2str(config.robot.bodyWidth)
    num2str(config.robot.bodyHeight)
    num2str(config.robot.bodyMassKg)
    num2str(config.robot.payloadMassKg)
    num2str(config.links.L1)
    num2str(config.links.L2)
    num2str(config.links.m1)
    num2str(config.links.m2)
    num2str(motor.Kt_Nm_per_A)
    num2str(motor.Kv_RPM_per_V)
    num2str(motor.GearRatio)
    num2str(motor.GearEfficiency)
    num2str(motor.NoLoadCurrent_A)
    num2str(motor.MaxCurrent_A)
    num2str(motor.Voltage_V)
    num2str(config.simulation.dt)
    num2str(config.simulation.animateEveryNFrames)
    num2str(config.simulation.maxAnimationFps)
    };
prompt = {
    'Body length (m)'
    'Body width (m)'
    'Body height / neutral foot drop (m)'
    'Robot body mass carried by legs (kg)'
    'Payload mass (kg)'
    'Upper link L1 (m)'
    'Lower link L2 (m)'
    'Upper link mass m1 (kg)'
    'Lower link mass m2 (kg)'
    'Motor Kt (Nm/A)'
    'Motor Kv (RPM/V)'
    'Gear ratio'
    'Gear efficiency, 0 to 1'
    'No-load current (A)'
    'Max current (A)'
    'Supply voltage (V)'
    'Simulation timestep dt (s)'
    'Display every N simulation frames'
    'Max animation FPS'
    };
answer = settingsDialog(prompt, 'Edit robot and motor', defaults);
if isempty(answer)
    return;
end

config.robot.bodyLength = scalarOrDefault(answer{1}, config.robot.bodyLength);
config.robot.bodyWidth = scalarOrDefault(answer{2}, config.robot.bodyWidth);
config.robot.bodyHeight = scalarOrDefault(answer{3}, config.robot.bodyHeight);
config.robot.bodyMassKg = nonnegativeScalarOrDefault(answer{4}, config.robot.bodyMassKg);
config.robot.payloadMassKg = nonnegativeScalarOrDefault(answer{5}, config.robot.payloadMassKg);
config.links.L1 = scalarOrDefault(answer{6}, config.links.L1);
config.links.L2 = scalarOrDefault(answer{7}, config.links.L2);
config.links.m1 = scalarOrDefault(answer{8}, config.links.m1);
config.links.m2 = scalarOrDefault(answer{9}, config.links.m2);
config.links.lc1 = config.links.L1 / 2;
config.links.lc2 = config.links.L2 / 2;
config.links.I1 = config.links.m1 * config.links.L1^2 / 12;
config.links.I2 = config.links.m2 * config.links.L2^2 / 12;

motor.Kt_Nm_per_A = scalarOrDefault(answer{10}, motor.Kt_Nm_per_A);
motor.Kv_RPM_per_V = scalarOrDefault(answer{11}, motor.Kv_RPM_per_V);
motor.GearRatio = scalarOrDefault(answer{12}, motor.GearRatio);
motor.GearEfficiency = min(max(scalarOrDefault(answer{13}, motor.GearEfficiency), 0.01), 1);
motor.NoLoadCurrent_A = max(0, scalarOrDefault(answer{14}, motor.NoLoadCurrent_A));
motor.MaxCurrent_A = scalarOrDefault(answer{15}, motor.MaxCurrent_A);
motor.Voltage_V = scalarOrDefault(answer{16}, motor.Voltage_V);
config.simulation.dt = scalarOrDefault(answer{17}, config.simulation.dt);
config.simulation.animateEveryNFrames = max(1, round(scalarOrDefault(answer{18}, config.simulation.animateEveryNFrames)));
config.simulation.maxAnimationFps = scalarOrDefault(answer{19}, config.simulation.maxAnimationFps);
motor.RobotMass_kg = config.robot.bodyMassKg;
motor.PayloadMass_kg = config.robot.payloadMassKg;

config = refreshQuadrupedConfig(config);
for i = 1:numel(app.gaits)
    if abs(app.gaits(i).bodyHeight - oldHeight) < 1e-9
        app.gaits(i).bodyHeight = config.robot.bodyHeight;
    end
end

app.config = config;
app.motor = motor;
set(app.handles.motorText, 'String', motorLabel(motor, config));
resetMotorStatusLight(app.handles.motorStatusText);
guidata(fig, app);
updatePreview(fig);
setStatus(fig, 'Updated robot geometry and motor constants.');
end

function importMotor(fig)
app = guidata(fig);
[name, path] = uigetfile({'*.csv;*.xlsx;*.xls;*.txt', 'Motor tables (*.csv, *.xlsx, *.xls, *.txt)'}, ...
    'Import motor constants');
if isequal(name, 0)
    return;
end

motors = importMotorConstants(fullfile(path, name));
if isempty(motors)
    setStatus(fig, 'No motors found in the selected file.');
    return;
end

idx = 1;
if numel(motors) > 1
    [idx, ok] = listdlg('PromptString', 'Choose motor constants', ...
        'SelectionMode', 'single', 'ListString', {motors.name});
    if ~ok
        return;
    end
end

app.motor = motors(idx);
app.config = applyRobotPresetToConfig(app.config, app.motor);
set(app.handles.motorText, 'String', motorLabel(app.motor, app.config));
resetMotorStatusLight(app.handles.motorStatusText);
guidata(fig, app);
updatePreview(fig);
setStatus(fig, sprintf('Imported motor constants: %s.', app.motor.name));
end

function analyzePose(fig)
app = guidata(fig);
defaultSupportN = (app.config.robot.bodyMassKg + app.config.robot.payloadMassKg) * ...
    app.config.simulation.gravity / 4;
defaults = {'-70', '-75', '0', '0', '0', '0', num2str(defaultSupportN)};
prompt = {
    'Hip angle q1 (deg, 0 = forward horizontal)'
    'Knee angle q2 (deg)'
    'Hip velocity (rad/s)'
    'Knee velocity (rad/s)'
    'Hip acceleration (rad/s^2)'
    'Knee acceleration (rad/s^2)'
    'Supported body load on this foot (N)'
    };
answer = settingsDialog(prompt, 'Specific orientation torque/current', defaults);
if isempty(answer)
    return;
end

qDeg = [signedScalarOrDefault(answer{1}, -70), signedScalarOrDefault(answer{2}, -75)];
qd = [signedScalarOrDefault(answer{3}, 0), signedScalarOrDefault(answer{4}, 0)];
qdd = [signedScalarOrDefault(answer{5}, 0), signedScalarOrDefault(answer{6}, 0)];
supportN = nonnegativeScalarOrDefault(answer{7}, defaultSupportN);
out = evaluate2DOFLegTorqueCurrent(qDeg * pi / 180, qd, qdd, app.config.links, ...
    app.motor, app.config.simulation.gravity, [0, -supportN]);

message = sprintf(['Pose q = [%.1f, %.1f] deg\n', ...
    'Body support load on foot: %.4f N\n', ...
    'Hip torque: %.4f Nm | current: %.4f A | voltage est.: %.4f V\n', ...
    'Knee torque: %.4f Nm | current: %.4f A | voltage est.: %.4f V'], ...
    qDeg(1), qDeg(2), supportN, out.torqueNm(1), out.currentA(1), out.voltageEstimateV(1), ...
    out.torqueNm(2), out.currentA(2), out.voltageEstimateV(2));
msgbox(message, 'Pose result');

poseFig = figure('Name', 'Pose Torque and Current', 'NumberTitle', 'off', 'Color', [1, 1, 1]);
subplot(1, 2, 1, 'Parent', poseFig);
bar(1:2, out.torqueNm);
set(gca, 'XTick', 1:2, 'XTickLabel', {'Hip', 'Knee'});
ylabel('Torque (Nm)');
title('Specific Pose Torque');
grid on;
subplot(1, 2, 2, 'Parent', poseFig);
bar(1:2, out.currentA);
set(gca, 'XTick', 1:2, 'XTickLabel', {'Hip', 'Knee'});
ylabel('Current (A)');
title('Specific Pose Current');
grid on;
end

function editBatteryEstimator(fig)
app = guidata(fig);
battery = app.battery;

defaults = {
    num2str(battery.nominalVoltageV)
    num2str(battery.capacityAh)
    num2str(battery.usableCapacityPct)
    num2str(battery.auxiliaryDrawW)
    num2str(battery.auxiliaryDrawA)
    num2str(battery.currentSafetyFactor)
    num2str(battery.internalResistanceOhm)
    num2str(battery.cutoffVoltageV)
    };
prompt = {
    'Battery nominal voltage (V)'
    'Battery capacity (Ah)'
    'Usable capacity (%)'
    'Logic/auxiliary draw (W)'
    'Additional auxiliary draw (A)'
    'Current safety/pessimism multiplier'
    'Pack internal resistance (Ohm, optional)'
    'Cutoff voltage (V, 0 disables cutoff warning)'
    };
answer = settingsDialog(prompt, 'Battery runtime estimator', defaults);
if isempty(answer)
    return;
end

battery.nominalVoltageV = scalarOrDefault(answer{1}, battery.nominalVoltageV);
battery.capacityAh = scalarOrDefault(answer{2}, battery.capacityAh);
battery.usableCapacityPct = min(100, max(1, scalarOrDefault(answer{3}, battery.usableCapacityPct)));
battery.auxiliaryDrawW = nonnegativeScalarOrDefault(answer{4}, battery.auxiliaryDrawW);
battery.auxiliaryDrawA = nonnegativeScalarOrDefault(answer{5}, battery.auxiliaryDrawA);
battery.currentSafetyFactor = scalarOrDefault(answer{6}, battery.currentSafetyFactor);
battery.internalResistanceOhm = nonnegativeScalarOrDefault(answer{7}, battery.internalResistanceOhm);
battery.cutoffVoltageV = nonnegativeScalarOrDefault(answer{8}, battery.cutoffVoltageV);

app.config.battery = battery;
app.config = refreshQuadrupedConfig(app.config);
app.battery = app.config.battery;

if isempty(app.results)
    app.batteryEstimate = [];
    set(app.handles.batteryText, 'String', batteryLabel([], app.battery));
    guidata(fig, app);
    setStatus(fig, 'Battery settings updated. Run a gait simulation to estimate runtime from the current profile.');
    return;
end

app.batteryEstimate = estimateBatteryRuntime(app.results, app.battery);
set(app.handles.batteryText, 'String', batteryLabel(app.batteryEstimate, app.battery));
guidata(fig, app);
showBatteryEstimatePlot(app.batteryEstimate);
setStatus(fig, batteryStatus(app.batteryEstimate));
end

function showBatteryEstimatePlot(estimate)
fig = figure('Name', 'Battery Runtime Estimate', 'NumberTitle', 'off', ...
    'Color', [0.98, 0.99, 1.00], 'Units', 'normalized', ...
    'Position', [0.16, 0.14, 0.62, 0.66]);
timeMinutes = estimate.time / 60;

subplot(2, 1, 1, 'Parent', fig);
plot(timeMinutes, estimate.inputCurrentA, 'Color', [0.00, 0.30, 0.95], 'LineWidth', 1.8);
hold on;
plot(timeMinutes, estimate.correctedMotorCurrentA, '--', 'Color', [0.90, 0.27, 0.00], 'LineWidth', 1.3);
grid on;
xlabel('Time (min)');
ylabel('Current (A)');
title('Battery Input Current Estimate');
legend({'Motor + aux input estimate', 'Safety-factored motor estimate'}, 'Location', 'best');

subplot(2, 1, 2, 'Parent', fig);
yyaxis left;
plot(timeMinutes, estimate.remainingSocPct, 'Color', [0.00, 0.56, 0.22], 'LineWidth', 1.8);
ylabel('Remaining SOC (%)');
ylim([0, 105]);
yyaxis right;
if estimate.battery.internalResistanceOhm > 0
    plot(timeMinutes, estimate.loadedVoltageV, 'Color', [0.62, 0.12, 0.92], 'LineWidth', 1.4);
    ylabel('Loaded voltage estimate (V)');
else
    plot(timeMinutes, estimate.remainingAh, 'Color', [0.62, 0.12, 0.92], 'LineWidth', 1.4);
    ylabel('Remaining usable capacity (Ah)');
end
grid on;
xlabel('Time (min)');
title('Battery Remaining Over Simulated Gait Window');

statsText = sprintf(['Avg %.2f A | Peak %.2f A | P95 %.2f A | P99 %.2f A | Runtime %.1f min\n', ...
    'Usable %.2f Ah / %.1f Wh | Profile uses %.3f Ah / %.2f Wh | Aux %.2f A | Safety %.2fx\n', ...
    'First-pass torque-derived current estimate; calibrate later with real battery current/voltage logs.'], ...
    estimate.averageCurrentA, estimate.peakCurrentA, estimate.percentile95CurrentA, ...
    estimate.percentile99CurrentA, estimate.runtimeMinutes, estimate.usableAh, ...
    estimate.usableWh, estimate.usedAh(end), estimate.usedWh(end), ...
    estimate.auxiliaryCurrentA, estimate.battery.currentSafetyFactor);
annotation(fig, 'textbox', [0.09, 0.005, 0.86, 0.085], 'String', statsText, ...
    'FitBoxToText', 'off', 'EdgeColor', [0.72, 0.76, 0.80], ...
    'BackgroundColor', [1, 1, 1], 'Color', [0.07, 0.09, 0.12], ...
    'FontSize', 9);
end

function saveGait(fig)
app = guidata(fig);
gait = activeGait(app);
savedDir = fullfile(pwd, 'saved_gaits');
if ~exist(savedDir, 'dir')
    mkdir(savedDir);
end
defaultFile = fullfile(savedDir, [safeFileName(gait.name), '.mat']);
[name, path] = uiputfile({'*.mat', 'MATLAB gait file (*.mat)'; '*.json', 'JSON gait file (*.json)'}, ...
    'Save gait', defaultFile);
if isequal(name, 0)
    return;
end

fileName = fullfile(path, name);
[~, ~, ext] = fileparts(fileName);
if isempty(ext)
    fileName = [fileName, '.mat'];
    ext = '.mat';
end

config = app.config;
motor = app.motor;
results = app.results;
if strcmpi(ext, '.json')
    payload = struct('gait', gait, 'config', config, 'motor', motor);
    writeTextFile(fileName, jsonencode(payload));
else
    save(fileName, 'gait', 'config', 'motor', 'results');
end
setStatus(fig, sprintf('Saved gait to %s.', fileName));
end

function loadGait(fig)
app = guidata(fig);
[name, path] = uigetfile({'*.mat;*.json', 'Gait files (*.mat, *.json)'}, 'Load gait');
if isequal(name, 0)
    return;
end
fileName = fullfile(path, name);
[~, ~, ext] = fileparts(fileName);

loadedGaits = struct([]);
if strcmpi(ext, '.json')
    raw = jsondecode(fileread(fileName));
    if isfield(raw, 'gait')
        loadedGaits = raw.gait;
    elseif isfield(raw, 'gaits')
        loadedGaits = raw.gaits;
    else
        loadedGaits = raw;
    end
else
    data = load(fileName);
    if isfield(data, 'gait')
        loadedGaits = data.gait;
    elseif isfield(data, 'gaits')
        loadedGaits = data.gaits;
    end
    if isfield(data, 'motor')
        app.motor = data.motor;
        app.config = applyRobotPresetToConfig(app.config, app.motor);
        set(app.handles.motorText, 'String', motorLabel(app.motor, app.config));
        resetMotorStatusLight(app.handles.motorStatusText);
    end
end

if isempty(loadedGaits)
    setStatus(fig, 'No gait struct was found in that file.');
    return;
end

for i = 1:numel(loadedGaits)
    gait = normalizeLoadedGait(loadedGaits(i), app.config);
    app = appendOrReplaceGait(app, gait);
end
set(app.handles.gaitPopup, 'String', {app.gaits.name}, 'Value', numel(app.gaits));
guidata(fig, app);
updatePreview(fig);
setStatus(fig, sprintf('Loaded %d gait(s) from %s.', numel(loadedGaits), fileName));
end

function exportResults(fig)
app = guidata(fig);
if isempty(app.results)
    setStatus(fig, 'Run a simulation before exporting results.');
    return;
end
savedDir = fullfile(pwd, 'saved_gaits');
if ~exist(savedDir, 'dir')
    mkdir(savedDir);
end
defaultFile = fullfile(savedDir, [safeFileName(app.results.gait.name), '_results.mat']);
[name, path] = uiputfile({'*.mat', 'MATLAB results (*.mat)'}, 'Export simulation results', defaultFile);
if isequal(name, 0)
    return;
end
results = app.results;
battery = app.battery;
batteryEstimate = app.batteryEstimate;
save(fullfile(path, name), 'results', 'battery', 'batteryEstimate');
setStatus(fig, sprintf('Exported results to %s.', fullfile(path, name)));
end

function app = appendOrReplaceGait(app, gait)
names = {app.gaits.name};
idx = find(strcmpi(names, gait.name), 1, 'first');
if isempty(idx)
    app.gaits(end + 1) = gait;
else
    app.gaits(idx) = gait;
end
end

function gait = normalizeLoadedGait(gait, config)
defaults = defaultQuadrupedGaits(config);
default = defaults(1);
fields = fieldnames(default);
for i = 1:numel(fields)
    fieldName = fields{i};
    if ~isfield(gait, fieldName) || isempty(gait.(fieldName))
        gait.(fieldName) = default.(fieldName);
    end
end
gait.name = char(gait.name);
gait.phaseOffsets = reshape(gait.phaseOffsets, 1, 4);
end

function gait = activeGait(app)
idx = get(app.handles.gaitPopup, 'Value');
idx = min(max(idx, 1), numel(app.gaits));
gait = app.gaits(idx);
end

function config = applyRobotPresetToConfig(config, motor)
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

function updateMetricsPanel(fig, results, frameIndex)
if ~ishandle(fig)
    return;
end
app = guidata(fig);
if ~isfield(app, 'handles') || ~isfield(app.handles, 'metricsText') || ~ishandle(app.handles.metricsText)
    return;
end

frameIndex = min(max(frameIndex, 1), numel(results.time));
duration = max(results.time(end), 0);
summary = results.summary;
warningText = sprintf('IK %d | Joint %d | Amp %d', ...
    summary.unreachableSamples, summary.jointLimitSamples, summary.currentLimitSamples);

message = sprintf(['Gait      %s\n', ...
    'Time      %5.2f / %5.2f s\n', ...
    'Torque    now %6.3f | peak %6.3f Nm\n', ...
    'Current   now %6.3f | peak %6.3f A\n', ...
    'Load      %6.3f Nm | support %6.2f N\n', ...
    'Speed     joint %5.2f rad/s | foot %5.2f m/s\n', ...
    'Limits    %s'], ...
    char(results.gait.name), results.time(frameIndex), duration, ...
    results.frameMaxTorqueNm(frameIndex), results.peakTorqueNm(frameIndex), ...
    results.frameMaxCurrentA(frameIndex), results.peakCurrentA(frameIndex), ...
    results.frameMaxLoadTorqueNm(frameIndex), max(results.supportForceN(frameIndex, :)), ...
    results.peakJointSpeedRadSec(frameIndex), results.peakFootSpeedMps(frameIndex), ...
    warningText);
set(app.handles.metricsText, 'String', message);
end

function setStatus(fig, message)
if ~ishandle(fig)
    return;
end
app = guidata(fig);
if isfield(app, 'handles') && isfield(app.handles, 'statusText') && ishandle(app.handles.statusText)
    set(app.handles.statusText, 'String', message);
    drawnow;
end
end

function message = summarizeStatus(results)
summary = results.summary;
message = sprintf('%s: max hip/knee torque %.3f/%.3f Nm, max hip/knee current %.3f/%.3f A.', ...
    results.gait.name, summary.maxAbsTorqueNm(1), summary.maxAbsTorqueNm(2), ...
    summary.maxCurrentA(1), summary.maxCurrentA(2));
message = sprintf('%s Max joint speed %.2f rad/s, max foot speed %.2f m/s.', ...
    message, summary.maxAbsJointSpeedAllRadSec, summary.maxFootSpeedMps);
message = sprintf('%s Max body-load torque %.3f Nm, max support %.2f N/leg.', ...
    message, summary.maxAbsLoadTorqueAllNm, summary.maxSupportForceN);
warnings = {};
if summary.unreachableSamples > 0
    warnings{end + 1} = sprintf('%d unreachable IK samples', summary.unreachableSamples);
end
if summary.jointLimitSamples > 0
    warnings{end + 1} = sprintf('%d joint-limit samples', summary.jointLimitSamples);
end
if summary.currentLimitSamples > 0
    warnings{end + 1} = sprintf('%d current-limit samples', summary.currentLimitSamples);
end
if summary.voltageLimitSamples > 0
    warnings{end + 1} = sprintf('%d voltage-limit samples', summary.voltageLimitSamples);
end
if ~isempty(warnings)
    message = [message, ' Warnings: ', strjoin(warnings, '; '), '.'];
end
end

function capability = evaluateMotorCapability(results, motor)
summary = results.summary;
capability = struct();
capability.maxJointTorqueNm = summary.maxAbsTorqueAllNm;
if isfield(summary, 'maxMotorTorqueAllNm')
    capability.maxMotorTorqueNm = summary.maxMotorTorqueAllNm;
else
    capability.maxMotorTorqueNm = nan;
end
capability.maxCurrentA = summary.maxCurrentAllA;
capability.currentLimitA = getFiniteMotorField(motor, 'MaxCurrent_A', inf);
capability.noLoadCurrentA = getFiniteMotorField(motor, 'NoLoadCurrent_A', 0);
capability.KtNmPerA = getFiniteMotorField(motor, 'Kt_Nm_per_A', nan);
capability.gearRatio = getFiniteMotorField(motor, 'GearRatio', 1);
capability.gearEfficiency = min(max(getFiniteMotorField(motor, 'GearEfficiency', 1), 0.01), 1);

if isfinite(capability.currentLimitA) && isfinite(capability.KtNmPerA)
    torqueCurrentLimitA = max(0, capability.currentLimitA - capability.noLoadCurrentA);
    capability.motorTorqueLimitNm = torqueCurrentLimitA * capability.KtNmPerA;
    capability.jointTorqueLimitNm = capability.motorTorqueLimitNm * capability.gearRatio * capability.gearEfficiency;
else
    capability.motorTorqueLimitNm = inf;
    capability.jointTorqueLimitNm = inf;
end

capability.currentOK = ~isfinite(capability.currentLimitA) || ...
    capability.maxCurrentA <= capability.currentLimitA;
capability.torqueOK = ~isfinite(capability.jointTorqueLimitNm) || ...
    capability.maxJointTorqueNm <= capability.jointTorqueLimitNm;
capability.knownLimits = isfinite(capability.currentLimitA) && isfinite(capability.jointTorqueLimitNm);
capability.ok = capability.currentOK && capability.torqueOK;
end

function updateMotorStatusLight(statusHandle, capability)
if isempty(statusHandle) || ~ishandle(statusHandle)
    return;
end

if ~capability.knownLimits
    set(statusHandle, 'String', sprintf('UNKNOWN\nNo current or\nKt limit'), ...
        'BackgroundColor', [0.98, 0.86, 0.36], 'ForegroundColor', [0.12, 0.09, 0.02]);
    return;
end

if capability.ok
    statusText = sprintf('OK\nTq %.2f / %.2f Nm\nAmp %.1f / %.1f A', ...
        capability.maxJointTorqueNm, capability.jointTorqueLimitNm, ...
        capability.maxCurrentA, capability.currentLimitA);
    set(statusHandle, 'String', statusText, ...
        'BackgroundColor', [0.58, 0.88, 0.60], 'ForegroundColor', [0.03, 0.12, 0.04]);
else
    statusText = sprintf('LIMIT\nTq %.2f / %.2f Nm\nAmp %.1f / %.1f A', ...
        capability.maxJointTorqueNm, capability.jointTorqueLimitNm, ...
        capability.maxCurrentA, capability.currentLimitA);
    set(statusHandle, 'String', statusText, ...
        'BackgroundColor', [0.98, 0.48, 0.45], 'ForegroundColor', [0.18, 0.02, 0.02]);
end
end

function resetMotorStatusLight(statusHandle)
if isempty(statusHandle) || ~ishandle(statusHandle)
    return;
end
set(statusHandle, 'String', sprintf('CHECK\nRun gait'), ...
    'BackgroundColor', [0.86, 0.88, 0.90], 'ForegroundColor', [0.08, 0.10, 0.12]);
end

function value = getFiniteMotorField(motor, fieldName, defaultValue)
if isstruct(motor) && isfield(motor, fieldName) && isnumeric(motor.(fieldName)) && ...
        ~isempty(motor.(fieldName)) && isfinite(motor.(fieldName)(1))
    value = motor.(fieldName)(1);
else
    value = defaultValue;
end
end

function label = batteryLabel(estimate, battery)
if isempty(estimate)
    label = sprintf(['Battery settings\n', ...
        '%.1f V | %.2f Ah | %.0f%% usable\n', ...
        'Aux %.2f W + %.2f A | safety %.2fx'], ...
        battery.nominalVoltageV, battery.capacityAh, battery.usableCapacityPct, ...
        battery.auxiliaryDrawW, battery.auxiliaryDrawA, battery.currentSafetyFactor);
    return;
end

label = sprintf(['Battery estimate\n', ...
    'Avg %.2f A | peak %.2f A | P95 %.2f A\n', ...
    'P99 %.2f A | runtime %.1f min'], ...
    estimate.averageCurrentA, estimate.peakCurrentA, estimate.percentile95CurrentA, ...
    estimate.percentile99CurrentA, estimate.runtimeMinutes);
end

function message = batteryStatus(estimate)
message = sprintf(['Battery estimate: avg %.2f A, peak %.2f A, P95 %.2f A, P99 %.2f A, ', ...
    'runtime %.1f min, usable %.2f Ah / %.1f Wh. First-pass estimate; calibrate with battery logs.'], ...
    estimate.averageCurrentA, estimate.peakCurrentA, estimate.percentile95CurrentA, ...
    estimate.percentile99CurrentA, estimate.runtimeMinutes, estimate.usableAh, estimate.usableWh);
if isfinite(estimate.cutoffTimeMinutes)
    message = sprintf('%s Cutoff voltage reached at %.2f min in this simulated window.', ...
        message, estimate.cutoffTimeMinutes);
end
end

function label = motorLabel(motor, config)
label = sprintf(['Motor: %s\n', ...
    'Kt %.4g Nm/A | Kv %.4g RPM/V | max %.3g A\n', ...
    'Gear %.3g:1 | eta %.2f'], ...
    motor.name, motor.Kt_Nm_per_A, motor.Kv_RPM_per_V, ...
    motor.MaxCurrent_A, motor.GearRatio, motor.GearEfficiency);
robotMassKg = nan;
payloadMassKg = nan;
if isfield(motor, 'RobotMass_kg') && isnumeric(motor.RobotMass_kg) && isfinite(motor.RobotMass_kg)
    robotMassKg = motor.RobotMass_kg;
elseif nargin > 1 && isstruct(config) && isfield(config, 'robot') && isfield(config.robot, 'bodyMassKg')
    robotMassKg = config.robot.bodyMassKg;
end
if isfield(motor, 'PayloadMass_kg') && isnumeric(motor.PayloadMass_kg) && isfinite(motor.PayloadMass_kg) && motor.PayloadMass_kg > 0
    payloadMassKg = motor.PayloadMass_kg;
elseif nargin > 1 && isstruct(config) && isfield(config, 'robot') && isfield(config.robot, 'payloadMassKg')
    payloadMassKg = config.robot.payloadMassKg;
end
if isfinite(robotMassKg)
    label = sprintf('%s | Robot %.3g kg', label, robotMassKg);
end
if isfinite(payloadMassKg) && payloadMassKg > 0
    label = sprintf('%s + %.3g kg payload', label, payloadMassKg);
end
end

function value = scalarOrDefault(textValue, defaultValue)
value = str2double(textValue);
if ~isfinite(value) || value <= 0
    value = defaultValue;
end
end

function value = signedScalarOrDefault(textValue, defaultValue)
value = str2double(textValue);
if ~isfinite(value)
    value = defaultValue;
end
end

function value = nonnegativeScalarOrDefault(textValue, defaultValue)
value = str2double(textValue);
if ~isfinite(value) || value < 0
    value = defaultValue;
end
end

function values = vectorOrDefault(textValue, count, defaultValues)
clean = strrep(textValue, '[', ' ');
clean = strrep(clean, ']', ' ');
clean = strrep(clean, ',', ' ');
values = sscanf(clean, '%f').';
if numel(values) ~= count || any(~isfinite(values))
    values = defaultValues;
end
values = mod(values, 1);
end

function answer = settingsDialog(prompt, dialogTitle, defaults)
prompt = prompt(:);
defaults = defaults(:);
count = numel(prompt);
answer = [];
if numel(defaults) ~= count
    defaults = repmat({''}, count, 1);
end

screen = get(0, 'ScreenSize');
figWidth = min(700, max(560, screen(3) - 120));
buttonAreaHeight = 58;
topPadding = 28;
rowHeight = 36;
contentHeight = topPadding + count * rowHeight + 12;
figHeight = min(max(330, contentHeight + buttonAreaHeight + 26), screen(4) - 140);
figHeight = max(figHeight, 300);
figLeft = screen(1) + (screen(3) - figWidth) / 2;
figBottom = screen(2) + (screen(4) - figHeight) / 2;

dialogFig = figure('Name', dialogTitle, 'NumberTitle', 'off', ...
    'MenuBar', 'none', 'ToolBar', 'none', 'Resize', 'on', ...
    'WindowStyle', 'modal', 'Units', 'pixels', ...
    'Position', [figLeft, figBottom, figWidth, figHeight], ...
    'Color', [0.94, 0.96, 0.98], 'CloseRequestFcn', @cancelCallback, ...
    'ResizeFcn', @resizeCallback);

margin = 18;
sliderWidth = 20;
viewPanel = uipanel('Parent', dialogFig, 'Units', 'pixels', ...
    'Position', [margin, buttonAreaHeight + 12, 10, 10], ...
    'BorderType', 'none', 'BackgroundColor', [0.94, 0.96, 0.98]);

contentPanel = uipanel('Parent', viewPanel, 'Units', 'pixels', ...
    'Position', [0, 0, 10, contentHeight], ...
    'BorderType', 'none', 'BackgroundColor', [0.94, 0.96, 0.98]);

labelHandles = gobjects(count, 1);
editHandles = gobjects(count, 1);

for i = 1:count
    y = contentHeight - topPadding - i * rowHeight + 7;
    labelHandles(i) = uicontrol(contentPanel, 'Style', 'text', 'String', prompt{i}, ...
        'Units', 'pixels', 'Position', [8, y + 2, 10, 22], ...
        'BackgroundColor', [0.94, 0.96, 0.98], ...
        'ForegroundColor', [0.08, 0.12, 0.16], ...
        'HorizontalAlignment', 'left', 'FontSize', 10);
    editHandles(i) = uicontrol(contentPanel, 'Style', 'edit', 'String', defaults{i}, ...
        'Units', 'pixels', 'Position', [10, y, 10, 26], ...
        'BackgroundColor', [1, 1, 1], 'ForegroundColor', [0.05, 0.07, 0.10], ...
        'HorizontalAlignment', 'left', 'FontSize', 10);
end

scrollBar = uicontrol(dialogFig, 'Style', 'slider', 'Units', 'pixels', ...
    'Position', [10, 10, sliderWidth, 10], 'Min', 0, 'Max', 1, ...
    'Value', 0, 'Visible', 'off', 'Callback', @scrollCallback);

okButton = uicontrol(dialogFig, 'Style', 'pushbutton', 'String', 'OK', ...
    'Units', 'pixels', 'Position', [10, 14, 88, 32], ...
    'FontSize', 10, 'FontWeight', 'bold', 'Callback', @okCallback);
cancelButton = uicontrol(dialogFig, 'Style', 'pushbutton', 'String', 'Cancel', ...
    'Units', 'pixels', 'Position', [108, 14, 88, 32], ...
    'FontSize', 10, 'Callback', @cancelCallback);

resizeCallback([], []);
if count > 0
    uicontrol(editHandles(1));
end
uiwait(dialogFig);
if ishandle(dialogFig)
    delete(dialogFig);
end

    function scrollCallback(src, ~)
        resizeCallback(src, []);
    end

    function resizeCallback(~, ~)
        if ~ishandle(dialogFig)
            return;
        end
        figPosition = get(dialogFig, 'Position');
        currentWidth = max(420, figPosition(3));
        currentHeight = max(260, figPosition(4));
        if currentWidth ~= figPosition(3) || currentHeight ~= figPosition(4)
            figPosition(3:4) = [currentWidth, currentHeight];
            set(dialogFig, 'Position', figPosition);
        end

        viewHeight = max(120, currentHeight - buttonAreaHeight - 24);
        needsScroll = contentHeight > viewHeight;
        scrollSpace = (sliderWidth + 6) * needsScroll;
        viewWidth = max(260, currentWidth - 2 * margin - 10 - scrollSpace);
        set(viewPanel, 'Position', [margin, buttonAreaHeight + 12, viewWidth, viewHeight]);

        maxScroll = max(0, contentHeight - viewHeight);
        if needsScroll
            scrollValue = min(get(scrollBar, 'Value'), maxScroll);
            set(scrollBar, 'Visible', 'on', 'Min', 0, 'Max', maxScroll, ...
                'Value', scrollValue, ...
                'Position', [margin + viewWidth + 6, buttonAreaHeight + 12, sliderWidth, viewHeight]);
            if maxScroll > 0
                set(scrollBar, 'SliderStep', [min(1, rowHeight / maxScroll), min(1, 5 * rowHeight / maxScroll)]);
            end
            contentY = viewHeight - contentHeight + scrollValue;
        else
            set(scrollBar, 'Visible', 'off', 'Min', 0, 'Max', 1, 'Value', 0);
            contentY = viewHeight - contentHeight;
        end

        set(contentPanel, 'Position', [0, contentY, viewWidth, contentHeight]);
        labelWidth = min(340, round(viewWidth * 0.45));
        editLeft = labelWidth + 18;
        editWidth = max(120, viewWidth - editLeft - 8);
        for k = 1:count
            y = contentHeight - topPadding - k * rowHeight + 7;
            set(labelHandles(k), 'Position', [8, y + 2, labelWidth, 22]);
            set(editHandles(k), 'Position', [editLeft, y, editWidth, 26]);
        end

        buttonY = 14;
        set(okButton, 'Position', [max(margin, currentWidth - 210), buttonY, 88, 32]);
        set(cancelButton, 'Position', [max(margin + 98, currentWidth - 112), buttonY, 88, 32]);
    end

    function okCallback(~, ~)
        answer = cell(count, 1);
        for k = 1:count
            answer{k} = get(editHandles(k), 'String');
        end
        uiresume(dialogFig);
    end

    function cancelCallback(~, ~)
        answer = [];
        uiresume(dialogFig);
    end
end

function fileName = safeFileName(textValue)
fileName = regexprep(char(textValue), '[^a-zA-Z0-9_\-]+', '_');
if isempty(fileName)
    fileName = 'gait';
end
end

function writeTextFile(fileName, textValue)
fid = fopen(fileName, 'w');
if fid < 0
    error('Could not open %s for writing.', fileName);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', textValue);
clear cleanup;
end
