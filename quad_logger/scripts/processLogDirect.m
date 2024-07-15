function processLogDirect(trialName, namespace, bagsDir, plotsDir)
%   processLog uses the default 'quad_log_current' file name to
%   yield a data structure containing select topic data. If this bag does
%   not exist, the user can select the bag via a UI.
%
%   processLog(FILENAME) uses the data in a bag with the specified
%   file name, looking in '../bags/'.

%% Prepare the environment
close all;clc;

% Check that this is the right current directory otherwise paths won't work
% if ~endsWith(pwd, '/quad_logger/scripts')
%     error('This script must be run from quad_logger/scripts/');
% end

%% Select rosbag to parse

bSave = true;                       % Save the figures/videos
bAnimate = false;                   % Animate the trajectory (no translation)
bTitles = true;                     % Turn on figure titles
bPlotLocalPlanInfo = true;          % Turn on to plot local plan information
tWindowStates = [];                 % Specify time window for state (use [] for no clipping)
tWindowControl = [];                % Specify time window for control (use [] for no clipping)
tWindowLocalPlan = [];              % Specify time window for local plan (use [] for no clipping)

%% Create a directory for saving plots
if bSave
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    plotDir = fullfile(bagsDir, '..', 'plots', [trialName '_' timestamp]);
    if ~exist(plotDir, 'dir')
        mkdir(plotDir);
    end
end

%% Load the data

% Load the data
[data, trialName] = parseQuadBagDirect(trialName, namespace, bagsDir);
stateEstimate = data.stateEstimate;
stateGroundTruth = data.stateGroundTruth;

figure(1)
t = tiledlayout(3,1);
nexttile
hold on
plot(stateGroundTruth.time, stateGroundTruth.position(:,1), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.position(:,1), 'r-', 'LineWidth',2)
title("X position")
legend("Ground Truth", "State Estimate")
grid on
nexttile
hold on
plot(stateGroundTruth.time, stateGroundTruth.position(:,2), 'b', 'LineWidth',2 )
plot(stateEstimate.time, stateEstimate.position(:,2), 'r-' , 'LineWidth',2)
title("Y position")
legend("Ground Truth", "State Estimate")
grid on
nexttile
hold on
plot(stateGroundTruth.time, stateGroundTruth.position(:,3), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.position(:,3), 'r-','LineWidth',2)
title("Z position")
legend("Ground Truth", "State Estimate")
grid on
xlabel(t, "Time (s)")
ylabel(t, 'Body position (m/s)')

if bSave
    saveas(gcf, fullfile(plotDir, 'Position.png'));
end

figure(2)
t = tiledlayout(3,1);
nexttile
hold on
plot(stateGroundTruth.time, stateGroundTruth.velocity(:,1), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.velocity(:,1), 'r-', 'LineWidth',2)
xlim([1 12])
title("X Velocity")
legend("Ground Truth", "State Estimate")
grid on
nexttile
hold on
plot(stateGroundTruth.time, stateGroundTruth.velocity(:,2), 'b', 'LineWidth',2 )
plot(stateEstimate.time, stateEstimate.velocity(:,2), 'r-' , 'LineWidth',2)
xlim([1 12])
title("Y Velocity")
legend("Ground Truth", "State Estimate")
grid on
nexttile
hold on
plot(stateGroundTruth.time, stateGroundTruth.velocity(:,3), 'b-','LineWidth',2)
plot(stateEstimate.time, stateEstimate.velocity(:,3), 'r-','LineWidth',2)
xlim([1 12])
title("Z Velocity")
legend("Ground Truth", "State Estimate")
grid on
xlabel(t, "Time (s)")
ylabel(t, 'Body Velocity (m/s)')

if bSave
    saveas(gcf, fullfile(plotDir, 'Velocity.png'));
end


