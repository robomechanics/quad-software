function [COT v P] = processLogBeam(varargin)
% processLog Process a quad data log file to generate figures
%   processLog uses the default 'quad_log_current' file name to
%   yield a data structure containing select topic data. If this bag does
%   not exist, the user can select the bag via a UI.
%
%   processLog(FILENAME) uses the data in a bag with the specified
%   file name, looking in '../bags/'.

%% Prepare the environment
close all;clc;

% Check that this is the right current directory otherwise paths won't work
if ~endsWith(pwd, '/quad_logger/scripts')
    error('This script must be run from quad_logger/scripts/');
end

%% Select rosbag to parse

% If a trial name is provided, use that to save everything
if nargin>0
    trialName = varargin{1};
    namespace = varargin{2};
    goal = varargin{3};
else
    trialName = ''; % Set to '' to load via GUI
    namespace = 'robot_1'; % Namespace of the robot bag, set to '' if none
end

bagDir = fullfile('..', 'bags', 'BeamWalking2');
% Use the provided trial name to construct the bag file path
bagFilePath = fullfile(bagDir, [trialName, '.bag']);

% Now you can use `bagFilePath` to load the ROS bag file
disp(['Processing bag file: ', bagFilePath]);


%% Set parameters

bSave = false;                       % Save the figures/videos
bAnimate = false;                   % Animate the trajectory (no translation)
bTitles = true;                     % Turn on figure titles
bPlotLocalPlanInfo = true;          % Turn on to plot local plan information
tWindowStates = [];                 % Specify time window for state (use [] for no clipping)
tWindowControl = [];                % Specify time window for control (use [] for no clipping)
tWindowLocalPlan = [];              % Specify time window for local plan (use [] for no clipping)
isHardware = false;

%% Load the data
% Load the data
[data, trialName] = parseQuadBag(trialName, namespace);
stateGroundTruth = data.stateGroundTruth;

% Isolate Time and X Pos Vectors
t_state = data.stateGroundTruth.time;
x_state = data.stateGroundTruth.position(:,1);

x_origin = 0; % m
% x_fo = 1.1938; x_lo = x_fo + 0.762;  % m; dist to first, last obstacle (mocap frame)
% x_start = x_fo - 0.3; % Start 30 cm before first obstacle
if isHardware
    x_start = 0.65;
else
    x_start = 0;
end

x_end =goal; % End 1 m after last obstacle

%Solve for closest x_start and x_end
%Could Linearly Interpolate and Solve for Exact, but doing this for
%simplicity
ind_start = find(x_state > x_start,1,'first');
ind_end = find(x_state < x_end,1,'last');

if isempty(ind_end)
    % If file ends before designated last obstacle
    ind_end = length(x_state);
end

%Generate Energy Plots
ktau = [0.546, 0.546, 1.092];   % N m/A
kv = ktau;                      % V s/rad
R = 0.174;                      % Ohms

mot_I = data.stateGroundTruth.jointEffort./repmat(ktau,[1,4]);
mot_V = repmat(kv,[1,4]).*data.stateGroundTruth.jointVelocity + mot_I*R;
mot_P = mot_V.*mot_I;
mot_E = cumtrapz(t_state,sum(max(mot_P,0),2));

v = (x_end - x_start)/(t_state(ind_end)-t_state(ind_start));
P = (mot_E(ind_start)-mot_E(ind_end))/(t_state(ind_start)-t_state(ind_end));
COT = P/(13*9.81*v);

