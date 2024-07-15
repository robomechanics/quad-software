function processLogEKFDirectory(varargin)
% processLog Process a quad data log file to generate figures
%   processLog uses the default 'quad_log_current' file name to
%   yield a data structure containing select topic data. If this bag does
%   not exist, the user can select the bag via a UI.
%
%   processLog(FILENAME) uses the data in a bag with the specified
%   file name, looking in '../bags/'.

%% Prepare the environment
close all;clc;

bagsDir = '/home/rml/catkin_ws1/src/quad-sdk/quad_logger/bags/P';
scriptsDir = '/home/rml/catkin_ws1/src/quad-sdk/quad_logger/scripts';
plotsDir = fullfile(scriptsDir, 'plots');

% Create the plots directory if it doesn't exist
if ~exist(plotsDir, 'dir')
    mkdir(plotsDir);
end

% Change to the directory with the bag files
cd(bagsDir);

% Get a list of all .bag files in the current directory
bagFiles = dir('*.bag');

% Loop through each bag file
for k = 1:length(bagFiles)
    % Get the file name
    bagFileName = bagFiles(k).name;
    
    % Display the file being processed
    fprintf('Processing file: %s\n', bagFileName);

    % Construct the full path to the bag file
    bagFilePath = fullfile(bagsDir, bagFileName);

    % Change to the scripts directory to run processLog
    cd(scriptsDir);
    [~, trialName, ~] = fileparts(bagFileName);
    % Call the processLog function with the bag file path
    processLogDirect(trialName, 'robot_1', bagsDir, plotsDir); % Adjust namespace as needed

    % Move generated plots to the plots directory
    % movefile('../plots/*', plotsDir);

    % Change back to the bags directory
    % cd(bagsDir);
end