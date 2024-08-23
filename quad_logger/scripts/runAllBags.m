function runAllBags()
    % goals = [17.0506 16.8843 16.8658 16.4947 16.8459 16.865 2.19645 16.8833 16.7825 20 20 20 17.0966 16.7627 16.7242 16.6407 16.6269 16.3174 3.57962 16.3663 20 16.7626 20 20 20 17.1304 16.3002 16.2388 16.7385 16.8261 3.91154 17.0326 16.8627 20 16.9383 20];
    goals = [3.25 4.55 4.55];
    filenames = {'robot_1_quad_log_2024-08-14-17-16-49'
'robot_1_quad_log_2024-08-14-17-21-24'
'robot_1_quad_log_2024-08-14-17-24-25'};
%     filenames = {'robot_1_quad_log_2024-08-14-07-57-59'
% 'robot_1_quad_log_2024-08-14-08-03-40'
% 'robot_1_quad_log_2024-08-14-08-06-33'
% 'robot_1_quad_log_2024-08-14-08-11-26'
% 'robot_1_quad_log_2024-08-14-08-14-34'
% 'robot_1_quad_log_2024-08-14-08-18-00'
% 'robot_1_quad_log_2024-08-14-08-21-40'
% 'robot_1_quad_log_2024-08-14-08-23-26'
% 'robot_1_quad_log_2024-08-14-08-26-34'
% 'robot_1_quad_log_2024-08-14-08-29-38'
% 'robot_1_quad_log_2024-08-14-08-32-38'
% 'robot_1_quad_log_2024-08-14-08-35-29'
% 'robot_1_quad_log_2024-08-14-08-38-18'
% 'robot_1_quad_log_2024-08-14-08-40-43'
% 'robot_1_quad_log_2024-08-14-08-43-16'
% 'robot_1_quad_log_2024-08-14-08-45-37'
% 'robot_1_quad_log_2024-08-14-08-48-15'
% 'robot_1_quad_log_2024-08-14-08-50-58'
% 'robot_1_quad_log_2024-08-14-08-53-24'
% 'robot_1_quad_log_2024-08-14-08-55-42'
% 'robot_1_quad_log_2024-08-14-08-58-15'
% 'robot_1_quad_log_2024-08-14-09-00-57'
% 'robot_1_quad_log_2024-08-14-09-03-38'
% 'robot_1_quad_log_2024-08-14-09-06-02'
% 'robot_1_quad_log_2024-08-14-09-08-51'
% 'robot_1_quad_log_2024-08-14-09-11-12'
% 'robot_1_quad_log_2024-08-14-09-13-36'
% 'robot_1_quad_log_2024-08-14-09-15-55'
% 'robot_1_quad_log_2024-08-14-09-18-05'
% 'robot_1_quad_log_2024-08-14-09-20-11'
% 'robot_1_quad_log_2024-08-14-09-24-16'
% 'robot_1_quad_log_2024-08-14-09-27-04'
% 'robot_1_quad_log_2024-08-14-09-29-19'
% 'robot_1_quad_log_2024-08-14-09-31-37'
% 'robot_1_quad_log_2024-08-14-09-33-47'
% 'robot_1_quad_log_2024-08-14-09-35-34'};
    CoTs = zeros(1, length(goals));
    vs = zeros(1, length(goals));
    Ps = zeros(1, length(goals));
    for i = 1:length(goals)
        goal = goals(i);
        bagfile = filenames{i};
        namespace = 'robot_1';
        [CoT, v, P] = processLogBeam(bagfile, namespace, goal);
        
        % Store results in preallocated arrays
        CoTs(i) = CoT;
        vs(i) = v;
        Ps(i) = P;
    end
    disp('CoTs:');
    disp(CoTs);
    disp('Velocities:');
    disp(vs);
    disp('Powers:');
    disp(Ps);
end