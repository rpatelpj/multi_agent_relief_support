%% Multi Agent Relief Support
restoredefaultpath; close all; clear; clc; % COMMENT WHEN SUBMITTING TO ROBOTARIUM
run('setup.m'); % COMMENT WHEN SUBMITTING TO ROBOTARIUM

%% Choose Parameters

% Agent
numAgent = 5; % Number of Agents
A = ones(1, numAgent)'*ones(1, numAgent) - eye(numAgent); % Adjacency Matrix of Graph (info to x info from), Complete Graph
g = 'K'; % Graph Type Indicator, Complete Graph
% A = diag(ones(1, (numAgent - 1)), 1) + diag(ones(1, (numAgent - 1)), -1); % Adjacency Matrix of Graph (info to x info from), Cycle Graph
% A(numAgent, 1) = 1; % Adjacency Matrix of Graph (info to x info from), Cycle Graph
% A(1, numAgent) = 1; % Adjacency Matrix of Graph (info to x info from), Cycle Graph
% g = 'C'; % Graph Type Indicator, Complete Graph
agentMetricSensingRadius = 0.1; % Metric Agent Sensing Radius

% Sink
numSink = 5; % Number of Sinks
sinkMetricLen = 0.3; % Metric Length of Square Sink
sinkIdxLen = 11; % Index Length of Square Sink
sinkDepth = 1.5; % Depth of Sink

% Simulation
contourRes = 5; % Contour Resolution
iteration = 1500; % Total Number of Iterations

% Functionality
rng('shuffle'); % Use RNG based on current system time
export = true;

%% Robotarium Parameters

fieldDim = [-1.6, 1.6, -1, 1]; % Field Dimensions (xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax), ARobotarium.boundaries
agentSize = 0.11; % Size of Agent, ARobotarium.robot_diameter
agentAntiCollisionTol = 0.02; % Minimum Distance between Agents
sinkMetricLen = max(sinkMetricLen, agentSize + agentAntiCollisionTol); % Avoid multiple sinks within agent size

%% Generate Parameters

[map, xMapMetricGrid, yMapMetricGrid, metricToIdx] = generateMap(fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4), numSink, sinkMetricLen, sinkIdxLen, sinkDepth); % Map (y, x)
[agentMetricPos0, agentState] = generateInitialConditions(numAgent, fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4)); % Agents' Initial Conditions
agentIdxSensingRadius = floor(agentMetricSensingRadius.*metricToIdx) + 1; % Index Apothem of Visible Map of Agent

%% Run Driver with Robotarium

% Initialize Robotarium
agentDriver = Robotarium('NumberOfRobots', numAgent, 'InitialConditions', agentMetricPos0);
agentMetricPosu = agentDriver.get_poses();
agentDriver.step(); % Set agents' initial pose
siToUni = create_si_to_uni_dynamics();
uniClamp = create_uni_barrier_certificate_with_boundary();

% Plot contour of map
contour(xMapMetricGrid, yMapMetricGrid, map, contourRes);

% Initialize plot for visible map of agents
agentMetricPosi = agentMetricPosu(1:2, :);
handle = scatter(agentMetricPosi(1, :), agentMetricPosi(2, :), 'filled');
currentunits = get(gca,'Units');
set(gca, 'Units', 'Points');
axpos = get(gca,'Position');
set(gca, 'Units', currentunits);
markerWidth = axpos(3).*(2.*agentMetricSensingRadius)./diff(xlim); % Calculate marker width in points
handle.SizeData = markerWidth.^2;
handle.MarkerFaceColor = [0.12, 0.49, 0.65];
handle.MarkerFaceAlpha = 0.35;

% Initialize video and export image of initial agent poses
if (export)
    vid = VideoWriter(['videos/sar+a_' num2str(numAgent) '+s_' num2str(numSink) '+amsr_' num2str(agentMetricSensingRadius) '+sml_' num2str(sinkMetricLen) '+g_' g '.mp4'], 'MPEG-4');
    vid.Quality = 100;
    vid.FrameRate = 60;
    open(vid);
    writeVideo(vid, getframe(gcf));

    exportgraphics(agentDriver.figure_handle, ['images/sar+t_0+a_' num2str(numAgent) '+s_' num2str(numSink) '+amsr_' num2str(agentMetricSensingRadius) '+sml_' num2str(sinkMetricLen) '+g_' g '.png'], 'Resolution', 500);
end

for k = 1:iteration
    agentMetricPosu = agentDriver.get_poses();
    agentMetricPosi = agentMetricPosu(1:2, :); % Get agents' single integrator position
    agentMetricVeli = zeros(2, numAgent); % Initialize agents' single integrator velocities

    set(handle, 'XData', agentMetricPosi(1, :), 'YData', agentMetricPosi(2, :)); % Load agent positions for visible map plots

    for agentNIdx = 1:numAgent
        agentNMetricPosi = agentMetricPosi(1:2, agentNIdx);
        agentAdjacentNIdx = find(A(:, agentNIdx) == 1); % Get set of agents adjacent to agent N
        visibleMapN = readSensorSim(agentNMetricPosi, agentIdxSensingRadius, map, fieldDim(1), fieldDim(3), metricToIdx); % Extract visible disk of agent
        [agentNState, agentNMetricVeli] = searchRescueController(agentMetricPosi(:, agentNIdx), agentState(:, agentNIdx), visibleMapN, agentMetricPosi(:, agentAdjacentNIdx), fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4), metricToIdx, agentMetricSensingRadius, sinkMetricLen); % Execute controller
        agentState(agentNIdx) = agentNState;
        agentMetricVeli(:, agentNIdx) = agentNMetricVeli;
    end

    agentMetricVelu = siToUni(agentMetricVeli, agentMetricPosu); % Convert single integrator to unicycle
    agentMetricVelu = uniClamp(agentMetricVelu, agentMetricPosu); % Impose inter-agent barrier and field boundary
    agentDriver.set_velocities(1:numAgent, agentMetricVelu);
    agentDriver.step();
    drawnow; % Update visible map of agents
    if (export)
        writeVideo(vid, getframe(agentDriver.figure_handle)); % Add frame to video
    end
end

% Finalize video and export image of final agent poses
if (export)
    close(vid);

    exportgraphics(agentDriver.figure_handle, ['images/sar+t_f+a_' num2str(numAgent) '+s_' num2str(numSink) '+amsr_' num2str(agentMetricSensingRadius) '+sml_' num2str(sinkMetricLen) '+g_' g '.png'], 'Resolution', 500);
end

agentDriver.debug(); % Debug simulation; COMMENT WHEN SUBMITTING TO ROBOTARIUM