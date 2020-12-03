%% Multi Agent Relief Support
restoredefaultpath; close all; clear; clc;
run('setup.m'); % COMMENT WHEN SUBMITTING TO ROBOTARIUM

%% Choose Parameters

% Agent
numAgent = 10; % Number of Agents
A = ones(1, numAgent)'*ones(1, numAgent) - eye(numAgent); % Adjacency Matrix of Graph (info to x info from)
agentMetricVisibilityApothem = 0.25; % Metric Apothem of Visible Map of Agent
gridSensor = false; % If true, use grid sensor. If false, use circular sensor.

% Simulation
contourRes = 5; % Contour Resolution
iteration = 2000; % Total Number of Iterations

% Map
numSink = 5; % Number of Sinks
sinkMetricLen = 0.2; % Metric Length of Square Sink
sinkIdxLen = 11; % Index Length of Square Sink
sinkDepth = 0.5; % Depth of Sink

%% Generate Parameters

fieldDim = [-1.6, 1.6, -1, 1]; % Field Dimensions (xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax)
[map, xMapMetricGrid, yMapMetricGrid, metricToIdx] = generateMap(fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4), numSink, sinkMetricLen, sinkIdxLen, sinkDepth); % Map (y, x)
[agentMetricPos0, agentState] = generateInitialConditions(numAgent, fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4)); % Agents' Initial Conditions
agentIdxVisibleApothem = floor(agentMetricVisibilityApothem.*metricToIdx) + 1; % Index Apothem of Visible Map of Agent
%%%%%% for debugging: 
% figure(5)
% symbol = ['m*', 'c*', 'k*', 'g*', 'b*', 'r*', 'md', 'cd', 'kd', 'gd'];
%% Run Driver with Robotarium

roboDrv = Robotarium('NumberOfRobots', numAgent, 'InitialConditions', agentMetricPos0);
agentMetricPosu = roboDrv.get_poses();
roboDrv.step(); % Set agents' initial pose
siToUni = create_si_to_uni_dynamics();
uniClamp = create_uni_barrier_certificate_with_boundary();

contour(xMapMetricGrid, yMapMetricGrid, map, contourRes); % Plot contour of map
agentMetricPosi = agentMetricPosu(1:2, :); % Get agents' single integrator position
handle = scatter(agentMetricPosi(1, :), agentMetricPosi(2, :), 'filled'); % Plot visible map of agents

currentunits = get(gca,'Units');
set(gca, 'Units', 'Points');
axpos = get(gca,'Position');
set(gca, 'Units', currentunits);
markerWidth = (2*agentMetricVisibilityApothem)/diff(xlim)*axpos(3); % Calculate Marker width in points

handle.SizeData = markerWidth^2;
handle.MarkerFaceColor = [0.12,0.49,0.65];
handle.MarkerFaceAlpha = 0.2;

for k = 1:iteration
    agentMetricPosu = roboDrv.get_poses();
    agentMetricPosi = agentMetricPosu(1:2, :); % Get agents' single integrator position
    agentMetricVeli = zeros(2, numAgent); % Initialize agents' single integrator velocities
    
    set(handle,'XData',agentMetricPosi(1, :),'YData',agentMetricPosi(2, :))

    for agentN = 1:numAgent
        agentNMetricPosi = agentMetricPosi(1:2, agentN);
        agentNMetricPos0i = agentMetricPos0(1:2, agentN);
        agentAdjacentN = find(A(:, agentN) == 1); % Get set of agents adjacent to agent N
        visibleMapN = readSensorSim(agentNMetricPosi, agentIdxVisibleApothem, map, fieldDim(1), fieldDim(3), metricToIdx, gridSensor); % Extract visible disk of agent
        [agentNState, agentNMetricVeli] = searchRescueController(agentN, agentAdjacentN, visibleMapN, agentNMetricPos0i, agentState, agentMetricPosi, fieldDim(3), fieldDim(4), agentMetricVisibilityApothem, sinkMetricLen); % Execute controller
        agentState(agentN) = agentNState;
        agentMetricVeli(:, agentN) = agentNMetricVeli;
%         plot(agentMetricVeli(1, agentN), agentMetricVeli(2, agentN), symbol(agentN))
    end
    
    agentMetricVelu = siToUni(agentMetricVeli, agentMetricPosu); % Convert single integrator to unicycle
    agentMetricVelu = uniClamp(agentMetricVelu, agentMetricPosu); % Impose inter-agent barrier and field boundary
    roboDrv.set_velocities(1:numAgent, agentMetricVelu);
    roboDrv.step();
    drawnow; % Update visible map of agents
end

roboDrv.debug(); % Debug simulation; COMMENT WHEN SUBMITTING TO ROBOTARIUM