%% Multi Agent Relief Support
restoredefaultpath; close all; clear; clc;
run('setup.m'); % COMMENT WHEN SUBMITTING TO ROBOTARIUM

%% Choose Parameters

% Agent
numAgent = 3; % Number of Agents
A = ones(1, numAgent)'*ones(1, numAgent) - eye(numAgent); % Adjacency Matrix of Graph
agentMetricRadius = 0.2; % Metric Radius of Visibility Disk of Agent

% Simulation
contourRes = 5; % Contour Resolution
iteration = 1000; % Total Number of Iterations

% Map
numSink = 3; % Number of Sinks
sinkMetricLen = 0.2; % Metric Length of Square Sink
sinkIdxLen = 9; % Index Length of Square Sink
sinkDepth = 1.5; % Depth of Sink

%% Generate Parameters

fieldSize = ARobotarium.boundaries;
[metricToIdx, xMapMetricGrid, yMapMetricGrid, map] = generateMap(fieldSize(1), fieldSize(2), fieldSize(3), fieldSize(4), numSink, sinkMetricLen, sinkIdxLen, sinkDepth); % Map
x0 = generateInitialConditions(numAgent, fieldSize(1), fieldSize(2), fieldSize(3), fieldSize(4)); % Agent Initial Conditions
agentIdxRadius = agentMetricRadius.*metricToIdx; % Index Radius of Visibility Disk of Agent

%% Run Driver with Robotarium

roboDrv = Robotarium('NumberOfRobots', numAgent, 'InitialConditions', x0);
x = roboDrv.get_poses();
roboDrv.step();
siToUni = create_si_to_uni_dynamics();
uniClamp = create_uni_barrier_certificate_with_boundary();

contour(xMapMetricGrid, yMapMetricGrid, map, contourRes); % Plot contour of map

for k = 1:iteration
    x = roboDrv.get_poses();
    xi = x(1:2, :); % Extract single integrator states

    dxi = searchRescueController(xi, map, agentIdxRadius); % Execute controller

    dxu = siToUni(dxi, x); % Convert single integrator states to unicycle states
    dxu = uniClamp(dxu, x); % Impose inter-agent barrier and field boundary
    roboDrv.set_velocities(1:numAgents, dxu);
    roboDrv.step();
end

roboDrv.debug(); % Debug simulation; COMMENT WHEN SUBMITTING TO ROBOTARIUM