%% Multi Agent Relief Support
restoredefaultpath; close all; clear; clc;
run('setup.m'); % COMMENT WHEN SUBMITTING TO ROBOTARIUM

%% Choose Parameters

% Agent
numAgent = 6; % Number of Agents
A = ones(1, numAgent)'*ones(1, numAgent) - eye(numAgent); % Adjacency Matrix of Graph (info to x info from)
agentMetricVisibleApothem = 0.1; % Metric Radius of Visible Map of Agent

% Simulation
contourRes = 5; % Contour Resolution
iteration = 500; % Total Number of Iterations

% Map
numSink = 6; % Number of Sinks
sinkMetricLen = 0.4; % Metric Length of Square Sink
sinkIdxLen = 11; % Index Length of Square Sink
sinkDepth = 1.5; % Depth of Sink

%% Generate Parameters

fieldDim = ARobotarium.boundaries; % Field Dimensions (xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax)
[map, xMapMetricGrid, yMapMetricGrid, metricToIdx] = generateMap(fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4), numSink, sinkMetricLen, sinkIdxLen, sinkDepth); % Map (y, x)
agentMetricPos0 = generateInitialPoses(numAgent, fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4)); % Agents' Initial Pose (x, y, theta)
agentState = zeros(1, numAgent); % Agents' State
agentIdxVisibleApothem = floor(agentMetricVisibleApothem.*metricToIdx) + 1; % Index Radius of Visible Map of Agent

%% Run Driver with Robotarium

roboDrv = Robotarium('NumberOfRobots', numAgent, 'InitialConditions', agentMetricPos0);
agentMetricPos = roboDrv.get_poses();
roboDrv.step(); % Set agents' initial pose
siToUni = create_si_to_uni_dynamics();
uniClamp = create_uni_barrier_certificate_with_boundary();

contour(xMapMetricGrid, yMapMetricGrid, map, contourRes); % Plot contour of map

for k = 1:iteration
    agentMetricPos = roboDrv.get_poses();
    agentMetricPosi = agentMetricPos(1:2, :); % Get agents' single integrator position
    agentMetricVeli = zeros(2, numAgent); % Initialize agents' single integrator velocities

    for agentN = 1:numAgent
        agentNMetricPosi = agentMetricPosi(1:2, agentN);
        agentAdjacentN = find(A(:, agentN) == 1); % Get set of agents adjacent to agent N
        visibleMapN = readSensorSim(agentNMetricPosi, agentIdxVisibleApothem, map, fieldDim(1), fieldDim(3), metricToIdx); % Extract visible disk of agent
        [agentMetricVelNi, agentNState] = searchRescueController(agentN, agentAdjacentN, visibleMapN, agentMetricPosi, agentState); % Execute controller
        agentMetricVeli(:, agentN) = agentMetricVelNi;
        agentState(agentN) = agentNState;
    end

    dxu = siToUni(agentMetricVeli, agentMetricPos); % Convert single integrator to unicycle
    dxu = uniClamp(dxu, agentMetricPos); % Impose inter-agent barrier and field boundary
    roboDrv.set_velocities(1:numAgent, dxu);
    roboDrv.step();
end

roboDrv.debug(); % Debug simulation; COMMENT WHEN SUBMITTING TO ROBOTARIUM