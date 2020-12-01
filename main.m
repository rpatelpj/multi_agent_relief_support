%% Multi Agent Relief Support
restoredefaultpath; close all; clear; clc;
run('setup.m'); % COMMENT WHEN SUBMITTING TO ROBOTARIUM

%% Choose Parameters

% Agent
numAgent = 3; % Number of Agents
A = ones(1, numAgent)'*ones(1, numAgent) - eye(numAgent); % Adjacency Matrix of Graph (info to x info from)
agentMetricVisibleApothem = 0.1; % Metric Apothem of Visible Map of Agent

% Simulation
contourRes = 5; % Contour Resolution
iteration = 1000; % Total Number of Iterations

% Map
numSink = 5; % Number of Sinks
sinkMetricLen = 0.4; % Metric Length of Square Sink
sinkIdxLen = 11; % Index Length of Square Sink
sinkDepth = 1.5; % Depth of Sink

%% Generate Parameters

fieldDim = ARobotarium.boundaries; % Field Dimensions (xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax)
[map, xMapMetricGrid, yMapMetricGrid, metricToIdx] = generateMap(fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4), numSink, sinkMetricLen, sinkIdxLen, sinkDepth); % Map (y, x)
[agentMetricPos0, agentState] = generateInitialConditions(numAgent, fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4)); % Agents' Initial Conditions
agentIdxVisibleApothem = floor(agentMetricVisibleApothem.*metricToIdx) + 1; % Index Apothem of Visible Map of Agent

%% Run Driver with Robotarium

roboDrv = Robotarium('NumberOfRobots', numAgent, 'InitialConditions', agentMetricPos0);
agentMetricPosu = roboDrv.get_poses();
roboDrv.step(); % Set agents' initial pose
siToUni = create_si_to_uni_dynamics();
uniClamp = create_uni_barrier_certificate_with_boundary();

contour(xMapMetricGrid, yMapMetricGrid, map, contourRes); % Plot contour of map

for k = 1:iteration
    agentMetricPosu = roboDrv.get_poses();
    agentMetricPosi = agentMetricPosu(1:2, :); % Get agents' single integrator position
    agentMetricVeli = zeros(2, numAgent); % Initialize agents' single integrator velocities

    for agentN = 1:numAgent
        agentNMetricPosi = agentMetricPosi(1:2, agentN);
        agentNMetricPos0i = agentMetricPos0(1:2, agentN);
        agentAdjacentN = find(A(:, agentN) == 1); % Get set of agents adjacent to agent N
        visibleMapN = readSensorSim(agentNMetricPosi, agentIdxVisibleApothem, map, fieldDim(1), fieldDim(3), metricToIdx); % Extract visible disk of agent
        [agentNState, agentMetricVelNi] = searchRescueController(agentN, agentAdjacentN, visibleMapN, agentNMetricPos0i, agentState, agentMetricPosi, fieldDim(3), fieldDim(4), agentMetricVisibleApothem, sinkMetricLen); % Execute controller
        agentState(agentN) = agentNState;
        agentMetricVeli(:, agentN) = agentMetricVelNi;
    end

    agentMetricVelu = siToUni(agentMetricVeli, agentMetricPosu); % Convert single integrator to unicycle
    agentMetricVelu = uniClamp(agentMetricVelu, agentMetricPosu); % Impose inter-agent barrier and field boundary
    roboDrv.set_velocities(1:numAgent, agentMetricVelu);
    roboDrv.step();
end

roboDrv.debug(); % Debug simulation; COMMENT WHEN SUBMITTING TO ROBOTARIUM