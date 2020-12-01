%% Search and Rescue Controller
function [agentNState, agentMetricVelNi] = searchRescueController(agentN, agentAdjacentN, visibleMapN, agentNMetricPos0, agentState, agentMetricPos, yMapMetricMin, yMapMetricMax, agentMetricVisibleApothem, sinkMetricLen)
    % Calculate gradient of visible map of agent
    % TODO: Start with metric gradient, then convert to index gradient.
    visibleMapN = flipud(visibleMapN); % Transform visible map of agent to (-y, x) from (y, x)
    xGradVisibleMapN = -diff(visibleMapN, 1, 2);
    yGradVisibleMapN = -diff(visibleMapN, 1, 1);
    yGradVisibleMapN = flipud(yGradVisibleMapN); % Transform y gradient of visible map of agent to (y, x) from (-y, x)

    % State Machine
        % State 0.x: Search field
            % State 0.2: Move up
            % State 0.4: Move right
            % State 0.6: Move left
            % State 0.8: Move down
        % State 1: Descend sink

	% Is agent descending sink?
    if (agentState(agentN) == 1)
        agentNState = 1;
        agentMetricVelNi = [mean(xGradVisibleMapN(xGradVisibleMapN ~= 0), 'all');
                            mean(yGradVisibleMapN(yGradVisibleMapN ~= 0), 'all')]; % Average sink only

    % If false, can agent see a sink?
    elseif any((visibleMapN ~= 0), 'all')
        agentState1Set = find(agentState == 1);
        sinkKnownSet = intersect(agentState1Set, agentAdjacentN);
        agentAdjacentNDist = vecnorm(agentMetricPos(:, sinkKnownSet) - agentMetricPos(:, agentN)); % Calculate distances between agent N and agents adjacent to agent N
        sinkMetricDiameter = sinkMetricLen.*sqrt(2);

        % If true, are there no agents already at a sink, or at least is there an agent not at this sink?
        if isempty(sinkKnownSet) || (min(agentAdjacentNDist) > sinkMetricDiameter)
            % TODO: Since sink is square, sink diameter is used for threshold distance.
            %       This results in a bug when there is an open sink positioned close to two sinks at greater than twice sink apothem, but less than sink diameter.
            %       Agent cannot reach this open sink.
            % TODO: Realistically, agent wouldn't know sinkMetricDiameter. Replace with another metric.
            agentNState = 1;
            agentMetricVelNi = [mean(xGradVisibleMapN(xGradVisibleMapN ~= 0), 'all');
                                mean(yGradVisibleMapN(yGradVisibleMapN ~= 0), 'all')]; % Average sink only

        % If false, keep moving.
        else
            [agentNState, agentMetricVelNi] = routingAlgorithm(agentN, agentNMetricPos0, agentMetricPos, agentState, yMapMetricMin, yMapMetricMax, agentMetricVisibleApothem);
        end

    % If false, keep moving.
    else
        [agentNState, agentMetricVelNi] = routingAlgorithm(agentN, agentNMetricPos0, agentMetricPos, agentState, yMapMetricMin, yMapMetricMax, agentMetricVisibleApothem);
    end
end

%% Routing Algorithm
function [agentNState, agentMetricVelNi] = routingAlgorithm(agentN, agentNMetricPos0, agentMetricPos, agentState, yMapMetricMin, yMapMetricMax, agentMetricVisibleApothem)
    % Local parameters
    speed = 0.25;
    boundaryTol = 0.2;
    yAgentMetricMin = yMapMetricMin + boundaryTol;
    yAgentMetricMax = yMapMetricMax - boundaryTol;

    % Move up
    if (agentState(agentN) == 0.2)
        if (agentMetricPos(2, agentN) < yAgentMetricMax)
            agentNState = 0.2;
            agentMetricVelNi = [0; speed];
        elseif (agentNMetricPos0(1) <= 0)
            agentNState = 0.4;
            agentMetricVelNi = [speed; 0];
        elseif (agentNMetricPos0(1) <= 0)
            agentNState = 0.6;
            agentMetricVelNi = [-speed; 0];
        else
            agentNState = 0.6;
            agentMetricVelNi = [-speed; 0];
        end

    % Move right
    elseif (agentState(agentN) == 0.4)
        if (mod(round(agentMetricPos(1, agentN), 2), agentMetricVisibleApothem) ~= 0)
            agentNState = 0.4;
            agentMetricVelNi = [speed; 0];
        elseif (agentMetricPos(2, agentN) >= yAgentMetricMax)
            agentNState = 0.8;
            agentMetricVelNi = [0; -speed];
        elseif (agentMetricPos(2, agentN) <= yAgentMetricMin)
            agentNState = 0.2;
            agentMetricVelNi = [0; speed];
        else
            agentNState = 0.8;
            agentMetricVelNi = [0; -speed];
        end

    % Move left
    elseif (agentState(agentN) == 0.6)
        if (mod(round(agentMetricPos(1, agentN), 2), agentMetricVisibleApothem) ~= 0)
            agentNState = 0.6;
            agentMetricVelNi = [-speed; 0];
        elseif (agentMetricPos(2, agentN) >= yAgentMetricMax)
            agentNState = 0.8;
            agentMetricVelNi = [0; -speed];
        elseif (agentMetricPos(2, agentN) <= yAgentMetricMin)
            agentNState = 0.2;
            agentMetricVelNi = [0; speed];
        else
            agentNState = 0.2;
            agentMetricVelNi = [0; speed];
        end

    % Move down
    elseif (agentState(agentN) == 0.8)
        if (agentMetricPos(2, agentN) > yAgentMetricMin)
            agentNState = 0.8;
            agentMetricVelNi = [0; -speed];
        elseif (agentNMetricPos0(1) <= 0)
            agentNState = 0.4;
            agentMetricVelNi = [speed; 0];
        elseif (agentNMetricPos0(1) <= 0)
            agentNState = 0.6;
            agentMetricVelNi = [-speed; 0];
        else
            agentNState = 0.4;
            agentMetricVelNi = [speed; 0];
        end
    end
end