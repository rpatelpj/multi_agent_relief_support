%% Search and Rescue Controller
function [agentNState, agentNMetricVel] = searchRescueController(agentNIdx, agentAdjacentNIdx, visibleMapN, agentState, agentMetricPos, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, metricToIdx, agentMetricVisibilityApothem, sinkMetricLen)
    % Transform visible map of agent to (-y, x) from (y, x)
    visibleMapN = flipud(visibleMapN);

    % State Machine
        % State 0.x: Search field
            % State 0.2: Move up
            % State 0.4: Move right
            % State 0.6: Move left
            % State 0.8: Move down
        % State 1: Descend sink

    % See a sink? If true, attempt to descend sink.
    if any((visibleMapN ~= 0), 'all')
        [agentNState, agentNMetricVel] = sinkDescentAlgorithm(agentNIdx, agentAdjacentNIdx, visibleMapN, agentMetricPos, xMapMetricMin, yMapMetricMin, metricToIdx, sinkMetricLen);

        % If no open sink, keep moving.
        if (agentNState == 0)
            [agentNState, agentNMetricVel] = routingAlgorithm(agentMetricPos(:, agentNIdx), agentState(:, agentNIdx), xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem);
        end

    % If no visible sink, keep moving.
    else
        [agentNState, agentNMetricVel] = routingAlgorithm(agentMetricPos(:, agentNIdx), agentState(:, agentNIdx), xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem);
    end
end

%% Sink Descent Algorithm
function [agentNState, agentNMetricVel] = sinkDescentAlgorithm(agentNIdx, agentAdjacentNIdx, visibleMapN, agentMetricPos, xMapMetricMin, yMapMetricMin, metricToIdx, sinkMetricLen)
    % Find lowest point of visible portion of sink(s) relative to visible map in index
	[sinkIdxMinVisibleMapN, ~] = min(visibleMapN, [], 'all', 'linear');
	sinkIdxMinVisibleMapNIdx = find(visibleMapN == sinkIdxMinVisibleMapN);
	[sinkIdxMinVisibleMapNRow, sinkIdxMinVisibleMapNCol] = ind2sub(size(visibleMapN), sinkIdxMinVisibleMapNIdx);

    % Find agent N position relative to visible map in index
    agentNIdxPosRelVisibleMapN = round(size(visibleMapN)./2);

    % Convert to lowest point of visible portion of sink(s) relative to visible map in metric
    idxToMetric = 1./metricToIdx;
    xSinkMetricMinRelVisibleMapN = (sinkIdxMinVisibleMapNCol - 1).*idxToMetric + xMapMetricMin; % Accuracy loss due to nonlinear transform
    ySinkMetricMinRelVisibleMapN = (sinkIdxMinVisibleMapNRow - 1).*idxToMetric + yMapMetricMin; % Accuracy loss due to nonlinear transform

    % Convert to agent N position relative to visible map in metric
    xAgentMetricPosRelVisibleMapN = (agentNIdxPosRelVisibleMapN(2) - 1)./metricToIdx + xMapMetricMin; % Accuracy loss due to nonlinear transform
    yAgentMetricPosRelVisibleMapN = (agentNIdxPosRelVisibleMapN(1) - 1)./metricToIdx + yMapMetricMin; % Accuracy loss due to nonlinear transform

    % Calculate vector(s) between lowest point of visible sink(s) and agent M
    agentNSinkMetricVec = [(xSinkMetricMinRelVisibleMapN' - xAgentMetricPosRelVisibleMapN);
                           (ySinkMetricMinRelVisibleMapN' - yAgentMetricPosRelVisibleMapN)];

    % Choose lowest point of visible sink closest to agent N
    [~, minAgentNSinkMetricVecIdx] = min(vecnorm(agentNSinkMetricVec));
    agentNSinkMetricVec = agentNSinkMetricVec(:, minAgentNSinkMetricVecIdx);

    % Find absolute sink position
    sinkMapMetricPos = agentNSinkMetricVec + agentMetricPos(:, agentNIdx);

    % Find minimum distance between sink and all agents adjacent to agent N and agent N
    agentSet = sort([agentNIdx; agentAdjacentNIdx]);
    agentSinkDist = vecnorm(sinkMapMetricPos - agentMetricPos);
    [minAgentAdjacentNSinkDist, ~] = min(agentSinkDist(:, agentSet));
    minAgentAdjacentNSinkDistIdx = find(agentSinkDist == minAgentAdjacentNSinkDist);

    % Is agent N closest to sink?
    if (minAgentAdjacentNSinkDistIdx == agentNIdx)
        % Find "agent(s) nearby agent N and adjacent to agent N"
        agentNAgentAdjacentNMetricDist = vecnorm(agentMetricPos(:, agentAdjacentNIdx) - agentMetricPos(:, agentNIdx));
        agentNearbyAdjacentNIdx = find(agentNAgentAdjacentNMetricDist <= sinkMetricLen);
        agentNearbyAdjacentNMetricPos = agentMetricPos(:, agentNearbyAdjacentNIdx);
        
        % Calculate direction overlap between (agent N to "agent(s) nearby agent N and adjacent to agent N") and (agent N to sink)
        agentNAgentNearbyAdjacentNMetricVec = agentNearbyAdjacentNMetricPos - agentMetricPos(:, agentNIdx);
        agentNSinkDotAgentNAgentNearbyAdjacentN = (agentNSinkMetricVec./vecnorm(agentNSinkMetricVec))'*(agentNAgentNearbyAdjacentNMetricVec./vecnorm(agentNAgentNearbyAdjacentNMetricVec));
        [minAgentNSinkDotAgentNAgentNearbyAdjacentN, ~] = min(agentNSinkDotAgentNAgentNearbyAdjacentN, [], 'all', 'linear');

        % Are there any agents nearby agent N and adjacent to agent N? Is the "agent nearby agent N and adjacent to agent N" not in the same direction as the sink? If true, descend sink.
        if (isempty(minAgentNSinkDotAgentNAgentNearbyAdjacentN) || minAgentNSinkDotAgentNAgentNearbyAdjacentN <= 0)
            agentNState =  1;
            agentNMetricVel = agentNSinkMetricVec;

        % If false, do not descend sink.
        else
            agentNState = 0;
            agentNMetricVel = [];
        end

    % If false, do not descend sink.
    else
        agentNState = 0;
        agentNMetricVel = [];
    end
end

%% Routing Algorithm
function [agentNState, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem)
    % Local parameters
    speed = 0.25;
    horizMovementGain = 3;
    mapBoundaryTol = 0.3;
    xAgentMetricMin = xMapMetricMin + mapBoundaryTol;
    xAgentMetricMax = xMapMetricMax - mapBoundaryTol;
    yAgentMetricMin = yMapMetricMin + mapBoundaryTol;
    yAgentMetricMax = yMapMetricMax - mapBoundaryTol;

    % Move up
    if (agentNState == 0.2)
        if (agentNMetricPos(2) < yAgentMetricMax)
            agentNState = 0.2;
            agentNMetricVel = [0; speed];
        else
            routingState = [0.4, 0.6];
            agentNState = routingState(randi([1, 2], 1));
            [agentNState, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem);
        end

    % Move right
    elseif (agentNState == 0.4)
        if ((mod(round(agentNMetricPos(1), 2), horizMovementGain.*agentMetricVisibilityApothem) ~= 0) && (agentNMetricPos(1) < xAgentMetricMax))
            agentNState = 0.4;
            agentNMetricVel = [speed; 0];
        elseif (agentNMetricPos(2) >= yAgentMetricMax)
            agentNState = 0.8;
            agentNMetricVel = [0; -speed];
        elseif (agentNMetricPos(2) <= yAgentMetricMin)
            agentNState = 0.2;
            agentNMetricVel = [0; speed];
        else
            routingState = [0.2, 0.6, 0.8];
            agentNState = routingState(randi([1, 3], 1));
            [agentNState, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem);
        end

    % Move left
    elseif (agentNState == 0.6)
        if ((mod(round(agentNMetricPos(1), 2), horizMovementGain.*agentMetricVisibilityApothem) ~= 0) && (agentNMetricPos(1) > xAgentMetricMin))
            agentNState = 0.6;
            agentNMetricVel = [-speed; 0];
        elseif (agentNMetricPos(2) >= yAgentMetricMax)
            agentNState = 0.8;
            agentNMetricVel = [0; -speed];
        elseif (agentNMetricPos(2) <= yAgentMetricMin)
            agentNState = 0.2;
            agentNMetricVel = [0; speed];
        else
            routingState = [0.2, 0.4, 0.8];
            agentNState = routingState(randi([1, 3], 1));
            [agentNState, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem);
        end

    % Move down
    elseif (agentNState == 0.8)
        if (agentNMetricPos(2) > yAgentMetricMin)
            agentNState = 0.8;
            agentNMetricVel = [0; -speed];
        else
            routingState = [0.4, 0.6];
            agentNState = routingState(randi([1, 2], 1));
            [agentNState, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem);

        end
    else
        routingState = [0.2, 0.4, 0.6, 0.8];
        agentNState = routingState(randi([1, 4], 1));
        [agentNState, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem);
    end
end