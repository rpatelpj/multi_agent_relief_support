%% Search and Rescue Controller
function [agentNStateNext, agentNMetricVel] = searchRescueController(agentNMetricPos, agentNState, visibleMapN, agentAdjacentNMetricPos, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, metricToIdx, agentMetricSensingRadius, sinkMetricLen)
    % Transform visible map of agent to (-y, x) from (y, x)
    visibleMapN = flipud(visibleMapN);

    % State Machine
        % State 0.x: Search field
            % State 0.2: Move +y direction
            % State 0.4: Move +x direction
            % State 0.6: Move -x direction
            % State 0.8: Move -y direction
        % State 1: Descend sink

    % See a sink? If true, attempt to descend sink.
    if any((visibleMapN ~= 0), 'all')
        [agentNStateNext, agentNMetricVel] = sinkDescentAlgorithm(agentNMetricPos, visibleMapN, agentAdjacentNMetricPos, xMapMetricMin, yMapMetricMin, metricToIdx, sinkMetricLen);

        % If no open sink, continue to search field.
        if (agentNStateNext == 0)
            [agentNStateNext, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, agentMetricSensingRadius, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax);
        end

    % If no visible sink, continue to search field.
    else
        [agentNStateNext, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, agentMetricSensingRadius, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax);
    end
end

%% Sink Descent Algorithm
function [agentNState, agentNMetricVel] = sinkDescentAlgorithm(agentNMetricPos, visibleMapN, agentAdjacentNMetricPos, xMapMetricMin, yMapMetricMin, metricToIdx, sinkMetricLen)
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
    sinkMapMetricPos = agentNSinkMetricVec + agentNMetricPos;

    % Find minimum distance between sink and all agents adjacent to agent N and agent N
    agentSetMetricPos = [agentNMetricPos, agentAdjacentNMetricPos];
    agentSetSinkDist = vecnorm(sinkMapMetricPos - agentSetMetricPos);
    [minAgentSinkDist, ~] = min(agentSetSinkDist);
    minAgentSetSinkDistIdx = find(agentSetSinkDist == minAgentSinkDist);

    % Is agent N one of the closest to the sink?
    if any(minAgentSetSinkDistIdx == 1)
        % Find distance to "agent(s) nearby agent N and adjacent to agent N"
        agentNAgentAdjacentNMetricVec = agentAdjacentNMetricPos - agentNMetricPos;
        agentNAgentAdjacentNMetricDist = vecnorm(agentNAgentAdjacentNMetricVec);
        agentNAgentNearbyNMetricVec = agentNAgentAdjacentNMetricVec(:, agentNAgentAdjacentNMetricDist <= sinkMetricLen);

        % Find smallest direction overlap between (agent N to "agent(s) nearby agent N and adjacent to agent N") and (agent N to sink)
        agentNSinkDotAgentNAgentNearbyN = (agentNSinkMetricVec./vecnorm(agentNSinkMetricVec))'*(agentNAgentNearbyNMetricVec./vecnorm(agentNAgentNearbyNMetricVec));
        [maxAgentNSinkDotAgentNAgentNearbyN, ~] = max(agentNSinkDotAgentNAgentNearbyN);

        % Are there any agents nearby agent N and adjacent to agent N? Is the "agent nearby agent N and adjacent to agent N" not in the same direction as the sink from the perspective of agent N? If true, descend sink.
        if (isempty(maxAgentNSinkDotAgentNAgentNearbyN) || maxAgentNSinkDotAgentNAgentNearbyN <= 0)
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
function [agentNState, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, agentMetricSensingRadius, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax)
    % Local parameters
    speed = 0.25;
    horizMovementGain = 3;
    mapBoundaryTol = 0.3;
    xAgentMetricMin = xMapMetricMin + mapBoundaryTol;
    xAgentMetricMax = xMapMetricMax - mapBoundaryTol;
    yAgentMetricMin = yMapMetricMin + mapBoundaryTol;
    yAgentMetricMax = yMapMetricMax - mapBoundaryTol;

    % Move +y direction
    if (agentNState == 0.2)
        if (agentNMetricPos(2) < yAgentMetricMax)
            agentNState = 0.2;
            agentNMetricVel = [0; speed];
        else
            routingState = [0.4, 0.6];
            agentNState = routingState(randi([1, 2], 1));
            [agentNState, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricSensingRadius);
        end

    % Move +x direction
    elseif (agentNState == 0.4)
        if ((mod(round(agentNMetricPos(1), 2), horizMovementGain.*agentMetricSensingRadius) ~= 0) && (agentNMetricPos(1) < xAgentMetricMax))
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
            [agentNState, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricSensingRadius);
        end

    % Move -x direction
    elseif (agentNState == 0.6)
        if ((mod(round(agentNMetricPos(1), 2), horizMovementGain.*agentMetricSensingRadius) ~= 0) && (agentNMetricPos(1) > xAgentMetricMin))
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
            [agentNState, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricSensingRadius);
        end

    % Move -y direction
    elseif (agentNState == 0.8)
        if (agentNMetricPos(2) > yAgentMetricMin)
            agentNState = 0.8;
            agentNMetricVel = [0; -speed];
        else
            routingState = [0.4, 0.6];
            agentNState = routingState(randi([1, 2], 1));
            [agentNState, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricSensingRadius);

        end
    else
        routingState = [0.2, 0.4, 0.6, 0.8];
        agentNState = routingState(randi([1, 4], 1));
        [agentNState, agentNMetricVel] = routingAlgorithm(agentNMetricPos, agentNState, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, agentMetricSensingRadius);
    end
end