%% Search and Rescue Controller
function [agentNState, agentNMetricVel] = searchRescueController(agentN, agentAdjacentN, visibleMapN, agentNMetricPos0, agentState, agentMetricPos, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem, sinkMetricLen, agentNMetricVeli)
    % Transform visible map of agent to (-y, x) from (y, x)
    visibleMapN = flipud(visibleMapN);

    % State Machine
        % State 0.x: Search field
            % State 0.2: Move up
            % State 0.4: Move right
            % State 0.6: Move left
            % State 0.8: Move down
        % State 1: Descend sink

	% Is agent descending sink?
    if (agentState(1, agentN)  == 1) && any((visibleMapN ~= 0), 'all')
        [agentNSearchState, agentNMetricVel] = sinkDescentAlgorithm(visibleMapN);
        agentNState = zeros(8, 1);
        agentEdgePos = agentState(3:4, agentN);
        agentCurrPos = agentMetricPos(:, agentN)
        % did another agent already get there? 
        sinkCenters = agentState(6:7, :);
        dirToSinks = vecnorm(agentMetricPos(:, agentN)*ones(1, length(agentState)) - sinkCenters);
        [sortedSinkDistances, inds] = sort(dirToSinks);
        if agentState(8, inds(1)) == 0 || (agentState(8, inds(1)) == 1)&&(inds(1)==agentN)
            if (norm(agentNMetricVeli) < sqrt(2)*.03)
                radius = agentState(2, agentN);
                found = 1;
            else
                
                found = 0;
            end
            radius = norm(agentEdgePos - agentCurrPos);
            agentNState(1) = agentNSearchState;
            agentNState(2) = radius;
            agentNState(3:4) = agentEdgePos;
            agentNState(5) = 0;
            agentNState(6:7) = agentCurrPos;
            agentNState(8) = found;
        else
            agentState(:, agentN) = zeros(8, 1);
            [agentNState, agentNMetricVel] = routingAlgorithm(agentN, agentNMetricPos0, agentMetricPos, agentState, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem);
        end
        
    % If false, can agent see a sink?
    elseif any((visibleMapN ~= 0), 'all')
        agentState1Set = find(agentState == 1);
        sinkKnownSet = intersect(agentState1Set, agentAdjacentN);
        agentAdjacentNDist = vecnorm(agentMetricPos(:, sinkKnownSet) - agentMetricPos(:, agentN)); % Calculate distances between agent N and agents adjacent to agent N
        sinkMetricDiameter = 2 * agentMetricVisibilityApothem; %sqrt(2) .* sinkMetricLen;
        
        
        % If true, are there no agents already at a sink, or at least is there an agent not at this sink?
        % TODO: Since sink is square, sink diameter is used for threshold distance.
        %       This results in a bug when there is an open sink positioned close to two sinks at greater than twice sink apothem, but less than sink diameter.
        %       Agent cannot reach this open sink.
        % TODO: Realistically, agent wouldn't know sinkMetricDiameter. Replace with another metric.
        if isempty(sinkKnownSet) || (min(agentAdjacentNDist) > sinkMetricDiameter)
            sinkCenters = agentState(6:7, :);
            dirToSinks = vecnorm(agentMetricPos(:, agentN)*ones(1, length(agentState)) - sinkCenters);
            [sortedSinkDistances, inds] = sort(dirToSinks);
            if (agentState(2, inds(1)) < sortedSinkDistances(1))&&(agentState(8, inds(1)) == 0)
                [agentNSearchState, agentNMetricVel] = sinkDescentAlgorithm(visibleMapN);

                radius = 0;
                agentNState = zeros(8, 1);
                agentEdgePos = agentMetricPos(:, agentN);
                agentNState(1) = agentNSearchState;
                agentNState(2) = radius;
                agentNState(3:4) = agentEdgePos;
                agentNState(5) = 0;
                agentNState(6:7) = agentEdgePos;
            else
                [agentNSearchState, agentNMetricVel] = sinkDescentAlgorithm(-visibleMapN);
                agentNState = zeros(8, 1);
                agentNState(1) = agentNSearchState;
            end
        % If false, keep moving.
        else
            [agentNState, agentNMetricVel] = routingAlgorithm(agentN, agentNMetricPos0, agentMetricPos, agentState, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem);
        end

    % If false, keep moving.
    else
        [agentNState, agentNMetricVel] = routingAlgorithm(agentN, agentNMetricPos0, agentMetricPos, agentState, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem);
    end
    
end

%% Sink Descent Algorithm
function [agentNSearchState, agentNMetricVel] = sinkDescentAlgorithm(visibleMapN)
    agentNSearchState = 1;
    [rad, sensor_reading] = findSmallestInnerCircle(visibleMapN);
%     [~, minVisibleMapNIdx] = min(visibleMapN, [], 'all', 'linear');
%     [minVisibleMapNRow, minVisibleMapNCol] = ind2sub(size(visibleMapN), minVisibleMapNIdx);
%     visibleMapNCent = round(size(visibleMapN)./2);
%     agentNCentVec = [(minVisibleMapNCol - visibleMapNCent(2));
%                      (minVisibleMapNRow - visibleMapNCent(1))];
%     agentNMetricVel = 10 * agentNCentVec./(norm(agentNCentVec));
    agentNMetricVel = sensor_reading + 10.^(-5);
    
end

%% Routing Algorithm
function [agentNState, agentNMetricVel] = routingAlgorithm(agentN, agentNMetricPos0, agentMetricPos, agentState, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem)
    % Local parameters
    speed = 0.25;
    boundaryTol = 0.2;
    yAgentMetricMin = yMapMetricMin + boundaryTol;
    yAgentMetricMax = yMapMetricMax - boundaryTol;
    agentNState = agentState(:, agentN);
    % Move up
    if (agentNState(1)  == 0.2)
        if (agentMetricPos(2, agentN) < yAgentMetricMax)
            agentNState(1)  = 0.2;
            agentNMetricVel = [0; speed];
        elseif (agentNMetricPos0(1) <= 0)
            agentNState(1)  = 0.4;
            agentNMetricVel = [speed; 0];
        elseif (agentNMetricPos0(1) > 0)
            agentNState(1)  = 0.6;
            agentNMetricVel = [-speed; 0];
        else
            agentNState(1)  = 0.6;
            agentNMetricVel = [-speed; 0];
        end

    % Move right
    elseif (agentNState(1)  == 0.4)
        if (mod(round(agentMetricPos(1, agentN), 2), agentMetricVisibilityApothem) ~= 0)
            agentNState(1)  = 0.4;
            agentNMetricVel = [speed; 0];
        elseif (agentMetricPos(2, agentN) >= yAgentMetricMax)
            agentNState(1)  = 0.8;
            agentNMetricVel = [0; -speed];
        elseif (agentMetricPos(2, agentN) <= yAgentMetricMin)
            agentNState(1)  = 0.2;
            agentNMetricVel = [0; speed];
        else
            agentNState(1)  = 0.8;
            agentNMetricVel = [0; -speed];
        end

    % Move left
    elseif (agentNState(1)  == 0.6)
        if (mod(round(agentMetricPos(1, agentN), 2), agentMetricVisibilityApothem) ~= 0)
            agentNState(1)  = 0.6;
            agentNMetricVel = [-speed; 0];
        elseif (agentMetricPos(2, agentN) >= yAgentMetricMax)
            agentNState(1)  = 0.8;
            agentNMetricVel = [0; -speed];
        elseif (agentMetricPos(2, agentN) <= yAgentMetricMin)
            agentNState(1)  = 0.2;
            agentNMetricVel = [0; speed];
        else
            agentNState(1)  = 0.2;
            agentNMetricVel = [0; speed];
        end

    % Move down
    elseif (agentNState(1)  == 0.8)
        if (agentMetricPos(2, agentN) > yAgentMetricMin)
            agentNState(1)  = 0.8;
            agentNMetricVel = [0; -speed];
        elseif (agentNMetricPos0(1) <= 0)
            agentNState(1)  = 0.4;
            agentNMetricVel = [speed; 0];
        elseif (agentNMetricPos0(1) > 0)
            agentNState(1)  = 0.6;
            agentNMetricVel = [-speed; 0];
        else
            agentNState(1)  = 0.4;
            agentNMetricVel = [speed; 0];
        end
    else
        agentNState(1)  = 0.2;
        agentNMetricVel = [0; speed];
    end
end
%% Distaster Radius Calculation
function [radius] = disasterSiteDistance(edgePos, currPos)

end