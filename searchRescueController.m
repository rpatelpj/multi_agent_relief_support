%% Search and Rescue Controller
function [agentNState, agentNMetricVel] = searchRescueController(agentN, agentAdjacentN, visibleMapN, agentNMetricPos0, agentState, agentMetricPos, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem, sinkMetricLen)
    % Transform visible map of agent to (-y, x) from (y, x)
    visibleMapN = flipud(visibleMapN);

    % State Machine
        % State 0.x: Search field
            % State 0.2: Move up
            % State 0.4: Move right
            % State 0.6: Move left
            % State 0.8: Move down
        % State 1: Descend sink
        dx = agentMetricPos(1, agentN) - agentMetricPos(1, agentAdjacentN);
        dy = agentMetricPos(2, agentN) - agentMetricPos(2, agentAdjacentN);
        
        normDist = sqrt(dx .^ 2 + dy .^ 2);
        closeAgents = agentAdjacentN(normDist < agentMetricVisibilityApothem);
        dx_close = agentMetricPos(1, agentN) - agentMetricPos(1, closeAgents);
        dy_close = agentMetricPos(2, agentN) - agentMetricPos(2, closeAgents);
        
        agentNState = 0;
        
        [~, gradMetricVel] = sinkDescentAlgorithm(visibleMapN);
        dx_close_sum = sum(max(-1,min(1, 1./(dx.^3))));
        dy_close_sum = sum(max(-1,min(1, 1./(dy.^3))));
        avoidMetricVel = [dx_close_sum; dy_close_sum];
       
        
        agentNMetricVel = gradMetricVel + 0.08 * avoidMetricVel;
        
        test = 5;

% 	% Is agent descending sink?
%     if (agentState(agentN) == 1) && any((visibleMapN ~= 0), 'all')
%         [agentNState, agentNMetricVel] = sinkDescentAlgorithm(visibleMapN);
% 
%     % If false, can agent see a sink?
%     elseif any((visibleMapN ~= 0), 'all')
%         agentState1Set = find(agentState == 1);
%         sinkKnownSet = intersect(agentState1Set, agentAdjacentN);
%         agentAdjacentNDist = vecnorm(agentMetricPos(:, sinkKnownSet) - agentMetricPos(:, agentN)); % Calculate distances between agent N and agents adjacent to agent N
%         sinkMetricDiameter = 2 * agentMetricVisibilityApothem; %sqrt(2) .* sinkMetricLen;
% 
%         % If true, are there no agents already at a sink, or at least is there an agent not at this sink?
%         % TODO: Since sink is square, sink diameter is used for threshold distance.
%         %       This results in a bug when there is an open sink positioned close to two sinks at greater than twice sink apothem, but less than sink diameter.
%         %       Agent cannot reach this open sink.
%         % TODO: Realistically, agent wouldn't know sinkMetricDiameter. Replace with another metric.
%         if isempty(sinkKnownSet) || (min(agentAdjacentNDist) > sinkMetricDiameter)
%             [agentNState, agentNMetricVel] = sinkDescentAlgorithm(visibleMapN);
% 
%         % If false, keep moving.
%         else
%             [agentNState, agentNMetricVel] = routingAlgorithm(agentN, agentNMetricPos0, agentMetricPos, agentState, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem);
%         end
% 
%     % If false, keep moving.
%     else
%         [agentNState, agentNMetricVel] = routingAlgorithm(agentN, agentNMetricPos0, agentMetricPos, agentState, yMapMetricMin, yMapMetricMax, agentMetricVisibilityApothem);
%     end
end

%% Sink Descent Algorithm
function [agentNState, agentNMetricVel] = sinkDescentAlgorithm(visibleMapN)
    agentNState = 1;
    [~, sensor_reading] = findSmallestInnerCircle(visibleMapN);
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

    % Move up
    if (agentState(agentN) == 0.2)
        if (agentMetricPos(2, agentN) < yAgentMetricMax)
            agentNState = 0.2;
            agentNMetricVel = [0; speed];
        elseif (agentNMetricPos0(1) <= 0)
            agentNState = 0.4;
            agentNMetricVel = [speed; 0];
        elseif (agentNMetricPos0(1) > 0)
            agentNState = 0.6;
            agentNMetricVel = [-speed; 0];
        else
            agentNState = 0.6;
            agentNMetricVel = [-speed; 0];
        end

    % Move right
    elseif (agentState(agentN) == 0.4)
        if (mod(round(agentMetricPos(1, agentN), 2), agentMetricVisibilityApothem) ~= 0)
            agentNState = 0.4;
            agentNMetricVel = [speed; 0];
        elseif (agentMetricPos(2, agentN) >= yAgentMetricMax)
            agentNState = 0.8;
            agentNMetricVel = [0; -speed];
        elseif (agentMetricPos(2, agentN) <= yAgentMetricMin)
            agentNState = 0.2;
            agentNMetricVel = [0; speed];
        else
            agentNState = 0.8;
            agentNMetricVel = [0; -speed];
        end

    % Move left
    elseif (agentState(agentN) == 0.6)
        if (mod(round(agentMetricPos(1, agentN), 2), agentMetricVisibilityApothem) ~= 0)
            agentNState = 0.6;
            agentNMetricVel = [-speed; 0];
        elseif (agentMetricPos(2, agentN) >= yAgentMetricMax)
            agentNState = 0.8;
            agentNMetricVel = [0; -speed];
        elseif (agentMetricPos(2, agentN) <= yAgentMetricMin)
            agentNState = 0.2;
            agentNMetricVel = [0; speed];
        else
            agentNState = 0.2;
            agentNMetricVel = [0; speed];
        end

    % Move down
    elseif (agentState(agentN) == 0.8)
        if (agentMetricPos(2, agentN) > yAgentMetricMin)
            agentNState = 0.8;
            agentNMetricVel = [0; -speed];
        elseif (agentNMetricPos0(1) <= 0)
            agentNState = 0.4;
            agentNMetricVel = [speed; 0];
        elseif (agentNMetricPos0(1) > 0)
            agentNState = 0.6;
            agentNMetricVel = [-speed; 0];
        else
            agentNState = 0.4;
            agentNMetricVel = [speed; 0];
        end
    else
        agentNState = 0.2;
        agentNMetricVel = [0; speed];
    end
end