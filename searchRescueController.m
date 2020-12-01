%% Search and Rescue Controller
function [agentMetricVelNi, agentNState] = searchRescueController(agentN, agentAdjacentN, visibleMapN, agentMetricPosi, agentState, sinkMetricLen)
    % Calculate gradient of visible map of agent
    % TODO: start with metric gradient then convert to index gradient
    visibleMapN = flipud(visibleMapN); % Transform visible map of agent to (-y, x) from (y, x)
    xGradVisibleMapN = -diff(visibleMapN, 1, 2);
    yGradVisibleMapN = -diff(visibleMapN, 1, 1);
    yGradVisibleMapN = flipud(yGradVisibleMapN); % Transform y gradient of visible map of agent to (y, x) from (-y, x)

    if (agentState(:, agentN) ~= 2)
        if (agentState(:, agentN) == 1)
            agentNState = 1;
            agentMetricVelNi = [mean(xGradVisibleMapN(xGradVisibleMapN ~= 0), 'all');
                                mean(yGradVisibleMapN(yGradVisibleMapN ~= 0), 'all')];
            if agentMetricVelNi < [10.^(-5), 10.^(-5)]
                agentNState = 2;
            end
        elseif any((visibleMapN ~= 0), 'all')
            agentStateBSet = find(agentState ~= 0);
            sinkKnownSet = intersect(agentStateBSet, agentAdjacentN);
            agentAdjacentNDist = sort(vecnorm(agentMetricPosi(:, sinkKnownSet) - agentMetricPosi(:, agentN)));
            if isempty(sinkKnownSet)
                agentNState = 1;
                agentMetricVelNi = [mean(xGradVisibleMapN(xGradVisibleMapN ~= 0), 'all');
                                    mean(yGradVisibleMapN(yGradVisibleMapN ~= 0), 'all')];
            elseif (agentAdjacentNDist(:, 1) > sinkMetricLen.*sqrt(2)) % TODO: maybe not sufficient condition
                agentNState = 1;
                agentMetricVelNi = [mean(xGradVisibleMapN(xGradVisibleMapN ~= 0), 'all');
                                    mean(yGradVisibleMapN(yGradVisibleMapN ~= 0), 'all')];
            else
                agentNState = 0;
                agentMetricVelNi = [0.2; 0.2]; % TODO: allow more mobility
    %             agentMetricVelNi = zeros(2);
    %             for sinkKnown = sinkKnownSet'
    %                 agentMetricVelNi = agentMetricPosi(:, agentN) - agentMetricPosi(:, sinkKnown);
    %             end
    %             if ((agentMetricPosi(1, agentN) > 1.5) && (agentMetricPosi(2, agentN) > 0.8))
    %                 agentMetricVelNi = [-2; -2];
    %             elseif ((agentMetricPosi(1, agentN) < -1.5) && (agentMetricPosi(2, agentN) < -0.8))
    %                 agentMetricVelNi = [2; 2];
    %             elseif ((agentMetricPosi(1, agentN) > 1.5) && (agentMetricPosi(2, agentN) < -0.8))
    %                 agentMetricVelNi = [-2; 2];
    %             elseif ((agentMetricPosi(1, agentN) < -1.5) && (agentMetricPosi(2, agentN) > 0.8))
    %                 agentMetricVelNi = [2; -2];
    %             end
            end
        else
            agentNState = 0;
            agentMetricVelNi = [0.2; 0.2]; % TODO: allow more mobility
    %         agentStateBSet = find(agentState ~= 0);
    %         sinkKnownSet = intersect(agentStateBSet, agentAdjacentN);
    %         if isempty(sinkKnownSet)
    %             agentNState = 0;
    %             agentMetricVelNi = [0.2; 0.2];
    %         else
    %             agentNState = 0;
    %             agentMetricVelNi = zeros(2);
    %             for sinkKnown = sinkKnownSet'
    %                 agentMetricVelNi = agentMetricPosi(:, agentN) - agentMetricPosi(:, sinkKnown);
    %             end
    %         end
    %         if ((agentMetricPosi(1, agentN) > 1.5) && (agentMetricPosi(2, agentN) > 0.8))
    %             agentMetricVelNi = [-2; -2];
    %         elseif ((agentMetricPosi(1, agentN) < -1.5) && (agentMetricPosi(2, agentN) < -0.8))
    %             agentMetricVelNi = [2; 2];
    %         elseif ((agentMetricPosi(1, agentN) > 1.5) && (agentMetricPosi(2, agentN) < -0.8))
    %             agentMetricVelNi = [-2; 2];
    %         elseif ((agentMetricPosi(1, agentN) < -1.5) && (agentMetricPosi(2, agentN) > 0.8))
    %             agentMetricVelNi = [2; -2];
    %         end
        end
    else
        agentNState = 0;
        agentMetricVelNi = [0; 0];
    end
end