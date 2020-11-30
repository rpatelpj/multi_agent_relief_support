%% Search and Rescue Controller
function [agentMetricVelNi, agentNState] = searchRescueController(agentN, agentAdjacentN, visibleMapN, agentMetricPosi, agentState)
    % Calculate gradient of visible map of agent
    visibleMapN = flipud(visibleMapN); % Transform visible map of agent to (-y, x) from (y, x)
    xGradVisibleMapN = -diff(visibleMapN, 1, 2);
    yGradVisibleMapN = -diff(visibleMapN, 1, 1);
    yGradVisibleMapN = flipud(yGradVisibleMapN); % Transform y gradient of visible map of agent to (y, x) from (-y, x)

    % State B: descend sink
    % TODO: Make sure agent doesn't go into same sink as another agent. Need to restrict state B for that effect.
    if any((visibleMapN ~= 0), 'all')
        agentNState = 1;
        agentMetricVelNi = [mean(xGradVisibleMapN(xGradVisibleMapN ~= 0), 'all');
                            mean(yGradVisibleMapN(yGradVisibleMapN ~= 0), 'all')];
    % State A: search for sink
    else
        agentNState = 0;
        agentStateBSet = find(any(agentState == 1, 'all'));
        sinkKnownSet = intersect(agentStateBSet, agentAdjacentN);
        % If no sink is found, move randomly
        if isempty(sinkKnownSet)
            agentMetricVelNi = [0.2; 0.2]; % TODO: Find better way to initially move.
        % If sink is found, avoid it
        else
              agentMetricVelNi = zeros(2);
              for sinkKnown = sinkKnownSet
                  agentMetricVelNi = agentMetricPosi(:, sinkKnown) - agentMetricPosi(:, agentN);
              end
        end
    end
end