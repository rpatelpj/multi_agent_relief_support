%% Search and Rescue Controller
function dxi = searchRescueController(x, map, agentIdxRadius, metricToIdx, fieldSize, states)
    mapSize = size(map);
    xInd = (metricToIdx .* (x - [fieldSize(1); fieldSize(3)])) + 1;
    dxi = zeros(size(x));
    for agent = 1:length(x)
        % TODO: visibility disc currently a square, change to disc
        lowerXBound = floor(max((xInd(1, agent) - agentIdxRadius), 1));
        upperXBound = ceil(min((xInd(1, agent) + agentIdxRadius), mapSize(2)));
        lowerYBound = floor(max((xInd(2, agent) - agentIdxRadius), 1));
        upperYBound = ceil(min((xInd(2, agent) + agentIdxRadius), mapSize(1)));

        visibleMap = map(lowerYBound:upperYBound, lowerXBound:upperXBound);
        visibleXGrad = visibleMap(:, 1:end-1) - visibleMap(:, 2: end);
        visibleYGrad = visibleMap(1:end-1, :) - visibleMap(2: end, :);

        if any(any(visibleMap ~= 0))
            dxi(:, agent) = [mean(mean(visibleXGrad(visibleXGrad ~= 0)));
                             mean(mean(visibleYGrad(visibleYGrad ~= 0)))];
            states(:, agent) = 1;
        else
            inds = find(any(any(states(:, agent))));
          if any(inds)
              dxi(:, agent) = x(inds(1) - agent);
          else
              dxi(:, agent) = .2;
        end
    end
end
