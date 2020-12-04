%% Read Sensor Simulation
function visibleMap = readSensorSim(agentMetricPosi, agentIdxVisibilityApothem, map, xMapMetricMin, yMapMetricMin, metricToIdx, gridSensorRange)
    % Convert agent metric position to agent index position
    xAgentNMetricPos = agentMetricPosi(1);
    yAgentNMetricPos = agentMetricPosi(2);
    xAgentNIdxPos = floor((xAgentNMetricPos - xMapMetricMin).*metricToIdx) + 1;
    yAgentNIdxPos = floor((yAgentNMetricPos - yMapMetricMin).*metricToIdx) + 1;

    % Find boundaries of map
    xMapIdxMin = 1;
    xMapIdxMax = size(map, 2);
    yMapIdxMin = 1;
    yMapIdxMax = size(map, 1);

    % Calculate range of visible map of agent inside map
    xAgentIdxMin = max((xAgentNIdxPos - agentIdxVisibilityApothem), xMapIdxMin);
    xAgentIdxMax = min((xAgentNIdxPos + agentIdxVisibilityApothem), xMapIdxMax);
    yAgentIdxMin = max((yAgentNIdxPos - agentIdxVisibilityApothem), yMapIdxMin);
    yAgentIdxMax = min((yAgentNIdxPos + agentIdxVisibilityApothem), yMapIdxMax);
    
    % Calculate boundaries of full visible map of agent
    xFullVisibleMapMin = 1;
    yFullVisibleMapMin = xFullVisibleMapMin;
    xFullVisibleMapMax = 2.*agentIdxVisibilityApothem + 1;
    yFullVisibleMapMax = xFullVisibleMapMax;

    % Calculate range of visible map of agent with map boundaries inside of full visible map
    if (xAgentNIdxPos - agentIdxVisibilityApothem) >= xMapIdxMin
        xVisibleMapMin = xFullVisibleMapMin;
    else
        xVisibleMapMin = xFullVisibleMapMin + (xMapIdxMin - (xAgentNIdxPos - agentIdxVisibilityApothem));
    end

    if (xAgentNIdxPos + agentIdxVisibilityApothem) <= xMapIdxMax
        xVisibleMapMax = xFullVisibleMapMax;
    else
        xVisibleMapMax = xFullVisibleMapMax - ((xAgentNIdxPos + agentIdxVisibilityApothem) - xMapIdxMax);
    end

    if (yAgentNIdxPos - agentIdxVisibilityApothem) >= yMapIdxMin
        yVisibleMapMin = yFullVisibleMapMin;
    else
        yVisibleMapMin = yFullVisibleMapMin + (yMapIdxMin - (yAgentNIdxPos - agentIdxVisibilityApothem));
    end

    if (yAgentNIdxPos + agentIdxVisibilityApothem) <= yMapIdxMax
        yVisibleMapMax = yFullVisibleMapMax;
    else
        yVisibleMapMax = yFullVisibleMapMax - ((yAgentNIdxPos + agentIdxVisibilityApothem) - yMapIdxMax);
    end

    % Assemble visible map of agent
    visibleMap = zeros(length(xFullVisibleMapMin:xFullVisibleMapMax), length(yFullVisibleMapMin:yFullVisibleMapMax));
    map = flipud(map); % Transform map to (-y, x) from (y, x)

    % Create visible map using grid sensor range?
    if (gridSensorRange == true)
        visibleMap(yVisibleMapMin:yVisibleMapMax, xVisibleMapMin:xVisibleMapMax) = map(yAgentIdxMin:yAgentIdxMax, xAgentIdxMin:xAgentIdxMax);

    % If false, create visible map using circular sensor range.
    else
        % Create a mesh over the entire map
        [xMesh, yMesh] = meshgrid(yVisibleMapMin:yVisibleMapMax, xVisibleMapMin:xVisibleMapMax);
        % Mask those indices which are within the sensing radius
        sensorRegion = (yMesh - yAgentNIdxPos).^2 + (xMesh - xAgentNIdxPos).^2 <= agentIdxVisibilityApothem.^2;
        map(sensorRegion == 0) = 0;
        visibleMap(yVisibleMapMin:yVisibleMapMax, xVisibleMapMin:xVisibleMapMax) = map(yAgentIdxMin:yAgentIdxMax, xAgentIdxMin:xAgentIdxMax);
    end

    visibleMap = flipud(visibleMap); % Transform visible map of agent to (y, x) from (-y, x)
end