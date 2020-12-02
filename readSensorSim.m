%% Read Sensor Simulation
function visibleMap = readSensorSim(agentMetricPosi, agentIdxVisibilityApothem, map, xMapMetricMin, yMapMetricMin, metricToIdx, gridSensor)
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

%     % Sensor region is a grid
%     if gridSensor
%         % Calculate boundaries of visible map of agent
%         xAgentIdxMin = max((xAgentNIdxPos - agentIdxVisibleApothem), xMapIdxMin);
%         xAgentIdxMax = min((xAgentNIdxPos + agentIdxVisibleApothem), xMapIdxMax);
%         yAgentIdxMin = max((yAgentNIdxPos - agentIdxVisibleApothem), yMapIdxMin);
%         yAgentIdxMax = min((yAgentNIdxPos + agentIdxVisibleApothem), yMapIdxMax);
%         % Find visible map
%         map = flipud(map); % Transform map to (-y, x) from (y, x)
%         visibleMap = map(yAgentIdxMin:yAgentIdxMax, xAgentIdxMin:xAgentIdxMax);
%         visibleMap = flipud(visibleMap); % Transform visible map of agent to (y, x) from (-y, x)
%     % Sensor region is a circle
%     else
%         % Create a mesh over the entire map
%         [xMesh, yMesh] = meshgrid(xMapIdxMin:xMapIdxMax, yMapIdxMin:yMapIdxMax);
%         % Mask those indices which are within the sensing radius
%         sensorRegion = (yMesh - yAgentNIdxPos).^2 + (xMesh - xAgentNIdxPos).^2 <= agentIdxVisibleApothem.^2;
%         % Remove all nonzero rows and columns
%         nonZeroX = any(sensorRegion, 1);
%         nonZeroY = any(sensorRegion, 2);
%         visibleMap = sensorRegion(nonZeroX, nonZeroY);
%     end

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
    visibleMap(yVisibleMapMin:yVisibleMapMax, xVisibleMapMin:xVisibleMapMax) = map(yAgentIdxMin:yAgentIdxMax, xAgentIdxMin:xAgentIdxMax);
    visibleMap = flipud(visibleMap); % Transform visible map of agent to (y, x) from (-y, x)
end