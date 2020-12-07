%% Read Sensor Simulation
function visibleMap = readSensorSim(agentMetricPosi, agentIdxSensingRadius, map, xMapMetricMin, yMapMetricMin, metricToIdx)
    % Convert agent metric position to agent index position
    xAgentMetricPos = agentMetricPosi(1);
    yAgentMetricPos = agentMetricPosi(2);
    xAgentIdxPos = floor((xAgentMetricPos - xMapMetricMin).*metricToIdx) + 1;
    yAgentIdxPos = floor((yAgentMetricPos - yMapMetricMin).*metricToIdx) + 1;

    % Find boundaries of map
    xMapIdxMin = 1;
    xMapIdxMax = size(map, 2);
    yMapIdxMin = 1;
    yMapIdxMax = size(map, 1);

    % Calculate range of visible map of agent inside map
    xAgentIdxMin = max((xAgentIdxPos - agentIdxSensingRadius), xMapIdxMin);
    xAgentIdxMax = min((xAgentIdxPos + agentIdxSensingRadius), xMapIdxMax);
    yAgentIdxMin = max((yAgentIdxPos - agentIdxSensingRadius), yMapIdxMin);
    yAgentIdxMax = min((yAgentIdxPos + agentIdxSensingRadius), yMapIdxMax);
    
    % Calculate boundaries of full visible map of agent
    xFullVisibleMapMin = 1;
    yFullVisibleMapMin = xFullVisibleMapMin;
    xFullVisibleMapMax = 2.*agentIdxSensingRadius + 1;
    yFullVisibleMapMax = xFullVisibleMapMax;

    % Calculate range of visible map of agent with map boundaries inside of full visible map
    if (xAgentIdxPos - agentIdxSensingRadius) >= xMapIdxMin
        xVisibleMapMin = xFullVisibleMapMin;
    else
        xVisibleMapMin = xFullVisibleMapMin + (xMapIdxMin - (xAgentIdxPos - agentIdxSensingRadius));
    end

    if (xAgentIdxPos + agentIdxSensingRadius) <= xMapIdxMax
        xVisibleMapMax = xFullVisibleMapMax;
    else
        xVisibleMapMax = xFullVisibleMapMax - ((xAgentIdxPos + agentIdxSensingRadius) - xMapIdxMax);
    end

    if (yAgentIdxPos - agentIdxSensingRadius) >= yMapIdxMin
        yVisibleMapMin = yFullVisibleMapMin;
    else
        yVisibleMapMin = yFullVisibleMapMin + (yMapIdxMin - (yAgentIdxPos - agentIdxSensingRadius));
    end

    if (yAgentIdxPos + agentIdxSensingRadius) <= yMapIdxMax
        yVisibleMapMax = yFullVisibleMapMax;
    else
        yVisibleMapMax = yFullVisibleMapMax - ((yAgentIdxPos + agentIdxSensingRadius) - yMapIdxMax);
    end

    % Assemble visible map of agent
    visibleMap = zeros(length(xFullVisibleMapMin:xFullVisibleMapMax), length(yFullVisibleMapMin:yFullVisibleMapMax));
    map = flipud(map); % Transform map to (-y, x) from (y, x)

    % Fill in visible map within agent sensing apothem
    visibleMap(yVisibleMapMin:yVisibleMapMax, xVisibleMapMin:xVisibleMapMax) = map(yAgentIdxMin:yAgentIdxMax, xAgentIdxMin:xAgentIdxMax);

    % Crop visible map to within agent sensing radius
    [xFullVisibleMapGrid, yFullVisibleMapGrid] = meshgrid(xFullVisibleMapMin:xFullVisibleMapMax, yFullVisibleMapMin:yFullVisibleMapMax);
    agentIdxPosRelVisibleMap = round(size(visibleMap)./2);
    agentSensingRegion = (yFullVisibleMapGrid - agentIdxPosRelVisibleMap(1)).^2 + (xFullVisibleMapGrid - agentIdxPosRelVisibleMap(2)).^2 <= agentIdxSensingRadius.^2;
    visibleMap(~agentSensingRegion) = 0;

    % Transform visible map of agent to (y, x) from (-y, x)
    visibleMap = flipud(visibleMap);
end