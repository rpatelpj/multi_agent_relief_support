%% Read Sensor Simulation
function visibleMap = readSensorSim(agentMetricPosi, agentIdxVisibleApothem, map, xMapMetricMin, yMapMetricMin, metricToIdx, gridSensor)
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
    
    % Sensor region is a grid
    if gridSensor
        % Calculate boundaries of visible map of agent
        xAgentIdxMin = max((xAgentNIdxPos - agentIdxVisibleApothem), xMapIdxMin);
        xAgentIdxMax = min((xAgentNIdxPos + agentIdxVisibleApothem), xMapIdxMax);
        yAgentIdxMin = max((yAgentNIdxPos - agentIdxVisibleApothem), yMapIdxMin);
        yAgentIdxMax = min((yAgentNIdxPos + agentIdxVisibleApothem), yMapIdxMax);
        % Find visible map
        map = flipud(map); % Transform map to (-y, x) from (y, x)
        visibleMap = map(yAgentIdxMin:yAgentIdxMax, xAgentIdxMin:xAgentIdxMax);
        visibleMap = flipud(visibleMap); % Transform visible map of agent to (y, x) from (-y, x)
    % Sensor region is a circle
    else
        % Create a mesh over the entire map
        [xMesh, yMesh] = meshgrid(xMapIdxMin:xMapIdxMax, yMapIdxMin:yMapIdxMax);
        % Mask those indices which are within the sensing radius
        sensorRegion = (yMesh - yAgentNIdxPos).^2 + (xMesh - xAgentNIdxPos).^2 <= agentIdxVisibleApothem.^2;
        % Remove all nonzero rows and columns
        nonZeroX = any(sensorRegion, 1);
        nonZeroY = any(sensorRegion, 2);
        visibleMap = sensorRegion(nonZeroX, nonZeroY);
    end
end