%% Read Sensor Simulation
function visibleMap = readSensorSim(agentMetricPosi, agentIdxVisibleApothem, map, xMapMetricMin, yMapMetricMin, metricToIdx)
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

    % Calculate boundaries of visible map of agent
    xAgentIdxMin = max((xAgentNIdxPos - agentIdxVisibleApothem), xMapIdxMin);
    xAgentIdxMax = min((xAgentNIdxPos + agentIdxVisibleApothem), xMapIdxMax);
    yAgentIdxMin = max((yAgentNIdxPos - agentIdxVisibleApothem), yMapIdxMin);
    yAgentIdxMax = min((yAgentNIdxPos + agentIdxVisibleApothem), yMapIdxMax);

    % Find visible map
    map = flipud(map); % Transform map to (-y, x) from (y, x)
    visibleMap = map(yAgentIdxMin:yAgentIdxMax, xAgentIdxMin:xAgentIdxMax);
    visibleMap = flipud(visibleMap); % Transform visible map of agent to (y, x) from (-y, x)
end