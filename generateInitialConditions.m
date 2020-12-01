%% Generate Initial Conditions
function [agentMetricPos0, agentState] = generateInitialConditions(numAgent, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax)

    % Choose agents' initial pose
    agentMetricPos0 = zeros(3, numAgent);
    agentMetricPos0(1, :) = linspace(xMapMetricMin, xMapMetricMax, numAgent);
    agentMetricPos0(2, :) = yMapMetricMin;
    agentMetricPos0(3, :) = pi./2;
    
    % Choose agents' initial state
    agentState = 0.2.*ones(1, numAgent);

end