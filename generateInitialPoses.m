%% Generate Initial Poses
function x0 = generateInitialPoses(numAgent, xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax)

    x0 = zeros(3, numAgent);
    x0(1, :) = linspace(xMapMetricMin, xMapMetricMax, numAgent);
    x0(2, :) = yMapMetricMin;
    x0(3, :) = pi./2;

end