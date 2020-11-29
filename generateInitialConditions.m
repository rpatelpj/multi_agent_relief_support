%% Generate Initial Conditions
function x0 = generateInitialConditions(numAgent, xMin, xMax, yMin, yMax)

    x0 = zeros(3, numAgent);
    x0(1, :) = linspace(xMin, xMax, numAgent);
    x0(2, :) = yMin;
    x0(3, :) = pi./2;

end