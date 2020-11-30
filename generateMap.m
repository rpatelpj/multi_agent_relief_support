%% Generate Map
function [map, xMapMetricGrid, yMapMetricGrid, metricToIdx] = generateMap(xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax, numSink, sinkMetricLen, sinkIdxLen, sinkDepth)
    % Constrain inputs
    if (round(numSink) ~= numSink || islogical(numSink) || numSink <= 0)
        error("The number of sinks must be a positive, non-zero integer.");
    end
    if (~isreal(sinkMetricLen) || islogical(sinkMetricLen) || sinkMetricLen <= 0)
        error("The sink metric dimensions must be a positive, non-zero real number.");
    end
    if (round(sinkIdxLen) ~= sinkIdxLen || islogical(sinkIdxLen) || sinkIdxLen <= 0)
        error("The sink index dimensions must be a positive, non-zero integer.");
    end
    if (~isreal(sinkDepth) || islogical(sinkDepth) || sinkDepth <= 0)
        error("The sink depth must be a positive, non-zero real number.");
    end
    if mod(sinkIdxLen, 2) == 0
        error("The sink index dimensions must be odd to simulate a singularity.");
    end
    if (sinkMetricLen > (xMapMetricMax - xMapMetricMin) || sinkMetricLen > (yMapMetricMax - yMapMetricMin))
        error("The sink dimensions are larger than the field dimensions.");
    end

    % Convert map metric dimensions to map index dimensions
    metricToIdx = sinkIdxLen./sinkMetricLen;
    xMapMetricLen = xMapMetricMax - xMapMetricMin;
    yMapMetricLen = yMapMetricMax - yMapMetricMin;
    xMapIdxLen = floor(xMapMetricLen.*metricToIdx) + 1;
    yMapIdxLen = floor(yMapMetricLen.*metricToIdx) + 1;

    % Initialize map
    map = zeros(yMapIdxLen, xMapIdxLen);

    % Find map metric grid
    idxToMetric = 1./metricToIdx;
    xMapMetricRange = xMapMetricMin:idxToMetric:xMapMetricMax;
    yMapMetricRange = yMapMetricMin:idxToMetric:yMapMetricMax;
    [xMapMetricGrid, yMapMetricGrid] = meshgrid(xMapMetricRange, yMapMetricRange);
    yMapMetricGrid = flipud(yMapMetricGrid);

    % Calculate relative properties of sink
    xSinkApothemExc = (sinkIdxLen - 1)./2;
    ySinkApothemExc = xSinkApothemExc;
    xSinkRange = round(linspace(-xSinkApothemExc, xSinkApothemExc, sinkIdxLen)); % Round to remove empty decimal
    ySinkRange = round(linspace(-ySinkApothemExc, ySinkApothemExc, sinkIdxLen)); % Round to remove empty decimal

    % Model square sink based on paraboloid
    [xSinkGrid, ySinkGrid] = meshgrid(xSinkRange, ySinkRange);
    zSinkRaw = xSinkGrid.^2 + ySinkGrid.^2;
    if (sinkIdxLen ~= 1)
        zSink = sinkDepth.*((zSinkRaw./max(zSinkRaw, [], 'all')) - 1);
    elseif (sinkIdxLen == 1)
        zSink = -sinkDepth;
    end

    % Split map into square tiles
    xNumTile = floor(xMapIdxLen./sinkIdxLen);
    yNumTile = floor(yMapIdxLen./sinkIdxLen);
    numTile = xNumTile.*yNumTile;
    if (numSink > numTile)
        error("There are more sinks than possible tiles. Recommendation: Increase the sink index length. The number of sinks or the sink metric length can be decreased for the same effect.");
    end
    
    % Choose set of non-overlapping sink centers
    xSinkApothemInc = xSinkApothemExc + 1;
    ySinkApothemInc = xSinkApothemInc;
    xSinkCenterSet = round(linspace(xSinkApothemInc, (xMapIdxLen - xSinkApothemExc), xNumTile));
    ySinkCenterSet = round(linspace(ySinkApothemInc, (yMapIdxLen - ySinkApothemExc), yNumTile));
    sinkCenterChoiceSet = randperm(numTile, numSink);

    % Update map for each random sink
    for sinkCenterChoice = sinkCenterChoiceSet
        % Determine center of sink
        [xSinkCenterIdx, ySinkCenterIdx] = ind2sub([xNumTile, yNumTile], sinkCenterChoice);
        xSinkCenter = xSinkCenterSet(xSinkCenterIdx);
        ySinkCenter = ySinkCenterSet(ySinkCenterIdx);

        % Add sink to map
        xSinkFieldRange = xSinkRange + xSinkCenter;
        ySinkFieldRange = ySinkRange + ySinkCenter;
        map(ySinkFieldRange, xSinkFieldRange) = map(ySinkFieldRange, xSinkFieldRange) + zSink;
    end
    map = flipud(map); % Transform map to (y, x) from (-y, x)
end