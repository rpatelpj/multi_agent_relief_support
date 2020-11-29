%% Generate Map
function [metricToIdx, xMapMetricGrid, yMapMetricGrid, map] = generateMap(xMin, xMax, yMin, yMax, numSink, sinkMetricLen, sinkIdxLen, sinkDepth)
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
    if (sinkMetricLen > (xMax - xMin) || sinkMetricLen > (yMax - yMin))
        error("The sink dimensions are larger than the field dimensions.");
    end

    % Convert map metric dimensions to map index dimensions
    metricToIdx = sinkIdxLen./sinkMetricLen;
    xMapMetricLen = xMax - xMin;
    yMapMetricLen = yMax - yMin;
    xMapIdxLen = round(xMapMetricLen.*metricToIdx) + 1;
    yMapIdxLen = round(yMapMetricLen.*metricToIdx) + 1;

    % Initialize map
    map = zeros(yMapIdxLen, xMapIdxLen);

    % Find map metric grid
    idxToMetric = 1./metricToIdx;
    xMapMetricRange = xMin:idxToMetric:xMax;
    yMapMetricRange = yMin:idxToMetric:yMax;
    [xMapMetricGrid, yMapMetricGrid] = meshgrid(xMapMetricRange, yMapMetricRange);

    % Calculate relative properties of sink
    xSinkHalfEdge = (sinkIdxLen - 1)./2;
    ySinkHalfEdge = xSinkHalfEdge;
    xSinkRange = linspace(-xSinkHalfEdge, xSinkHalfEdge, sinkIdxLen);
    ySinkRange = linspace(-ySinkHalfEdge, ySinkHalfEdge, sinkIdxLen);

    % Calculate sink gradient based on paraboloid
    [xSinkGrid, ySinkGrid] = meshgrid(xSinkRange, ySinkRange);
    zSinkRaw = xSinkGrid.^2 + ySinkGrid.^2;
    zSink = sinkDepth.*((zSinkRaw./max(zSinkRaw, [], 'all')) - 1);
    
    numTiles = [floor(xMapIdxLen / sinkIdxLen);
                floor(yMapIdxLen / sinkIdxLen)];
    possibleXCenters = linspace(ceil(sinkIdxLen./2), xMapIdxLen - ceil(sinkIdxLen./2), numTiles(1));
    possibleYCenters = linspace(ceil(sinkIdxLen./2), yMapIdxLen - ceil(sinkIdxLen./2), numTiles(2));    
    sinkCenterIdxs = randperm(numTiles(1) * numTiles(2), numSink);

    % Update map for each random sink
    for sinkCenterIdx = sinkCenterIdxs
        % Calculate center of sink        
        [row, col] = ind2sub([numTiles(1), numTiles(2)], sinkCenterIdx);
        xSinkCent = possibleXCenters(row);
        ySinkCent = possibleYCenters(col);

        % Add sink to map
        xSinkFieldRange = xSinkRange + xSinkCent;
        ySinkFieldRange = ySinkRange + ySinkCent;
        map(ySinkFieldRange, xSinkFieldRange) = map(ySinkFieldRange, xSinkFieldRange) + zSink;
    end
end