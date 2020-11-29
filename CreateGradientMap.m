function [grid] = CreateGradientMap(gridSize, tileSize, numSinks, maxMagnitude)
    if nargin == 3
        maxMagnitude = 1;
    elseif nargin ~= 4
        error("Usage: CreateGradientMap(gidSize, tileSize, numSinks, maxMagnitude=1)");
    end
    if numSinks > gridSize(1) * gridSize(2)
        error("There must be fewer sinks than grid spaces");
    end
    
    gridXWidth = gridSize(1) .* tileSize;
    gridYWidth = gridSize(2) .* tileSize;
    numTiles = gridSize(1) .* gridSize(2);
    
    gridSetup = zeros(gridSize);
    sinkIndices = randperm(numTiles, numSinks);
    gridSetup(sinkIndices) = 1;
    
    x = -tileSize/2:(tileSize/2 - 1);
    y = x;
    [xg, yg] = meshgrid(x, y);
    E = xg.^2 + yg.^2;
    sink = maxMagnitude * ((E / max(max(E)))- 1);
    
    grid = zeros(gridXWidth, gridYWidth);
    for ix = 0:gridSize(1)-1
        for iy = 0:gridSize(2)-1
            if gridSetup(ix+1, iy+1) == 1
                grid(tileSize*ix+1:tileSize*ix+tileSize, tileSize*iy+1:tileSize*iy+tileSize) = sink;
            end
        end
    end
end