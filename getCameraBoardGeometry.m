% =========================================================
% getCameraBoardGeometry.m
% =========================================================
function [normalVec, centroid, basisX, basisY, success] = ...
    getCameraBoardGeometry(cameraPoints, checkerboardDimensions)

    success = false;
    normalVec = [];
    centroid = [];
    basisX = [];
    basisY = [];

    if any(isnan(cameraPoints), 'all') || size(cameraPoints, 1) < 4
        return;
    end

    boardRows = checkerboardDimensions(1) - 1;
    boardCols = checkerboardDimensions(2) - 1;

    if size(cameraPoints, 1) ~= boardRows * boardCols
        return;
    end

    cameraGrid = reshape(cameraPoints, [boardCols, boardRows, 3]);
    cameraGrid = permute(cameraGrid, [2 1 3]);

    topLeft = squeeze(cameraGrid(1, 1, :));
    topRight = squeeze(cameraGrid(1, end, :));
    bottomLeft = squeeze(cameraGrid(end, 1, :));
    bottomRight = squeeze(cameraGrid(end, end, :));

    basisX = topRight - topLeft;
    basisY = bottomLeft - topLeft;

    basisX = basisX / norm(basisX);
    basisY = basisY - dot(basisY, basisX) * basisX;
    basisY = basisY / norm(basisY);

    normalVec = cross(basisX, basisY);
    normalVec = normalVec / norm(normalVec);

    if dot(normalVec, [0; 0; 1]) > 0
        normalVec = -normalVec;
        basisY = -basisY;
    end

    centroid = (topLeft + topRight + bottomLeft + bottomRight) / 4;
    success = true;
end
