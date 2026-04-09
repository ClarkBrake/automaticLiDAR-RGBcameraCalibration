% =========================================================
% getLidarBoardGeometry.m
% =========================================================
function [centroid, normalVec, basisX, basisY, success] = ...
    getLidarBoardGeometry(lidarPoints, initialNormal)

    success = false;
    centroid = [];
    normalVec = [];
    basisX = [];
    basisY = [];

    if size(lidarPoints, 1) < 50
        return;
    end

    lidarMean = mean(lidarPoints, 1);
    centeredPoints = lidarPoints - lidarMean;

    [~, ~, V] = svd(centeredPoints, 'econ');

    basisX = V(:, 1);
    basisY = V(:, 2);
    normalVec = V(:, 3);

    if dot(normalVec, initialNormal) < 0
        normalVec = -normalVec;
    end
    normalVec = normalVec / norm(normalVec);

    if dot(normalVec, [0; 0; 1]) > 0
        normalVec = -normalVec;
    end

    basisX = basisX / norm(basisX);
    basisY = cross(normalVec, basisX);
    basisY = basisY / norm(basisY);
    basisX = cross(basisY, normalVec);
    basisX = basisX / norm(basisX);

    u = centeredPoints * basisX;
    v = centeredPoints * basisY;

    uMin = prctile(u, 10);
    uMax = prctile(u, 90);
    vMin = prctile(v, 10);
    vMax = prctile(v, 90);

    keepMask = (u >= uMin & u <= uMax & v >= vMin & v <= vMax);
    if nnz(keepMask) < 50
        return;
    end

    trimmedPoints = lidarPoints(keepMask, :);
    trimmedMean = mean(trimmedPoints, 1);
    trimmedCentered = trimmedPoints - trimmedMean;

    [~, ~, Vtrim] = svd(trimmedCentered, 'econ');

    basisX = Vtrim(:, 1);
    basisY = Vtrim(:, 2);
    refinedNormal = Vtrim(:, 3);

    if dot(refinedNormal, normalVec) < 0
        refinedNormal = -refinedNormal;
    end
    normalVec = refinedNormal / norm(refinedNormal);

    if dot(normalVec, [0; 0; 1]) > 0
        normalVec = -normalVec;
    end

    basisX = basisX / norm(basisX);
    basisY = cross(normalVec, basisX);
    basisY = basisY / norm(basisY);
    basisX = cross(basisY, normalVec);
    basisX = basisX / norm(basisX);

    uTrim = trimmedCentered * basisX;
    vTrim = trimmedCentered * basisY;

    uCenter = (min(uTrim) + max(uTrim)) / 2;
    vCenter = (min(vTrim) + max(vTrim)) / 2;

    centroid = trimmedMean' + uCenter * basisX + vCenter * basisY;
    success = true;
end
