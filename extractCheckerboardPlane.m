% =========================================================
% extractCheckerboardPlane.m
% =========================================================
function [planePoints, planeNormal, originalInlierIndices, success] = ...
    extractCheckerboardPlane(pointCloudROI, roiIndices, maxDistance)

    success = false;
    planePoints = [];
    planeNormal = [];
    originalInlierIndices = [];

    try
        [model1, inliers1] = pcfitplane(pointCloudROI, maxDistance);
    catch
        return;
    end

    normal1 = model1.Normal(:) / norm(model1.Normal);

    % If the first plane is likely the floor, remove it and refit
    if abs(dot(normal1, [0; 0; 1])) > 0.9
        remainingIndices = setdiff((1:pointCloudROI.Count)', inliers1);
        if numel(remainingIndices) < 200
            return;
        end

        pointCloudNoFloor = select(pointCloudROI, remainingIndices);

        try
            [model2, inliers2] = pcfitplane(pointCloudNoFloor, maxDistance);
        catch
            return;
        end

        planeCloud = select(pointCloudNoFloor, inliers2);
        planePoints = reshape(planeCloud.Location, [], 3);
        originalInlierIndices = roiIndices(remainingIndices(inliers2));
        planeNormal = model2.Normal(:);
    else
        planeCloud = select(pointCloudROI, inliers1);
        planePoints = reshape(planeCloud.Location, [], 3);
        originalInlierIndices = roiIndices(inliers1);
        planeNormal = model1.Normal(:);
    end

    if isempty(planePoints)
        return;
    end

    planeNormal = planeNormal / norm(planeNormal);
    success = true;
end
