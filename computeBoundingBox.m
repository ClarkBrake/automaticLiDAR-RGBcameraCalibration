% =========================================================
% computeBoundingBox.m
% =========================================================
function bbox = computeBoundingBox(imagePoints)
    if isempty(imagePoints)
        bbox = [];
        return;
    end

    minX = min(imagePoints(:, 1));
    maxX = max(imagePoints(:, 1));
    minY = min(imagePoints(:, 2));
    maxY = max(imagePoints(:, 2));

    bbox = [minX, minY, maxX - minX, maxY - minY];
end
