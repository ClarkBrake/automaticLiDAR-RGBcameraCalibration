% =========================================================
% estimateCheckerboardCorners3d.m
% =========================================================
function [imageCorners3D, checkerboardSize, validFrames] = ...
    estimateCheckerboardCorners3d(imageFiles, intrinsics, squareSizeMM)

numImages = numel(imageFiles);

allImagePoints = [];
validFrames = false(numImages, 1);

fprintf('Detecting checkerboards in images...\n');

for i = 1:numImages
    I = imread(imageFiles{i});

    [imagePoints, boardSize] = detectCheckerboardPoints(I);

    if isempty(imagePoints)
        continue;
    end

    % Store only valid detections
    if isempty(allImagePoints)
        numPoints = size(imagePoints, 1);
        allImagePoints = zeros(numPoints, 2, numImages);
        checkerboardSize = boardSize;
    end

    if size(imagePoints,1) ~= numPoints
        continue;
    end

    allImagePoints(:,:,i) = imagePoints;
    validFrames(i) = true;
end

if ~any(validFrames)
    error('No checkerboards detected in any images.');
end

% Keep only valid frames
allImagePoints = allImagePoints(:,:,validFrames);

numValid = size(allImagePoints, 3);

fprintf('Valid checkerboard detections: %d\n', numValid);

% Generate world coordinates of checkerboard corners
worldPoints = generateCheckerboardPoints(checkerboardSize, squareSizeMM);

imageCorners3D = zeros(size(worldPoints,1), 3, numValid);

% Estimate pose for each frame
for i = 1:numValid
    imagePoints = allImagePoints(:,:,i);

    % Estimate extrinsics (camera pose relative to checkerboard)
    [R, t] = extrinsics(imagePoints, worldPoints, intrinsics);

    % Convert world points into camera coordinates
    camPoints = (R * worldPoints') + t';

    imageCorners3D(:,:,i) = camPoints';
end

end
