% =========================================================
% main_calibrate_lidar_camera.m
% =========================================================
clc;
clear;
close all;

%% Configuration
imageDir = fullfile('output', 'imagesCalibrationPrimary');
pointCloudDir = fullfile('output', 'pcdCalibration');

calibrationData = load('calibrationSessionCaliPrim.mat');
cameraParams = calibrationData.calibrationSession.CameraParameters;
intrinsics = cameraParams.Intrinsics;

squareSizeMM = 100;
roi = [-5 15 -10 10 -1.8 4];   % [xmin xmax ymin ymax zmin zmax]
planeDistanceThreshold = 0.03; % meters

rng('default');

%% Load image and point cloud file lists
imds = imageDatastore(imageDir);
pcds = fileDatastore(pointCloudDir, 'ReadFcn', @pcread);

imageFiles = imds.Files;
pointCloudFiles = pcds.Files;

%% Estimate checkerboard corners in the camera frame
[imageCorners3D, checkerboardDimensions, validImageMask] = ...
    estimateCheckerboardCorners3d(imageFiles, intrinsics, squareSizeMM);

imageFiles = imageFiles(validImageMask);
pointCloudFiles = pointCloudFiles(validImageMask);

numFrames = numel(imageFiles);
fprintf('Usable checkerboard detections: %d\n', numFrames);

if numFrames == 0
    error('No usable frames were found after checkerboard detection.');
end

%% Preallocate storage
lidarNormals   = [];
lidarCentroids = [];
lidarBasisX    = [];
lidarBasisY    = [];

cameraNormals   = [];
cameraCentroids = [];
cameraBasisX    = [];
cameraBasisY    = [];

validFrameIndices = [];
inlierIndices = cell(numFrames, 1);
checkerboardBBoxes = cell(numFrames, 1);

%% Process each frame
for frameIdx = 1:numFrames
    image = imread(imageFiles{frameIdx});
    pointCloud = pcread(pointCloudFiles{frameIdx});

    % Detect 2D checkerboard corners for optional bounding box storage
    [imagePoints2D, ~] = detectCheckerboardPoints(image);
    checkerboardBBoxes{frameIdx} = computeBoundingBox(imagePoints2D);

    % Crop point cloud to region of interest
    roiIndices = findPointsInROI(pointCloud, roi);
    if isempty(roiIndices)
        continue;
    end

    pointCloudROI = select(pointCloud, roiIndices);
    if pointCloudROI.Count < 200
        continue;
    end

    % Fit dominant planar surface
    [planePoints, lidarNormal, frameInlierIndices, success] = ...
        extractCheckerboardPlane(pointCloudROI, roiIndices, planeDistanceThreshold);

    if ~success || size(planePoints, 1) < 100
        continue;
    end

    % Get checkerboard geometry in the camera frame
    cameraPoints = reshape(imageCorners3D(:, :, frameIdx), [], 3);
    [camNormal, camCentroid, camX, camY, success] = ...
        getCameraBoardGeometry(cameraPoints, checkerboardDimensions);

    if ~success
        continue;
    end

    % Estimate checkerboard geometry in the LiDAR frame
    [lidarCentroid, lidarNormal, lidarX, lidarY, success] = ...
        getLidarBoardGeometry(planePoints, lidarNormal);

    if ~success
        continue;
    end

    % Align LiDAR in-plane directions with camera basis to reduce flips
    if dot(lidarX, camX) < 0
        lidarX = -lidarX;
    end
    if dot(lidarY, camY) < 0
        lidarY = -lidarY;
    end

    lidarNormal = cross(lidarX, lidarY);
    lidarNormal = lidarNormal / norm(lidarNormal);

    if dot(lidarNormal, [0; 0; 1]) > 0
        lidarNormal = -lidarNormal;
        lidarY = -lidarY;
    end

    % Store valid frame data
    lidarNormals   = [lidarNormals, lidarNormal];
    lidarCentroids = [lidarCentroids, lidarCentroid];
    lidarBasisX    = [lidarBasisX, lidarX];
    lidarBasisY    = [lidarBasisY, lidarY];

    cameraNormals   = [cameraNormals, camNormal];
    cameraCentroids = [cameraCentroids, camCentroid];
    cameraBasisX    = [cameraBasisX, camX];
    cameraBasisY    = [cameraBasisY, camY];

    inlierIndices{frameIdx} = frameInlierIndices;
    validFrameIndices = [validFrameIndices, frameIdx];
end

%% Validate frame count
numValidFrames = numel(validFrameIndices);
fprintf('Valid frames used: %d\n', numValidFrames);

if numValidFrames < 3
    error('Not enough valid frames to estimate a reliable transform.');
end

% Retain only valid frame data
imageFiles = imageFiles(validFrameIndices);
pointCloudFiles = pointCloudFiles(validFrameIndices);
imageCorners3D = imageCorners3D(:, :, validFrameIndices);
inlierIndices = inlierIndices(validFrameIndices);
checkerboardBBoxes = checkerboardBBoxes(validFrameIndices);

%% Estimate rotation
H = zeros(3, 3);
for i = 1:numValidFrames
    lidarBasis = [lidarBasisX(:, i), lidarBasisY(:, i), lidarNormals(:, i)];
    cameraBasis = [cameraBasisX(:, i), cameraBasisY(:, i), cameraNormals(:, i)];
    H = H + cameraBasis * lidarBasis';
end

[U, ~, V] = svd(H);
R = U * V';

if det(R) < 0
    U(:, 3) = -U(:, 3);
    R = U * V';
end

%% Estimate translation
tCandidates = cameraCentroids - R * lidarCentroids;
t = median(tCandidates, 2);

%% One-step refinement using centroids
rotatedLidarCentroids = R * lidarCentroids;
lidarMean = mean(rotatedLidarCentroids, 2);
cameraMean = mean(cameraCentroids, 2);

X = rotatedLidarCentroids - lidarMean;
Y = cameraCentroids - cameraMean;

Hcentroid = Y * X';
[Uc, ~, Vc] = svd(Hcentroid);
dR = Uc * Vc';

if det(dR) < 0
    Uc(:, 3) = -Uc(:, 3);
    dR = Uc * Vc';
end

R = dR * R;
t = median(cameraCentroids - R * lidarCentroids, 2);

%% Build homogeneous transform
T = eye(4);
T(1:3, 1:3) = R;
T(1:3, 4) = t;

fprintf('\nEstimated LiDAR-to-Camera Transform:\n');
disp(T);

%% Create rigid transform object if available
tform = [];
if exist('rigidtform3d', 'class') == 8
    tform = rigidtform3d(R', t');
elseif exist('rigid3d', 'class') == 8
    tform = rigid3d(R', t');
else
    warning('No rigid transform class available. Using 4x4 transform matrix only.');
end

%% Optional fusion preview
if exist('helperFuseLidarCamera1', 'file') == 2 && ~isempty(tform)
    try
        helperFuseLidarCamera1(imageFiles, pointCloudFiles, inlierIndices, cameraParams, tform);
    catch ME
        warning('Fusion preview failed: %s', ME.message);
    end
end

%% Alignment summary
rotationErrorsDeg = zeros(numValidFrames, 1);
translationErrorsMM = zeros(numValidFrames, 1);

for i = 1:numValidFrames
    predictedNormal = R * lidarNormals(:, i);
    predictedNormal = predictedNormal / norm(predictedNormal);

    rotationErrorsDeg(i) = acosd(max(-1, min(1, dot(predictedNormal, cameraNormals(:, i)))));

    predictedCentroid = R * lidarCentroids(:, i) + t;
    translationErrorsMM(i) = norm(predictedCentroid - cameraCentroids(:, i));
end

fprintf('\nMean normal alignment error: %.3f deg\n', mean(rotationErrorsDeg));
fprintf('Mean centroid alignment error: %.3f mm\n', mean(translationErrorsMM));
