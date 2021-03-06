function [Inliers_1, Inliers_2, I1Rect, I2Rect] = hough_binning(I1, I2, f1match, f2match, d1match, d2match, allX, allY, allScales, allAngs, matches)
%RANSAC Summary of this function goes here
%   Detailed explanation goes here


%[f_LMedS, inliers] = estimateFundamentalMatrix(matchedPoints_1,matchedPoints_2,'Method','RANSAC','NumTrials', 2000, 'InlierPercentage', 30, 'DistanceType', 'Algebraic', 'DistanceThreshold', 0.01);

% Use a coarse Hough space.
% Dimensions are [angle, scale, x, y]
% Define bin centers

aBin = -pi:(pi/4):pi;
sBin = 0.5:(2):10;
xBin = 1:(size(I2,2)/5):size(I2,2);
yBin = 1:(size(I2,1)/5):size(I2,1);

H = zeros(length(aBin), length(sBin), length(xBin), length(yBin));

for i=1:size(matches, 2)
    a = allAngs(i);
    s = allScales(i);
    x = allX(i);
    y = allY(i);
    % Find bin that is closest to a,s,x,y
    [~, ia] = min(abs(a-aBin));
    [~, is] = min(abs(s-sBin));
    [~, ix] = min(abs(x-xBin));
    [~, iy] = min(abs(y-yBin));
    H(ia,is,ix,iy) = H(ia,is,ix,iy) + 1; % Inc accumulator array
end

% Find all bins with 3 or more features

[ap,sp,xp,yp] = ind2sub(size(H), find(H>=3));
%fprintf('Peaks in the Hough array:\n');

%for i=1:length(ap)
    %fprintf('%d: %d points, (a,s,x,y) = %f,%f,%f,%f\n', ...
    %i, H(ap(i), sp(i), xp(i), yp(i)), ...
    %aBin(ap(i)), sBin(sp(i)), xBin(xp(i)), yBin(yp(i)) );
%end


% Get the features corresponding to the largest bin

nFeatures = max(H(:)); % Number of features in largest bin

%fprintf('Largest bin contains %d features\n', nFeatures);
[ap,sp,xp,yp] = ind2sub(size(H), find(H == nFeatures));
indices = []; % Make a list of indices

for i=1:size(matches, 2)
    a = allAngs(i);
    s = allScales(i);
    x = allX(i);
    y = allY(i);
    
% Find bin that is closest to a,s,x,y
    [~, ia] = min(abs(a-aBin));
    [~, is] = min(abs(s-sBin));
    [~, ix] = min(abs(x-xBin));
    [~, iy] = min(abs(y-yBin));
    if ia==ap(1) && is==sp(1) && ix==xp(1) && iy==yp(1)
    indices = [indices i];
end
end
%fprintf('Features belonging to highest peak:\n');
%disp(indices);


% Show matches to features in largest bin as line segments
Inliers_1 = zeros(2, length(indices));
Inliers_2 = zeros(2, length(indices));
figure, imshow([I1,I2],[]);
o = size(I1,2) ;
line([f1match(1,indices);f2match(1,indices)+o], ...
[f1match(2,indices);f2match(2,indices)]) ;
title('showing matches via toolbox');


for i=1:length(indices)
x = f1match(1,indices(i));
y = f1match(2,indices(i));
Inliers_1(1,i) = [x];
Inliers_1(2,i) = [y];
%text(x,y,sprintf('%d',indices(i)), 'Color', 'r');
end

for i=1:length(indices)
x = f2match(1,indices(i));
y = f2match(2,indices(i));
Inliers_2(1,i) = [x];
Inliers_2(2,i) = [y];
%text(x+o,y,sprintf('%d',indices(i)), 'Color', 'r');
end

% transpose of Inliers to account for the syntax of proporetary matlab
% functions

[f_LMedS, inliers] = estimateFundamentalMatrix(transpose(Inliers_1), transpose(Inliers_2),'Method','RANSAC','NumTrials', 2000, 'InlierPercentage', 30, 'DistanceType', 'Algebraic', 'DistanceThreshold', 0.01);
[t1, t2] = estimateUncalibratedRectification(f_LMedS,transpose(Inliers_1),...
    transpose(Inliers_2),size(I2));

[I1Rect,I2Rect] = rectifyStereoImages(I1,I2,t1,t2);


figure;
imshow(I1Rect);
imshow(I2Rect);

%[t1, t2] = estimateUncalibratedRectification(f_LMedS,inlier_points1,...
%    inlier_points2,size(I2));

%[I1Rect,I2Rect] = rectifyStereoImages(I1,I2,t1,t2);
%figure;
%imshow(I1Rect);

%figure;
%imshow(I2Rect)
%figure
%showMatchedFeatures(I1,I2, transpose(Inliers_1), transpose(Inliers_2), 'Method', 'blend');
%title('showing matches via prop matlab');

% Clustering of inliers via k-means, wieder raus

%trans_inliers = transpose(Inliers_2);
%trans_keys = transpose(Keypoints_2);
%[idx, ctrs] = kmeans(trans_keys, length(Inliers_2));

%figure
%showMatchedFeatures(I1,I2, transpose(Inliers_1), transpose(Inliers_2), 'Method', 'blend');
%title('showing matches via prop matlab');
%hold on;
%PlotClusters(trans_keys, idx, ctrs)



%plot(x,y)

