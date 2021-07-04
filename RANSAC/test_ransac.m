I1_raw = rgb2gray(imread('./Images/2012_08.jpg'));
I2_raw = rgb2gray(imread('./Images/2015_07.jpg'));

I1 = adapthisteq(I1_raw,'NumTiles',[10 10]);
I2 = adapthisteq(I2_raw,'NumTiles',[10 10]);

points_1 = detectSURFFeatures(I1);
points_2 = detectSURFFeatures(I2);

points_1_res = points_1.selectStrongest(50000);
points_2_res = points_1.selectStrongest(50000);


figure

tiledlayout(1,2)

ax1 = nexttile;

imshow(I1_raw);
hold on;
plot(points_1_res);

ax2 = nexttile;
imshow(I1);




[features_1, valid_points_1] = extractFeatures(I1, points_1_res);
[features_2, valid_points_2] = extractFeatures(I2, points_2_res);

pairs = matchFeatures(features_1, features_2);

matchedPoints_1 = valid_points_1(pairs(:,1),:);
matchedPoints_2 = valid_points_2(pairs(:,2),:);


%f_matrix_RANSAC = estimateFundamentalMatrix(matchedPoints_1, matchedPoints_2,'Method','RANSAC', 'NumTrials',2000,'DistanceThreshold', 1e-4)

[f_LMedS, inliers] = estimateFundamentalMatrix(matchedPoints_1,matchedPoints_2,'Method','RANSAC','NumTrials', 2000, 'InlierPercentage', 30, 'DistanceType', 'Algebraic', 'DistanceThreshold', 0.01);



figure; 
%tiledlayout(2,1)                           % Creates subplots  

%ax1 = nexttile;                            % moves over to the next subplot

hold on;



showMatchedFeatures(I1,I2,matchedPoints_1,matchedPoints_2, 'montage');
title('matched w/o ransac');
legend('matched points 1','matched points 2');

%ax2 = nexttile;
figure;
%showMatchedFeatures(I1, I2, matchedPoints_1(inliers,:),matchedPoints_2(inliers,:),'montage','PlotOptions',{'ro','go','y--'});
showMatchedFeatures(I1, I2, matchedPoints_1(inliers,:),matchedPoints_2(inliers,:),'blend','PlotOptions',{'ro','go','y--'});

title('Point matches after outliers were removed');
legend('matched points 1','matched points 2');