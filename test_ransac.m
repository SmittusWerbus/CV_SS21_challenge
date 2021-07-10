


close all;

I1_raw = rgb2gray(imread(fullfile('images','Wiesn','2018_04.jpg')));
I2_raw = rgb2gray(imread(fullfile('images','Wiesn','2019_09.jpg')));

I1 = adapthisteq(I1_raw,'NumTiles',[10 10]);
I2 = adapthisteq(I2_raw,'NumTiles',[10 10]);

points_1 = detectSURFFeatures(I1);
points_2 = detectSURFFeatures(I2);

[f1, d1] = vl_sift(single(I1));

[f2, d2] = vl_sift(single(I2));

descrip_1 = transpose(d1);
descrip_2 = transpose(d2);


%sift_points_1 = SIFTPoints('LOCATION', 1,2 );

% Hand over of x & y coordinates to the SIFTPoints object with inheritance
% of the SURFPoints class

sift_points_1 = SURFPoints(transpose(f1([1 2], :)));
sift_points_1.Scale = transpose(f1(3,:));
sift_points_1.Orientation = transpose(f1(4,:));

sift_points_2 = SURFPoints(transpose(f2([1 2], :)));
sift_points_2.Scale = transpose(f2(3,:));
sift_points_2.Orientation = transpose(f2(4,:));



%[matches, scores] = vl_ubcmatch(da, db)

points_1_res = sift_points_1;
points_2_res = sift_points_2;


%points_1_res = points_1%.selectStrongest(50000);
%points_2_res = points_2%.selectStrongest(50000);


% figure
% 
% tiledlayout(1,2)
% 
% ax1 = nexttile;
% 
% imshow(I1_raw);
% hold on;
% plot(points_1_res);
% 
% ax2 = nexttile;
% imshow(I1);
% 



[features_1, valid_points_1] = extractFeatures(I1, points_1_res);
[features_2, valid_points_2] = extractFeatures(I2, points_2_res);

pairs = matchFeatures(features_1, features_2);
%pairs = matchFeatures(descrip_1, descrip_2);


matchedPoints_1 = valid_points_1(pairs(:,1),:);
matchedPoints_2 = valid_points_2(pairs(:,2),:);


imshow(I2); hold on;
%
plot(points_2);

%f_matrix_RANSAC = estimateFundamentalMatrix(matchedPoints_1, matchedPoints_2,'Method','RANSAC', 'NumTrials',2000,'DistanceThreshold', 1e-4)

[f_LMedS, inliers] = estimateFundamentalMatrix(matchedPoints_1,matchedPoints_2,'Method','RANSAC','NumTrials', 2000, 'InlierPercentage', 30, 'DistanceType', 'Algebraic', 'DistanceThreshold', 0.01);
%[tform,inliers] = estimateGeometricTransform2D(matchedPoints_1,matchedPoints_2)



%[t1, t2] = estimateUncalibratedRectification(f,inlier_points1,...
 %   inlier_points2,size(I2));


%figure; 
%tiledlayout(2,1)                           % Creates subplots  

%ax1 = nexttile;                            % moves over to the next subplot

%hold on;



%showMatchedFeatures(I1,I2,matchedPoints_1,matchedPoints_2, 'montage');
%title('matched w/o ransac');
%legend('matched points 1','matched points 2');

%ax2 = nexttile;
%figure;
%showMatchedFeatures(I1, I2, matchedPoints_1(inliers,:),matchedPoints_2(inliers,:),'montage','PlotOptions',{'ro','go','y--'});

%[clustersCentroids,clustersGeoMedians,clustersXY] = clusterXYpoints(inputfile,maxdist,minClusterSize,method,mergeflag);
%showMatchedFeatures(I1, I2, matchedPoints_1(inliers,:),matchedPoints_2(inliers,:),'blend','PlotOptions',{'ro','go','y--'});


% Hier muss noch von matched points zu changed points ge�ndert werden, reicht aber
% zum rumprobieren

writematrix(matchedPoints_2.Location, 'matches2clst.txt'); 

[clustersCentroids,clustersGeoMedians,clustersXY] = clusterXYpoints('matches2clst.txt', 50, 1,'point', 'merge');

allLengths = cellfun(@length, clustersXY);
% Now find the longest vector between element 15 and 35 (for example)
maxLength = max(allLengths);

centroid_bins = [clustersCentroids, allLengths];


clusters_mat = cell2mat(clustersXY);

figure; hold on;

H = showMatchedFeatures(I1, I2, matchedPoints_1(inliers,:),matchedPoints_2(inliers,:),'montage','PlotOptions',{'ro','go','y--'});

x0 = 0;
y0 = size(I2,1);



  for i=1:length(clustersCentroids)
        xunit = x0 + clustersCentroids(i,1);
        yunit =  -(clustersCentroids(i,2)) ;
      plot(xunit, -yunit, 'Ob', 'MarkerSize',50)
  end

  %hold off;
 % Interpolation zwischen den Datenpunkten
 

 %F = TriScatteredInterp(centroid_bins(:,1), centroid_bins(:,2), centroid_bins(:,3));
%F = scatteredInterpolant(centroid_bins(:,[1 2]), centroid_bins(:,3), 'linear', 'linear');
 
F = scatteredInterpolant(centroid_bins(:,2),centroid_bins(:,1), centroid_bins(:,3), 'natural');
 



%[qx,qy] = meshgrid(double(I2(1,:)), double(I2(:,2)));
%[qx,qy] = meshgrid(centroid_bins(:,1), centroid_bins(:,2));

[qx,qy] = meshgrid(linspace(0,size(I2,2)), linspace(0,size(I2,1)));
qz = F(qx,qy);
 
mesh(qx,qy,qz)
%hold on
figure; hold on;
for i=1:length(centroid_bins)
    plot3(centroid_bins(i,2), centroid_bins(i,1) , centroid_bins(i,3),'o')
end
hold off



%figure
%meshCanopy(I2,qz,@spring)
figure; 


contourf(qx, qy, qz,'LineColor','none', 'ShowText', 'on')
hold on;

imshow(I2);
title('heatmap')

hold off
alpha(.1) 
legend('Sample Points','Interpolated Surface','Location','NorthWest')



%DataDensityPlot(size(I2,1),size(I2,1) , centroid_bins(:,3) );

%[xG, yG] = meshgrid(-5:5);
%sigma = 2.5;
%g = exp(-xG.^2./(2.*sigma.^2)-yG.^2./(2.*sigma.^2));
%g = g./sum(g(:));
 

%imagesc(pts, pts, conv2(N, g, 'same'));

 
%title('Point matches after outliers were removed');
%legend('matched points 1','matched points 2');