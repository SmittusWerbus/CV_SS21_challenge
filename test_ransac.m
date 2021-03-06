


close all;


I1_raw = rgb2gray(imread(fullfile('images','Dubai','2003_12.jpg')));
I2_raw = rgb2gray(imread(fullfile('images','Dubai','2005_12.jpg')));
I2_colour = imread(fullfile('images','Dubai','2005_12.jpg'));


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

figure

imshow(I2); hold on;
%
plot(points_2);

hold off;
%f_matrix_RANSAC = estimateFundamentalMatrix(matchedPoints_1, matchedPoints_2,'Method','RANSAC', 'NumTrials',2000,'DistanceThreshold', 1e-4)

[f_LMedS, inliers] = estimateFundamentalMatrix(matchedPoints_1,matchedPoints_2,'Method','RANSAC','NumTrials', 2000, 'InlierPercentage', 30, 'DistanceType', 'Algebraic', 'DistanceThreshold', 0.01);
%[tform,inliers] = estimateGeometricTransform2D(matchedPoints_1,matchedPoints_2)



%[t1, t2] = estimateUncalibratedRectification(f,inlier_points1,...
 %   inlier_points2,size(I2));


%figure; 
%tiledlayout(2,1)                           % Creates subplots  

%ax1 = nexttile;                            % moves over to the next subplot

%hold on;



showMatchedFeatures(I1,I2,matchedPoints_1,matchedPoints_2, 'montage');
%title('matched w/o ransac');
%legend('matched points 1','matched points 2');

%ax2 = nexttile;
%figure;
%showMatchedFeatures(I1, I2, matchedPoints_1(inliers,:),matchedPoints_2(inliers,:),'montage','PlotOptions',{'ro','go','y--'});

%[clustersCentroids,clustersGeoMedians,clustersXY] = clusterXYpoints(inputfile,maxdist,minClusterSize,method,mergeflag);
%showMatchedFeatures(I1, I2, matchedPoints_1(inliers,:),matchedPoints_2(inliers,:),'blend','PlotOptions',{'ro','go','y--'});


% clustering der changed punkte ??ber radien/area

% Hier muss noch von matched points zu changed points ge???ndert werden, reicht aber
% zum rumprobieren

cluster_radius = 75;

writematrix(matchedPoints_2.Location, 'matches2clst.txt'); 

[clustersCentroids,clustersGeoMedians,clustersXY] = clusterXYpoints('matches2clst.txt', cluster_radius, 1,'centroid', 'merge');

allLengths = cellfun(@length, clustersXY);

% Now find the longest vector between element 15 and 35 (for example)
maxLength = max(allLengths);
centroid_bins = [clustersCentroids, allLengths];
clusters_mat = cell2mat(clustersXY);

figure;
showMatchedFeatures(I1, I2, matchedPoints_1(inliers,:),matchedPoints_2(inliers,:),'montage','PlotOptions',{'ro','go','y--'});
hold on;

x0 = size(I2,2);
y0 = size(I2,1);


%   for i=1:length(clustersCentroids)
%         xunit = x0 + clustersCentroids(i,1);
%         yunit =  -(clustersCentroids(2,i)) + y0 ;
%       plot(xunit, yunit, 'Ob', 'MarkerSize',cluster_radius)
%       
%   end

%  plot(clustersCentroids(:,1),clustersCentroids(:,2), 'Ob', 'MarkerSize',cluster_radius)
% %set(gca,'YDir','reverse')
%  
F = scatteredInterpolant(centroid_bins(:,1),centroid_bins(:,2), centroid_bins(:,3), 'linear');
 



%[qx,qy] = meshgrid(double(I2(1,:)), double(I2(:,2)));
%[qx,qy] = meshgrid(centroid_bins(:,1), centroid_bins(:,2));


[qx,qy] = meshgrid(linspace(0,size(I2,2), size(I2,2)), linspace(0,size(I2,1), size(I2,1)));
qz = F(qx,qy);
 
figure
mesh(qx,qy,qz)
%hold on
figure; hold on;
for i=1:length(centroid_bins)
    plot3(centroid_bins(i,1), centroid_bins(i,2) , centroid_bins(i,3),'o')
end
hold off



%figure
meshCanopy(I2,qz,@cool, 100)

figure;
title('heatmap')
contourf(qx, qy, qz,'LineColor','none', 'ShowText', 'on')
hold on;

im = imshow(I2);
im.AlphaData = 0.3;
hold off
legend('Sample Points','Interpolated Surface','Location','NorthWest')


figure
[c,cont] = contour(qx, qy, qz);
colorbar
contP = get(cont,'Parent'); 
X = contP.XLim ;
Y = contP.YLim ;

hold on

imgh = imshow(I2,'XData',X,'YData',Y); 
imgh.AlphaData = .2 



figHeat = figure;
title('heatmap')
tiledlayout(1,2);

ax1 = nexttile;
imshow(I2_colour);

ax2 = nexttile;
contourf(qx, qy, qz,'LineColor','none', 'ShowText', 'on')
set(gca,'YDir','reverse')
set(gca,'XTick',[])
set(gca,'YTick',[])
chb = colorbar('TickLabels',{'low density of change','high density of change'});
chb.Label.String = 'relative density of change';
set(chb,'YTick',[])
colormap(ax2, cool)

linkaxes([ax1 ax2],'xy')


%ax3([2 2]) = nexttile;

meshCanopy_2(I2,qz,@cool, 100);
set(gca,'XTick',[])
set(gca,'YTick',[])
chb = colorbar('TickLabels',{'low density of change','high density of change'});
chb.Label.String = 'relative density of change';
set(chb,'YTick',[])
set(chb, 'ylim', [255 500])
%colormap(cool)

