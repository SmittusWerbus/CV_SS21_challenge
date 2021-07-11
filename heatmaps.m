function [figHeat, figCanopy] = heatmaps(image_gray, image_colour, changed_points_loc, cluster_radius)
%HEATMAPS Summary of this function goes here
%   Detailed explanation goes here


writematrix(changed_points_loc, 'points2clst.txt'); 
[clustersCentroids,clustersGeoMedians,clustersXY] = clusterXYpoints('matches2clst.txt', cluster_radius, 1,'point', 'merge');


allLengths = cellfun(@length, clustersXY);
centroid_bins = [clustersCentroids, allLengths];



F = scatteredInterpolant(centroid_bins(:,1),centroid_bins(:,2), centroid_bins(:,3), 'linear');
[qx,qy] = meshgrid(linspace(0,size(image_gray,2), size(image_gray,2)), linspace(0,size(image_gray,1), size(image_gray,1)));
qz = F(qx,qy);



figHeat = figure;
tiledlayout(1,2);
title('heatmap')

ax1 = nexttile;
imshow(image_colour);

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

figCanopy = figure;
meshCanopy(image_gray,qz,@cool, 100);


end

