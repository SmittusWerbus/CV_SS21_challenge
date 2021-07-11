function [figHeat, figCanopy] = heatmaps(image_colour, changed_points_loc, cluster_radius,index)
%HEATMAPS Summary of this function goes here
%   Detailed explanation goes here

image_gray = rgb2gray(image_colour);
writematrix(changed_points_loc, join(['points2clst',num2str(index),'.txt'],''),'WriteMode','overwrite'); 
[clustersCentroids,clustersGeoMedians,clustersXY] = clusterXYpoints(join(['points2clst',num2str(index),'.txt'],''), cluster_radius, 1,'point', 'merge');


allLengths = cellfun(@length, clustersXY);
centroid_bins = [clustersCentroids, allLengths];



F = scatteredInterpolant(centroid_bins(:,1),centroid_bins(:,2), centroid_bins(:,3), 'linear');
[qx,qy] = meshgrid(linspace(0,size(image_gray,2), size(image_gray,2)), linspace(0,size(image_gray,1), size(image_gray,1)));
qz = F(qx,qy);

figHeat={qx,qy,qz};
figCanopy={image_gray,qz};

% figure;
% tiledlayout(1,2);
% title('heatmap')
% 
% ax1 = nexttile;
% figHeat{1,1}=imshow(image_colour);
% 
% ax2 = nexttile;
% figHeat{1,1}=contourf(qx, qy, qz,'LineColor','none', 'ShowText', 'on');
% set(gca,'YDir','reverse')
% set(gca,'XTick',[])
% set(gca,'YTick',[])
% chb = colorbar('TickLabels',{'low density of change','high density of change'});
% chb.Label.String = 'relative density of change';
% set(chb,'YTick',[])
% colormap(ax2, cool)
% 
% linkaxes([ax1 ax2],'xy')
% 
% figure;
% figCanopy=meshCanopy(image_gray,qz,@cool, 100);
end

