function [points_1_res, points_2_res] = surf(I1,I2)
%SURF Summary of this function goes here
%   Detailed explanation goes here

points_1 = detectSURFFeatures(I1);
points_2 = detectSURFFeatures(I2);

points_1_res = points_1.selectStrongest(50000);
points_2_res = points_2.selectStrongest(50000);


end

