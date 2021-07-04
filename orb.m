function [points_1_res, points_2_res] = orb(I1,I2)
%ORB Summary of this function goes here
%   Detailed explanation goes here

points_1 = detectORBFeatures(I1);
points_2 = detectORBFeatures(I2);

points_1_res = points_1.selectStrongest(50000);
points_2_res = points_2.selectStrongest(50000);


end

