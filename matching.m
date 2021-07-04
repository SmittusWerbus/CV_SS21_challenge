function [matchedPoints_1, matchedPoints_2] = matching(I1,I2, points_1_res, points_2_res)
%MATCHING Summary of this function goes here
%   Detailed explanation goes here

[features_1, valid_points_1] = extractFeatures(I1, points_1_res);
[features_2, valid_points_2] = extractFeatures(I2, points_2_res);

pairs = matchFeatures(features_1, features_2);

matchedPoints_1 = valid_points_1(pairs(:,1),:);
matchedPoints_2 = valid_points_2(pairs(:,2),:);

end

