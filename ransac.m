function [f_LMedS, inliers] = ransac(matchedPoints_1, matchedPoints_2)
%RANSAC Summary of this function goes here
%   Detailed explanation goes here


[f_LMedS, inliers] = estimateFundamentalMatrix(matchedPoints_1,matchedPoints_2,'Method','RANSAC','NumTrials', 2000, 'InlierPercentage', 30, 'DistanceType', 'Algebraic', 'DistanceThreshold', 0.01);

end

