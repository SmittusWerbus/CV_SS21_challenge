function [f1, d1, f2, d2, Keypoints_1, Keypoints_2] = sift(I1, I2)
%SIFT Summary of this function goes here
%   Detailed explanation goes here


% These parameters limit the number of features detected
peak_thresh = 0; % increase to limit; default is 0
edge_thresh = 10; % decrease to limit; default is 10

[f1,d1] = vl_sift(I1,'PeakThresh', peak_thresh, 'edgethresh', edge_thresh );

Keypoints_1 = [f1(1,:);f1(2,:)];

% These parameters limit the number of features detected

peak_thresh = 0; % increase to limit; default is 0
edge_thresh = 10; % decrease to limit; default is 10
[f2,d2] = vl_sift(I2, 'PeakThresh', peak_thresh, 'edgethresh', edge_thresh );


Keypoints_2 = [f2(1,:);f2(2,:)];


end

