
% Pre processing der Bilder

[I1, I2] = prep;


% Feature detection 

[points_1_res, points_2_res] = surf(I1, I2);


% Feature matching

 [matchedPoints_1, matchedPoints_2] = matching(I1,I2, points_1_res, points_2_res);


% Ransac'en

[f_LMedS, inliers] = ransac(matchedPoints_1, matchedPoints_2);


% Plotten von matches

plot_matches(I1,I2, matchedPoints_1, matchedPoints_2, inliers);


% Der Rest?