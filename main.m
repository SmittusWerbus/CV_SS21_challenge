
% Pre processing der Bilder

[I1, I2] = prep;


% Feature detection 

%[points_1_res, points_2_res] = surf(I1, I2);
%[points_1_res, points_2_res] = orb(I1, I2);  % nicht wundern orb braucht ewig deswegen für demo mal surf
[f1, d1, f2, d2, Keypoints_1, Keypoints_2] = sift(I1, I2);


% Feature matching

%[matchedPoints_1, matchedPoints_2] = matching(I1,I2, points_1_res, points_2_res);
[f1match, f2match, d1match, d2match, allX, allY, allScales, allAngs, matches] = matching(f1, f2, d1, d2);



% Ransac'en

%[f_LMedS, inliers] = ransac(matchedPoints_1, matchedPoints_2);
[Inliers_1, Inliers_2] = ransac(f1match, f2match, allX, allY, allScales, allAngs, matches);


% Plotten von bereinigten matches

%plot_matches(I1,I2, matchedPoints_1, matchedPoints_2, inliers);


% Der Rest?