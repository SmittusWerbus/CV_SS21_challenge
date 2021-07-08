% Pre processing der Bilder

[I1, I2] = prep;


% Feature detection 

%[points_1_res, points_2_res] = surf(I1, I2);
%[points_1_res, points_2_res] = orb(I1, I2);  % nicht wundern orb braucht ewig deswegen für demo mal surf

 
% The matrix f has a column for each frame. A frame is a disk of center f(1:2), scale f(3) and orientation f(4)
% The matrix d consists of a 128 (descriptor) x #feature/frame arrangement  
% The Keypoints consist of the feature location x & y

[f1, d1, f2, d2, Keypoints_1, Keypoints_2] = sift(I1, I2);


% Feature matching

%[matchedPoints_1, matchedPoints_2] = matching(I1,I2, points_1_res, points_2_res);
[f1match, f2match, d1match, d2match, allX, allY, allScales, allAngs, matches] = matching(f1, f2, d1, d2);



% Hough Transform binning -> ersetzt erstmal unseren Ransac
% Inliers bezeichnet die matched keypoints nach dem binning 

%[f_LMedS, inliers] = ransac(matchedPoints_1, matchedPoints_2);
[Inliers_1, Inliers_2] = hough_binning(I1, I2, f1match, f2match, allX, allY, allScales, allAngs, matches);
    

% Plotten von bereinigten matches

%plot_matches(I1,I2, matchedPoints_1, matchedPoints_2, inliers);


% Der Rest?