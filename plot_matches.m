function pm = plot_matches(I1,I2, matchedPoints_1, matchedPoints_2, inliers)
%PLOT_MATCHES Summary of this function goes here
%   Detailed explanation goes here



pm = showMatchedFeatures(I1, I2, matchedPoints_1(inliers,:),matchedPoints_2(inliers,:), 'blend','PlotOptions',{'ro','go','y--'});

title('Point matches after outliers were removed');
legend('matched points 1','matched points 2');


end

