function [f1match, f2match, d1match, d2match, allX, allY, allScales, allAngs, matches] = matching(f1, f2, d1, d2)
%MATCHING Summary of this function goes here
%   Detailed explanation goes here

%[features_1, valid_points_1] = extractFeatures(I1, points_1_res);
%[features_2, valid_points_2] = extractFeatures(I2, points_2_res);

%pairs = matchFeatures(features_1, features_2);

%matchedPoints_1 = valid_points_1(pairs(:,1),:);
%matchedPoints_2 = valid_points_2(pairs(:,2),:);


thresh = 2.0; % default = 1.5; increase to limit matches

[matches, scores] = vl_ubcmatch(d1, d2, thresh);

fprintf('Number of matching frames (features): %d\n', size(matches,2));

indices1 = matches(1,:); % Get matching features
f1match = f1(:,indices1);
d1match = d1(:,indices1);

indices2 = matches(2,:);
f2match = f2(:,indices2);
d2match = d2(:,indices2);


% Show matches
% figure, imshow([I1,I2],[]);
% o = size(I1,2) ;
% line([f1match(1,:);f2match(1,:)+o], ...
% [f1match(2,:);f2match(2,:)]) ;
% for i=1:size(f1match,2)
% x = f1match(1,i);
% y = f1match(2,i);
% text(x,y,sprintf('%d',i), 'Color', 'r');
% end
% for i=1:size(f2match,2)
% x = f2match(1,i);
% y = f2match(2,i);
% text(x+o,y,sprintf('%d',i), 'Color', 'r');
% end


% Between all pairs of matching features, compute
% orientation difference, scale ratio, and center offset

allScales = zeros(1,size(matches,2)); % Store computed values
allAngs = zeros(1,size(matches,2));
allX = zeros(1,size(matches,2));
allY = zeros(1,size(matches,2));


for i=1:size(matches, 2)
%fprintf('Match %d: image 1 (scale,orient = %f,%f) matches', ...
%i, f1match(3,i), f1match(4,i));
%fprintf(' image 2 (scale,orient = %f,%f)\n', ...
%f2match(3,i), f2match(4,i));
scaleRatio = f1match(3,i)/f2match(3,i);
dTheta = f1match(4,i) - f2match(4,i);

% Force dTheta to be between -pi and +pi

while dTheta > pi dTheta = dTheta - 2*pi; end
while dTheta < -pi dTheta = dTheta + 2*pi; end
allScales(i) = scaleRatio;
allAngs(i) = dTheta;

x1 = f1match(1,i); % the feature in image 1
y1 = f1match(2,i);
x2 = f2match(1,i); % the feature in image 2
y2 = f2match(2,i);

% The "center" of the object in image 1 is located at an offset of
% (-x1,-y1) relative to the detected feature. We need to scale and rotate
% this offset and apply it to the image 2 location.

offset = [-x1; -y1];
offset = offset / scaleRatio; % Scale to match image 2 scale
offset = [cos(dTheta) +sin(dTheta); -sin(dTheta) cos(dTheta)]*offset;
allX(i) = x2 + offset(1);
allY(i) = y2 + offset(2);
end

% figure, plot(allScales, allAngs, '.'), xlabel('scale'), ylabel('angle');
% figure, plot(allX, allY, '.'), xlabel('x'), ylabel('y');







end

