%% Esketit
% Matrizen einlesen
Keypoints_Image_1 = readmatrix("Keypoints1.csv");
Keypoints_Image_2 = readmatrix("Keypoints2.csv");
% Keypoints_Image_3 = readmatrix("lelele");
% Keypoints_Image_4 = readmatrix("lelele");

Matched_Points_1_2 = readmatrix("Matches.csv");
% Matched_Points_2_3 = readmatrix("lelokr");
% Matched_Points_3_4 = readmatrix("lelokr");

%% a contrario
x_1 = Keypoints_Image_1(1,:);
y_1 = Keypoints_Image_1(2,:);
x_2 = Keypoints_Image_2(1,:);
y_2 = Keypoints_Image_2(2,:);
mx_1 = Matched_Points_1_2(1,:);
my_1 = Matched_Points_1_2(2,:);
plot(x_1,y_1,'.r')
hold on
plot(x_2,y_2,'.b')
plot(mx_1,my_1,'.g')
%%
% schmeiß raus was gleich is
lel=1;
for i=1:length(x_2)
    for j=1:length(mx_1)
        if isequal(x_2(i),mx_1(j)) && isequal(y_2(i),my_1(j))
            x_2(i) = 0;
            y_2(i) = 0;
        end
    end
end
kp_2_no_matched = [x_2; y_2];
kp_2_no_matched(:,any(kp_2_no_matched == 0))=[];

% for keypoint_2: