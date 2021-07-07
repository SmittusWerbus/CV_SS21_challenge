%% Esketit
% Matrizen einlesen
Keypoints_Image_1 = readmatrix("Keypoints1.csv");
Keypoints_Image_2 = readmatrix("Keypoints2.csv");
% Keypoints_Image_3 = readmatrix("Keypoints2.csv");
% Keypoints_Image_4 = readmatrix("Keypoints2.csv");

Matched_Points_1_2 = readmatrix("Matches.csv");
% Matched_Points_2_3 = readmatrix("Matches.csv");
% Matched_Points_3_4 = readmatrix("Matches.csv");

%% a contrario
kx_1 = Keypoints_Image_1(1,:);
ky_1 = Keypoints_Image_1(2,:);
kx_2 = Keypoints_Image_2(1,:);
ky_2 = Keypoints_Image_2(2,:);
mx_1 = Matched_Points_1_2(1,:);
my_1 = Matched_Points_1_2(2,:);
% plot(kx_1,ky_1,'.r')
% hold on
% plot(kx_2,ky_2,'.b')
% plot(mx_1,my_1,'.g')
%%
% entferne Keypoints die auch Matched Keypoints sind aus Keypoint Matrix
for i=1:length(kx_2)
    for j=1:length(mx_1)
        if isequal(kx_2(i),mx_1(j)) && isequal(ky_2(i),my_1(j))
            kx_2(i) = 0;
            ky_2(i) = 0;
        end
    end
end
kp_2_no_matched = [kx_2; ky_2];
kp_2_no_matched(:,any(kp_2_no_matched == 0))=[];

%% Pro Keypoint Radius Ánzahl an Keypoints und Matchedpoints berechnen
r = 30;
cnt_mat = zeros(2, 3802);
for i=1:length(kp_2_no_matched)
    
    for j=1:length(kp_2_no_matched)
        if i~=j
           d1 = kp_2_no_matched(1,i)-kp_2_no_matched(1,j);
           d2 = kp_2_no_matched(2,i)-kp_2_no_matched(2,j);
           d = sqrt(d1^2+d2^2);
           if d<r
               cnt_mat(1,i) = cnt_mat(1,i) + 1;
           end
        end
    end
    
    for j=1:length(Matched_Points_1_2)
        d1 = kp_2_no_matched(1,i)-Matched_Points_1_2(1,j);
        d2 = kp_2_no_matched(2,i)-Matched_Points_1_2(2,j);
        d = sqrt(d1^2+d2^2);
        if d<r
            cnt_mat(2,i) = cnt_mat(2,i) + 1;
        end   
    end
    
end
plot(kx_2,ky_2,'.b')
%% sdrh
ergebnis = zeros(1,length(cnt_mat));
for i=1:length(cnt_mat)
    n = cnt_mat(1,i);
    m = cnt_mat(2,i);
    x = n;
    M = length(Matched_Points_1_2);
    N = length(Keypoints_Image_2);
    p = m/M;
    
    if (n-m) > 10
        ergebnis(i)=1;
    else
        ergebnis(i)=0;
    end
end
anders_x = kp_2_no_matched(1,:).*ergebnis;
anders_y = kp_2_no_matched(2,:).*ergebnis;
anders = [anders_x; anders_y];
anders(:,any(anders == 0))=[];
anders_x = anders(1,:);
anders_y = anders(2,:);
image_skia = imread("/Users/maxikrahschutz/Desktop/Studium/Master_EI/1.Semester/CV/Project/Challange/Local_Challange/Images/Dubai/2005_12.jpg");
% image_skia = rgb2gray(image_skia);
% image_skia = flipdim(image_skia ,1);
% imshow(image_skia);
% hold on
% plot(anders_x, anders_y, ".r");
for i=1:length(anders)
    disp("skia")
end