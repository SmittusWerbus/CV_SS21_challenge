function [skia] = acontario(object)
%% Esketit
alg_name = object.app.Switch.Value;
if alg_name=="SURF"
    for w=2:length(object.pic)
        % Keypoints und Matchedkeypoints einlesen
        Keypoints_Image_2 = round(object.pic{1,w}.SURF.features);
        Keypoints_Image_2 = Keypoints_Image_2([1,2],:);
        Matched_Points_1_2 = round(object.pic{1,w}.SURF.filteredmatchedPts{1,2});
        Matched_Points_1_2 = Matched_Points_1_2';
        %% a contrario
        kx_2 = Keypoints_Image_2(1,:);
        ky_2 = Keypoints_Image_2(2,:);
        mx_1 = Matched_Points_1_2(1,:);
        my_1 = Matched_Points_1_2(2,:);
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
        r = 80;
        cnt_mat = zeros(2, length(kp_2_no_matched));
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
        %% sdrh
        ergebnis = zeros(1,length(cnt_mat));
        for i=1:length(cnt_mat)
            n = cnt_mat(1,i);
            m = cnt_mat(2,i);
            x = n;
            M = length(Matched_Points_1_2);
            N = length(Keypoints_Image_2);
            p = m/M;
            
            if m>0
                ergebnis(i)=1;
            else
                ergebnis(i)=0;
            end
        end
        anders_x = kp_2_no_matched(1,:).*ergebnis;
        anders_y = kp_2_no_matched(2,:).*ergebnis;
        anders = [anders_x; anders_y];
        anders(:,any(anders == 0))=[];
        object.pic{1,w}.SURF.change = anders;
    end
elseif alg_name=="SIFT"
    for w=2:length(object.pic)
        % Keypoints und Matchedkeypoints einlesen
        Keypoints_Image_2 = round(object.pic{1,w}.SIFT.features);
        Keypoints_Image_2 = Keypoints_Image_2([1,2],:);
        Matched_Points_1_2 = round(object.pic{1,w}.SIFT.filteredmatchedPts{1,2});
        Matched_Points_1_2 = Matched_Points_1_2';
        %% a contrario
        kx_2 = Keypoints_Image_2(1,:);
        ky_2 = Keypoints_Image_2(2,:);
        mx_1 = Matched_Points_1_2(1,:);
        my_1 = Matched_Points_1_2(2,:);
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
        r = 80;
        cnt_mat = zeros(2, length(kp_2_no_matched));
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
        %% sdrh
        ergebnis = zeros(1,length(cnt_mat));
        for i=1:length(cnt_mat)
            n = cnt_mat(1,i);
            m = cnt_mat(2,i);
            x = n;
            M = length(Matched_Points_1_2);
            N = length(Keypoints_Image_2);
            p = m/M;
            
            if m>0
                ergebnis(i)=1;
            else
                ergebnis(i)=0;
            end
        end
        anders_x = kp_2_no_matched(1,:).*ergebnis;
        anders_y = kp_2_no_matched(2,:).*ergebnis;
        anders = [anders_x; anders_y];
        anders(:,any(anders == 0))=[];
        object.pic{1,w}.SIFT.change = anders;
    end
end
skia = object;
end