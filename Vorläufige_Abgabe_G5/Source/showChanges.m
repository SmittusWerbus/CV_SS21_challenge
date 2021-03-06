function [pic,Legend] = showChanges(pic)
red = [255, 0, 0];
orange = [255, 127, 0];
yellow = [255, 255, 0];
light_green = [0, 100, 0];
dark_green = [0, 255, 0];
light_blue = [135, 206, 250];
blue = [0, 0, 255];
purple = [160, 32, 240];
pink = [255, 105, 180];
chocolate = [139, 169, 19];
color = {red, orange, yellow, light_green, dark_green, light_blue, blue, purple, pink, chocolate, [25, 55, 200], [125, 155, 20], randi([0,255],1,3), randi([0,255],1,3), randi([0,255],1,3), randi([0,255],1,3), randi([0,255],1,3), randi([0,255],1,3), randi([0,255],1,3), randi([0,255],1,3)};
color_hist = {randi([0,255],1,3), randi([0,255],1,3), randi([0,255],1,3), randi([0,255],1,3), randi([0,255],1,3), randi([0,255],1,3), randi([0,255],1,3), randi([0,255],1,3), [125, 155, 20], [25, 55, 200], chocolate, pink, purple, blue, light_blue, dark_green, light_green, yellow, orange, red};
color_index=1;
for i=2:(length(pic))
    
    j = length(pic)-i+2;
    cj = pic{1,j}.SURF.changed_points;
%     if j>2
%         cv = pic{1,j-1}.SURF.changed_points;
%         cj = cj';
%         cj_x = cj(1,:);
%         cj_y = cj(2,:);
%         cv = cv';
%         cv_x = cv(1,:);
%         cv_y = cv(2,:);
%         for k=1:length(cj_x)
%             for l=1:length(cv_x)
%                 if isequal(cj_x(k),cv_x(l)) && isequal(cj_y(k),cv_y(l))
%                     cj_x(k) = 0;
%                     cj_y(k) = 0;
%                 end
%             end
%         end
%         cj = [cj_x; cj_y];
%         cj(:,any(cj == 0))=[];
%         cj = cj';
%     end
    
    pic{1,j}.SURF.only_change = cj;
    I = insertMarker(pic{1,1}.picture, cj, 'color', color{color_index});
    pic{1,j}.SURF.change_picture = I;
    color_index=color_index+1;
end
saved_color_index=color_index;
color_index=1;
pic{1,1}.SURF.time_change_pic = pic{1,1}.picture;
Legend{1}=datestr(pic{1,1}.date);
pic{1,1}.Histo.Data=[];
pic{1,1}.Histo.only_Data=[];
pic{1,1}.SURF.change_color = [255, 255, 255];
for i=2:length(pic)
    Legend{i}=datestr(pic{1,i}.date);
    pic{1,i}.SURF.time_change_pic = insertMarker(pic{1,i-1}.SURF.time_change_pic, pic{1,i}.SURF.only_change, 'color', color{saved_color_index-color_index});
    pic{1,i}.SURF.change_color = color{i-1};
%     imshow(pic{1,i}.SURF.time_change_pic);
    color_index=color_index+1;
    pic{1,i}.Histo.Data=[pic{1,i-1}.Histo.Data,(i-1)*ones(1,length(pic{1,i}.SURF.changed_points))];
    pic{1,i}.Histo.only_Data=(i-1)*ones(1,length(pic{1,i}.SURF.changed_points));
end
pic{1,1}.color_hist = color_hist;
end

