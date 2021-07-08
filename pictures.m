classdef pictures
    
    properties
        app
        pic
    end
    
    methods
        function obj = pictures(application,name,dir,old_pic)
            obj.app = application;
            pic = cell(1,length(old_pic)+length(name));
            pic(1,1:length(old_pic)) = old_pic;
            for i=1:length(name)
                pic{1,length(old_pic)+i} = struct;
                pic{1,length(old_pic)+i}.filename = convertCharsToStrings(name{1,i});
                pic{1,length(old_pic)+i}.directory = convertCharsToStrings(dir);
                pic{1,length(old_pic)+i}.date = obj.determine_date(name{1,i});
                pic{1,length(old_pic)+i}.picture = imread(join([dir,name{1,i}],""));
                pic{1,length(old_pic)+i}.SURF = [];
                pic{1,length(old_pic)+i}.ORB = [];
            end
            obj.pic=obj.determine_order(pic);
            % hier muss noch eine Überprüfung nach doppelten Bildern kommen
        end
        
        function obj = run_change_detection(obj,progress)
            progress.Message = 'Detecting Features';
            progress.Value = 0;
            for i=1:length(obj.pic)
                
                bw = obj.preprocessing(obj.pic{1,i}.picture);
                
                %% Feature Detection
                if(obj.app.Switch.Value=="SURF")
                    pts = detectSURFFeatures(bw);
                    [obj.pic{1,i}.SURF.features,obj.pic{1,i}.SURF.validPts] = extractFeatures(bw,pts);
                elseif(obj.app.Switch.Value=="SIFT")
                    peak_thresh = 0; % increase to limit; default is 0
                    edge_thresh = 10; % decrease to limit; default is 10
                    [f1,d1] = vl_sift(single(bw),'PeakThresh', peak_thresh,'edgethresh', edge_thresh );
                    obj.pic{1,i}.SIFT.features = f1;
                    obj.pic{1,i}.SIFT.d = d1;
                end
                progress.Value = i/length(obj.pic);
            end
            copytform=[];
            progress.Message = 'Matching Features, Scaling and Rotating Pictures';
            progress.Value = 0;
            for i=1:length(obj.pic)
                if(obj.app.Switch.Value=="SURF")
                    if(i~=length(obj.pic))
                        index_pairs = matchFeatures(obj.pic{1,i}.SURF.features,obj.pic{1,i+1}.SURF.features);
                        obj.pic{1,i}.SURF.matchedPts{1,1} = obj.pic{1,i}.SURF.validPts(index_pairs(:,1));
                        obj.pic{1,i}.SURF.matchedPts{1,2} = obj.pic{1,i+1}.SURF.validPts(index_pairs(:,2));
                    else
                        index_pairs = matchFeatures(obj.pic{1,i}.SURF.features,obj.pic{1,1}.SURF.features);
                        obj.pic{1,i}.SURF.matchedPts{1,1} = obj.pic{1,i}.SURF.validPts(index_pairs(:,1));
                        obj.pic{1,i}.SURF.matchedPts{1,2} = obj.pic{1,1}.SURF.validPts(index_pairs(:,2));
                    end
                    %%
%                     if(i==1)
% 
%                     else
%                         showMatchedFeatures(obj.pic{1,i}.picture,obj.pic{1,i+1}.picture,obj.pic{1,i}.SURF.matchedPts{1,1},obj.pic{1,i}.SURF.matchedPts{1,2})
%                     end
                    %%
                    [tform,inlierIdx] = estimateGeometricTransform2D(obj.pic{1,i}.SURF.matchedPts{1,2},obj.pic{1,i}.SURF.matchedPts{1,1},'similarity');
                    obj.pic{1,i}.SURF.filteredmatchedPts{1,1} = obj.pic{1,i}.SURF.matchedPts{1,1}(inlierIdx,:);
                    obj.pic{1,i}.SURF.filteredmatchedPts{1,2} = obj.pic{1,i}.SURF.matchedPts{1,2}(inlierIdx,:);
                    if(~isempty(copytform))
                       tform.T=tform.T*copytform.T; 
                    end
                    %%
%                     figure;
%                     if(i==1)
%                         
%                     else
%                         showMatchedFeatures(obj.pic{1,i}.picture,obj.pic{1,i+1}.picture,obj.pic{1,i}.SURF.filteredmatchedPts{1,1},obj.pic{1,i}.SURF.filteredmatchedPts{1,2})
%                     end
                    %%
                    if(i~=length(obj.pic))
                        outputView = imref2d(size(obj.pic{1,i}.picture));
                        obj.pic{1,i+1}.SURF.rotated_picture = imwarp(obj.pic{1,i+1}.picture,tform,'OutputView',outputView);
                    end
                    %% Background first try
%                     background_tform=tform;
%                     background_tform.T=single([zeros(2,3);background_tform.T(3,:)]);
%                     background = imwarp(obj.pic{1,1}.picture,background_tform);
%                     rot_pic=imresize(obj.pic{1,i+1}.SURF.rotated_picture,size(obj.pic{1,1}.picture,[1,2]));
%                     for j=1:size(rot_pic,1)
%                         for k=1:size(rot_pic,2)
%                             if(~all(rot_pic(j,k,:)))
%                                 rot_pic(j,k,:)=background(j,k,:);
%                             end
%                         end
%                     end
%                     obj.pic{1,i+1}.SURF.rotated_picture=rot_pic;
%                     figure;imshow(obj.pic{1,i+1}.SURF.rotated_picture);
                    %%
                    copytform=tform;
                elseif(obj.app.Switch.Value=="SIFT")
                    thresh = 2.0; % default = 1.5; increase to limit matches
                    if(i~=length(obj.pic))
                        [matches, scores] = vl_ubcmatch(obj.pic{1,i}.SIFT.d,obj.pic{1,i+1}.SIFT.d, thresh);
                        indices1 = matches(1,:); % Get matching features
                        obj.pic{1,i}.SIFT.matchedPts{1,1} = obj.pic{1,i}.SIFT.features(1:2,indices1)';
                        d1match = obj.pic{1,i}.SIFT.d(:,indices1);
                        indices2 = matches(2,:);
                        obj.pic{1,i}.SIFT.matchedPts{1,2} = obj.pic{1,i+1}.SIFT.features(1:2,indices2)';
                        d2match = obj.pic{1,i+1}.SIFT.d(:,indices2);
                    else
                        [matches, scores] = vl_ubcmatch(obj.pic{1,i}.SIFT.d,obj.pic{1,1}.SIFT.d, thresh);
                        indices1 = matches(1,:); % Get matching features
                        obj.pic{1,i}.SIFT.matchedPts{1,1} = obj.pic{1,i}.SIFT.features(1:2,indices1)';
                        d1match = obj.pic{1,i}.SIFT.d(:,indices1);
                        indices2 = matches(2,:);
                        obj.pic{1,i}.SIFT.matchedPts{1,2} = obj.pic{1,1}.SIFT.features(1:2,indices2)';
                        d2match = obj.pic{1,1}.SIFT.d(:,indices2);
                    end
                    [tform,inlierIdx] = estimateGeometricTransform2D(obj.pic{1,i}.SIFT.matchedPts{1,2},obj.pic{1,i}.SIFT.matchedPts{1,1},'similarity');
                    obj.pic{1,i}.SIFT.filteredmatchedPts{1,1} = obj.pic{1,i}.SIFT.matchedPts{1,1}(inlierIdx,:);
                    obj.pic{1,i}.SIFT.filteredmatchedPts{1,2} = obj.pic{1,i}.SIFT.matchedPts{1,2}(inlierIdx,:);
                    if(~isempty(copytform))
                       tform.T=copytform.T*tform.T; 
                    end
                    if(i~=length(obj.pic))
                        outputView = imref2d(size(obj.pic{1,i}.picture));
%                         outputView.XWorldLimits = outputView.XWorldLimits-mean(outputView.XWorldLimits);
%                         outputView.YWorldLimits = outputView.YWorldLimits-mean(outputView.YWorldLimits);
                        obj.pic{1,i+1}.SIFT.rotated_picture = imwarp(obj.pic{1,i+1}.picture,tform,'OutputView',outputView);
                    end
                    copytform=tform;
                end
                
                progress.Value = i/length(obj.pic);
            end
        end
        
        function date = determine_date(obj,name)
            try
                splitted = split(convertCharsToStrings(name),["_";"."]);
                date=[];
                if(length(splitted)==3)
                    for i=1:2
                        if(~isnan(str2double(splitted(i,1))))
                            date(1,i)=str2double(splitted(i,1));
                        else
                            date(1,i)=NaN;
                            uialert(obj.app.UIFigure,join(["The file ",name," has got the wrong nameclature. Please rename the file."],""),"Wrong Filename");
                        end
                    end
                    if(date(1,1)<1960 || date(1,1)>2100)
                        date(1,1)=NaN;
                        uialert(obj.app.UIFigure,join(["The file ",name," has got the wrong nameclature. Please rename the file."],""),"Wrong Filename");
                    end
                    if(date(1,2)>12 || date(1,2)<=0)
                        date(1,2)=NaN;
                        uialert(obj.app.UIFigure,join(["The file ",name," has got the wrong nameclature. Please rename the file."],""),"Wrong Filename");
                    end
                else
                    date = [NaN,NaN];
                    uialert(obj.app.UIFigure,join(["The file ",name," has got the wrong nameclature. Please rename the file."],""),"Wrong Filename");
                end
                if(all(~isnan(date)))
                    date=datetime(date(1,1),date(1,2),1);
                end
            catch
               date = [NaN,NaN];
               uialert(obj.app.UIFigure,join(["The file ",name," has got the wrong nameclature. Please rename the file."],""),"Wrong Filename");
            end
        end
        
        function pic = determine_order(~,picture)
            dates = NaT(1,length(picture));
            for i=1:length(picture)
                if(isdatetime(picture{1,i}.date))
                    dates(1,i)=picture{1,i}.date;
                end
            end
            [~,indices]=sort(dates);
            sorted_pic=cell(1,length(indices));
            for i=1:length(indices)
                sorted_pic{1,indices(1,i)}=picture{1,i};
            end
            pic=sorted_pic;
        end
        
        function old_pic = get_pic(obj)
            old_pic = obj.pic;
        end
        
        function preprocessed = preprocessing(~,picture)
            %% Color Histogram Equalization
            hsi = rgb2hsv(picture);
            histo_equal = histeq(hsi(:,:,3));
            hsi_mod = hsi;
            hsi_mod(:,:,3) = histo_equal;
            rgb = hsv2rgb(hsi_mod);
            %% Image sharpening
            sharp = imsharpen(rgb);
            %% Flat-field correction
            ffc = imflatfield(sharp,30);
            %% RGB to Gray
            bw = rgb2gray(ffc);
            preprocessed = adapthisteq(bw);
        end
    end
end