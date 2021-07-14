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
                pic{1,length(old_pic)+i}.SIFT = [];
            end
            obj.pic=obj.determine_order(pic);
            % hier muss noch eine Überprüfung nach doppelten Bildern kommen
        end
        
        function obj = run_change_detection(obj,progress)
            progress.Title = 'Please Wait (Step 1/6)';
            progress.Message = 'Detecting Features';
            progress.Value = 0;
            for i=1:length(obj.pic)
                if(i>1)
                    obj.pic{1,i}.picture = imhistmatch(obj.pic{1,i}.picture,obj.pic{1,i-1}.picture);
                end
                bw = obj.preprocessing(obj.pic{1,i}.picture);
%                 obj.pic{1,i}.preprocessed = bw;
                %% Feature Detection
                if(obj.app.Switch.Value=="SURF")
                    pts = detectSURFFeatures(bw);
                    [obj.pic{1,i}.SURF.features,obj.pic{1,i}.SURF.validPts] = extractFeatures(bw,pts);
                elseif(obj.app.Switch.Value=="SIFT")
%                     peak_thresh = obj.app.PeakThresholdEditField.Value; % increase to limit; default is 0
%                     edge_thresh = obj.app.EdgeThresholdEditField.Value; % decrease to limit; default is 10
%                     [f1,d1] = vl_sift(single(bw),'PeakThresh', peak_thresh,'edgethresh', edge_thresh );
%                     obj.pic{1,i}.SIFT.features = f1;
%                     obj.pic{1,i}.SIFT.d = d1;
                    obj.pic{1,i}.SIFT.features = detectORBFeatures(bw);
                    obj.app.Switch.Value="SURF";
                end
                progress.Value = i/length(obj.pic);
            end
%             copytform=[];
            progress.Title = 'Please Wait (Step 2/6)';
            progress.Message = 'Matching Features, Scaling and Rotating Pictures';
            progress.Value = 0;
            tform=cell(1,length(obj.pic));
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
                    [tform{1,i},inlierIdx] = estimateGeometricTransform2D(obj.pic{1,i}.SURF.matchedPts{1,2},obj.pic{1,i}.SURF.matchedPts{1,1},'similarity');%'similarity');projective
%                     tform{1,i}=projective2d(tform{1,i}.T);
%                     if(i>1 && isa(tform{1,i-1},'projective2d'))
%                         tform{1,i}=projective2d(tform{1,i}.T);
%                     end
                    obj.pic{1,i}.SURF.filteredmatchedPts{1,1} = obj.pic{1,i}.SURF.matchedPts{1,1}(inlierIdx,:);
                    obj.pic{1,i}.SURF.filteredmatchedPts{1,2} = obj.pic{1,i}.SURF.matchedPts{1,2}(inlierIdx,:);
                    if(length(find(inlierIdx==1))<4)
                        j=0;maxD=1.5;
                        while(length(find(inlierIdx==1))<4)
                            if(i~=length(obj.pic))
                                index_pairs = matchFeatures(obj.pic{1,i-j}.SURF.features,obj.pic{1,i+1}.SURF.features);
                                copy_matchedPts{1,1} = obj.pic{1,i-j}.SURF.validPts(index_pairs(:,1));
                                copy_matchedPts{1,2} = obj.pic{1,i+1}.SURF.validPts(index_pairs(:,2));
                            else
                                index_pairs = matchFeatures(obj.pic{1,i-j}.SURF.features,obj.pic{1,1}.SURF.features);
                                copy_matchedPts{1,1} = obj.pic{1,i-j}.SURF.validPts(index_pairs(:,1));
                                copy_matchedPts{1,2} = obj.pic{1,1}.SURF.validPts(index_pairs(:,2));
                            end
                            [tform{1,i},inlierIdx] = estimateGeometricTransform2D(copy_matchedPts{1,2},copy_matchedPts{1,1},'similarity','MaxDistance',maxD);
                            copy_matchedPts{1,1} = copy_matchedPts{1,1}(inlierIdx,:);
                            copy_matchedPts{1,2} = copy_matchedPts{1,2}(inlierIdx,:);
                            j=j+1;
                            if(j==i)
                                j=0;
                                maxD=maxD+0.5;
                            end
                        end
                        good_picture=i-j+1;
                    else
                        good_picture=i;
                        
                    end
                    if(i~=1)
                        if(isa(tform{1,good_picture-1},'projective2d'))
                            transformed=affine2d([tform{1,good_picture-1}.T(1,1:2),single(0);tform{1,good_picture-1}.T(2,1:2),single(0);tform{1,good_picture-1}.T(3,1:2),single(1)]);
                            tform{1,i}.T=tform{1,good_picture}.T*transformed.T;
                        else
                            tform{1,i}.T=tform{1,i}.T*tform{1,good_picture-1}.T;
                        end

                    end
%                     tform{1,i}.T=tform{1,i}.T*tform{1,good_picture}.T;
%                     if(i~=1)
%                         tform{1,i}.T=tform{1,i}.T*tform{1,good_picture-1}.T;
%                     end
%                     if(~isempty(copytform))
%                        tform.T=tform.T*copytform.T; 
%                     end
                    %%
                    if(i~=length(obj.pic))
                        outputView = imref2d(size(obj.pic{1,i}.picture));
                        obj.pic{1,i+1}.SURF.rotated_picture = imwarp(obj.pic{1,i+1}.picture,tform{1,i},'OutputView',outputView);
%                         figure;imshow(obj.pic{1,i+1}.SURF.rotated_picture);
%                         for k=1:i
%                             if(i==1)
%                                 outputView = imref2d(size(obj.pic{1,i}.picture));
%                                 obj.pic{1,i+1}.SURF.rotated_picture = imwarp(obj.pic{1,i+1}.picture,tform{1,k},'OutputView',outputView);
%                             elseif(k==1)
%                                 copytform=tform{1,i-k+1};
%                                 copytform.T(3,1)=single(0);copytform.T(3,2)=single(0);
%                                 obj.pic{1,i+1}.SURF.rotated_picture = imwarp(obj.pic{1,i+1}.picture,copytform);
%                             elseif(k==i)
%                                 XLimit=tform{1,i-k+2}.T(3,:)*tform{1,i-k+1}.T(:,1);
%                                 YLimit=tform{1,i-k+2}.T(3,:)*tform{1,i-k+1}.T(:,2);
%                                 copytform=tform{1,i-k+1};
%                                 
%                                 copytform.T(3,1)=single(0);copytform.T(3,2)=single(0);
%                                 obj.pic{1,i+1}.SURF.rotated_picture = imwarp(obj.pic{1,i+1}.SURF.rotated_picture,copytform);%,'OutputView',outputView);
%                                 outputView = imref2d(size(obj.pic{1,i}.SURF.rotated_picture));
%                                 copyt=affine2d([1 0 0;0 1 0;XLimit YLimit 1]);
%                                 obj.pic{1,i+1}.SURF.rotated_picture = imwarp(obj.pic{1,i+1}.SURF.rotated_picture,copyt);%,'OutputView',outputView);
% %                                 obj.pic{1,i+1}.SURF.rotated_picture = imtranslate(obj.pic{1,i+1}.SURF.rotated_picture,[XLimit,YLimit]);%,'OutputView','full');
% %                                 ref=obj.pic{1,i}.SURF.rotated_picture;
% %                                 new=obj.pic{1,i+1}.SURF.rotated_picture;
% %                                 obj.pic{1,i+1}.SURF.rotated_picture = new(round((size(new,1)-size(ref,1))/2):round(size(new,1)-((size(new,1)-size(ref,1))/2)),round((size(new,2)-size(ref,2))/2):round(size(new,2)-((size(new,2)-size(ref,2))/2)),:);
%                             else
%                                 outputView = imref2d(size(obj.pic{1,i}.SURF.rotated_picture));
%                                 obj.pic{1,i+1}.SURF.rotated_picture = imwarp(obj.pic{1,i+1}.SURF.rotated_picture,tform{1,i-k+1});%,'OutputView',outputView);
% %                                 XLimit=tform{1,i-k+2}.T(3,:)*tform{1,i-k+1}.T(:,1);
% %                                 YLimit=tform{1,i-k+2}.T(3,:)*tform{1,i-k+1}.T(:,2);
%                             end
%                         end
%                         if(~exist('copy_matchedPts','Var'))
%                             moving = imhistmatch(obj.pic{1,i}.preprocessed,obj.pic{1,i+1}.preprocessed);
%                         else
%                             moving = imhistmatch(obj.pic{1,good_picture}.preprocessed,obj.pic{1,i+1}.preprocessed);
%                         end
%                         [~,movingReg] = imregdemons(moving,obj.pic{1,i+1}.preprocessed,[500,400,200],'AccumulatedFieldSmoothing',1.3);
%                         D=movingReg;
%                         obj.pic{1,i+1}.SURF.rotated_picture = imwarp(obj.pic{1,i+1}.preprocessed,D);
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
%                     copytform=tform;
                elseif(obj.app.Switch.Value=="SIFT")
                    thresh = obj.app.ThresholdMatchingEditField.Value; % default = 1.5; increase to limit matches
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
                            uialert(obj.app.SatelliteImageChangeRecognitionUIFigure,join(["The file ",name," has got the wrong nameclature. Please rename the file."],""),"Wrong Filename");
                        end
                    end
                    if(date(1,1)<1960 || date(1,1)>2100)
                        date(1,1)=NaN;
                        uialert(obj.app.SatelliteImageChangeRecognitionUIFigure,join(["The file ",name," has got the wrong nameclature. Please rename the file."],""),"Wrong Filename");
                    end
                    if(date(1,2)>12 || date(1,2)<=0)
                        date(1,2)=NaN;
                        uialert(obj.app.SatelliteImageChangeRecognitionUIFigure,join(["The file ",name," has got the wrong nameclature. Please rename the file."],""),"Wrong Filename");
                    end
                else
                    date = [NaN,NaN];
                    uialert(obj.app.SatelliteImageChangeRecognitionUIFigure,join(["The file ",name," has got the wrong nameclature. Please rename the file."],""),"Wrong Filename");
                end
                if(all(~isnan(date)))
                    date=datetime(date(1,1),date(1,2),1);
                end
            catch
               date = [NaN,NaN];
               uialert(obj.app.SatelliteImageChangeRecognitionUIFigure,join(["The file ",name," has got the wrong nameclature. Please rename the file."],""),"Wrong Filename");
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
%             %% Resize
%             picture = imresize(picture,0.75);
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
            %% Adapt Histogram Equation
            bw = imadjust(bw);
            preprocessed = adapthisteq(bw);
        end
    end
end