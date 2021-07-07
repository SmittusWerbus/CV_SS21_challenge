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
        
        function obj = run_change_detection(obj)
            for i=1:length(obj.pic)
                %% Color Histogram Equalization
                hsi = rgb2hsv(obj.pic{1,i}.picture);
                histo_equal = histeq(hsi(:,:,3));
                hsi_mod = hsi;
                hsi_mod(:,:,3) = histo_equal;
                rgb = hsv2rgb(hsi_mod);
                %% Image sharpening
                sharp = imsharpen(rgb);
                %% Flat-field correction
                ffc = imflatfield(sharp,30);
                %% RGB to Gray
                bw = rgb2gray(sharp);
                %% Feature Detection
                if(obj.app.Switch.Value=="SIFT")
%                     [des1,loc1] = getFeatures(bw);
%                     drawFeatures(obj.pic{1,i}.picture,loc1);
%                     detectSIFTFeatures(bw);
                    pts = detectSURFFeatures(bw);
                    [obj.pic{1,i}.SURF.features,obj.pic{1,i}.SURF.validPts] = extractFeatures(bw,pts);
%                     pts = detectMSERFeatures(bw(0.05*size(bw,1):0.95*size(bw,1),0.05*size(bw,2):0.95*size(bw,2)));
%                     [obj.pic{1,i}.SURF.features,obj.pic{1,i}.SURF.validPts] = extractFeatures(bw(0.05*size(bw,1):0.95*size(bw,1),0.05*size(bw,2):0.95*size(bw,2)),pts,'Upright',true);
%                     figure;I = obj.pic{1,i}.picture;I = insertMarker(I,obj.pic{1,i}.SURF.validPts,'x');imshow(I);
                elseif(obj.app.Switch.Value=="ORB")
                    pts = detectORBFeatures(bw);
                    obj.pic{1,i}.ORB.validPts = pts;
                end
            end
            copytform=[];
            for i=1:length(obj.pic)-1
                if(obj.app.Switch.Value=="SIFT")
                    index_pairs = matchFeatures(obj.pic{1,i}.SURF.features,obj.pic{1,i+1}.SURF.features);
                    obj.pic{1,i}.SURF.matchedPts{1,1} = obj.pic{1,i}.SURF.validPts(index_pairs(:,1));
                    obj.pic{1,i}.SURF.matchedPts{1,2} = obj.pic{1,i+1}.SURF.validPts(index_pairs(:,2));
%                     if(i==1)
% 
%                     else
%                         showMatchedFeatures(obj.pic{1,i}.picture,obj.pic{1,i+1}.picture,obj.pic{1,i}.SURF.matchedPts{1,1},obj.pic{1,i}.SURF.matchedPts{1,2})
%                     end
                    [tform,inlierIdx] = estimateGeometricTransform2D(obj.pic{1,i}.SURF.matchedPts{1,2},obj.pic{1,i}.SURF.matchedPts{1,1},'similarity');
                    obj.pic{1,i}.SURF.filteredmatchedPts{1,1} = obj.pic{1,i}.SURF.matchedPts{1,1}(inlierIdx,:);
                    obj.pic{1,i}.SURF.filteredmatchedPts{1,2} = obj.pic{1,i}.SURF.matchedPts{1,2}(inlierIdx,:);
                    if(~isempty(copytform))
                       tform.T=tform.T*copytform.T; 
                    end
%                     figure;
%                     if(i==1)
%                         
%                     else
%                         showMatchedFeatures(obj.pic{1,i}.picture,obj.pic{1,i+1}.picture,obj.pic{1,i}.SURF.filteredmatchedPts{1,1},obj.pic{1,i}.SURF.filteredmatchedPts{1,2})
%                     end
                    outputView = imref2d(size(obj.pic{1,i}.picture));
                    obj.pic{1,i+1}.SURF.rotated_picture = imwarp(obj.pic{1,i+1}.picture,tform);%,'OutputView',outputView);
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
                    copytform=tform;
                elseif(obj.app.Switch.Value=="ORB")

                end
                
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
    end
end

%Copyright (c) 2016, Starsky Wong <sununs11@gmail.com>
function [ descrs, locs ] = getFeatures( input_img )
% Function: Get sift features and descriptors
global gauss_pyr;
global dog_pyr;
global init_sigma;
global octvs;
global intvls;
global ddata_array;
global features;
if(size(input_img,3)==3)
    input_img = rgb2gray(input_img);
end
input_img = im2double(input_img);

%% Build DoG Pyramid
% initial sigma
init_sigma = 1.6;
% number of intervals per octave
intvls = 3;
s = intvls;
k = 2^(1/s);
sigma = ones(1,s+3);
sigma(1) = init_sigma;
sigma(2) = init_sigma*sqrt(k*k-1);
for i = 3:s+3
    sigma(i) = sigma(i-1)*k;
end
% default cubic method
input_img = imresize(input_img,2);
% assume the original image has a blur of sigma = 0.5
input_img = gaussian(input_img,sqrt(init_sigma^2-0.5^2*4));
% smallest dimension of top level is about 8 pixels
octvs = floor(log( min(size(input_img)) )/log(2) - 2);

% gaussian pyramid
[img_height,img_width] =  size(input_img);
gauss_pyr = cell(octvs,1);
% set image size
gimg_size = zeros(octvs,2);
gimg_size(1,:) = [img_height,img_width];
for i = 1:octvs
    if (i~=1)
        gimg_size(i,:) = [round(size(gauss_pyr{i-1},1)/2),round(size(gauss_pyr{i-1},2)/2)];
    end
    gauss_pyr{i} = zeros( gimg_size(i,1),gimg_size(i,2),s+3 );
end
for i = 1:octvs
    for j = 1:s+3
        if (i==1 && j==1)
            gauss_pyr{i}(:,:,j) = input_img;
        % downsample for the first image in an octave, from the s+1 image
        % in previous octave.
        elseif (j==1)
            gauss_pyr{i}(:,:,j) = imresize(gauss_pyr{i-1}(:,:,s+1),0.5);
        else
            gauss_pyr{i}(:,:,j) = gaussian(gauss_pyr{i}(:,:,j-1),sigma(j));
        end
    end
end
% dog pyramid
dog_pyr = cell(octvs,1);
for i = 1:octvs
    dog_pyr{i} = zeros(gimg_size(i,1),gimg_size(i,2),s+2);
    for j = 1:s+2
    dog_pyr{i}(:,:,j) = gauss_pyr{i}(:,:,j+1) - gauss_pyr{i}(:,:,j);
    end
end
% for i = 1:size(dog_pyr,1)
%     for j = 1:size(dog_pyr{i},3)
%         imwrite(im2bw(im2uint8(dog_pyr{i}(:,:,j)),0),['dog_pyr\dog_pyr_',num2str(i),num2str(j),'.png']);
%     end
% end

%% Accurate Keypoint Localization
% width of border in which to ignore keypoints
img_border = 5;
% maximum steps of keypoint interpolation
max_interp_steps = 5;
% low threshold on feature contrast
contr_thr = 0.04;
% high threshold on feature ratio of principal curvatures
curv_thr = 10;
prelim_contr_thr = 0.5*contr_thr/intvls;
ddata_array = struct('x',0,'y',0,'octv',0,'intvl',0,'x_hat',[0,0,0],'scl_octv',0);
ddata_index = 1;
for i = 1:octvs
    [height, width] = size(dog_pyr{i}(:,:,1));
    % find extrema in middle intvls
    for j = 2:s+1
        dog_imgs = dog_pyr{i};
        dog_img = dog_imgs(:,:,j);
        for x = img_border+1:height-img_border
            for y = img_border+1:width-img_border
                % preliminary check on contrast
                if(abs(dog_img(x,y)) > prelim_contr_thr)
                    % check 26 neighboring pixels
                    if(isExtremum(j,x,y))
                        ddata = interpLocation(dog_imgs,height,width,i,j,x,y,img_border,contr_thr,max_interp_steps);
                        if(~isempty(ddata))
                            if(~isEdgeLike(dog_img,ddata.x,ddata.y,curv_thr))
                                 ddata_array(ddata_index) = ddata;
                                 ddata_index = ddata_index + 1;
                            end
                        end
                    end
                end
            end
        end
    end
end

function [ out_img ] = gaussian( input_img, sigma )
    % Function: Gaussian smooth for an image
    k = 3;
    hsize = round(2*k*sigma+1);
    if mod(hsize,2) == 0
        hsize = hsize+1;
    end
    g = fspecial('gaussian',hsize,sigma);
    out_img = conv2(input_img,g,'same');
end

function [ flag ] = isExtremum( intvl, x, y)
    % Function: Find Extrema in 26 neighboring pixels
        value = dog_imgs(x,y,intvl);
        block = dog_imgs(x-1:x+1,y-1:y+1,intvl-1:intvl+1);
        if ( value > 0 && value == max(block(:)) )
            flag = 1;
        elseif ( value == min(block(:)) )
            flag = 1;
        else
            flag = 0;
        end
    end

    %% Orientation Assignment
    % number of detected points
    n = size(ddata_array,2);
    % determines gaussian sigma for orientation assignment
    ori_sig_factr = 1.5;
    % number of bins in histogram
    ori_hist_bins = 36;
    % orientation magnitude relative to max that results in new feature
    ori_peak_ratio = 0.8;
    % array of feature
    features = struct('ddata_index',0,'x',0,'y',0,'scl',0,'ori',0,'descr',[]);
    feat_index = 1;
    for i = 1:n
        ddata = ddata_array(i);
        ori_sigma = ori_sig_factr * ddata.scl_octv;
        % generate a histogram for the gradient distribution around a keypoint
        hist = oriHist(gauss_pyr{ddata.octv}(:,:,ddata.intvl),ddata.x,ddata.y,ori_hist_bins,round(3*ori_sigma),ori_sigma);
        for j = 1:2
            smoothOriHist(hist,ori_hist_bins);
        end
        % generate feature from ddata and orientation hist peak
        % add orientations greater than or equal to 80% of the largest orientation magnitude
        feat_index = addOriFeatures(i,feat_index,ddata,hist,ori_hist_bins,ori_peak_ratio);
    end

    %% Descriptor Generation
    % number of features
    n = size(features,2);
    % width of 2d array of orientation histograms
    descr_hist_d = 4;
    % bins per orientation histogram
    descr_hist_obins = 8;
    % threshold on magnitude of elements of descriptor vector
    descr_mag_thr = 0.2;
    descr_length = descr_hist_d*descr_hist_d*descr_hist_obins;
    local_features = features;
    local_ddata_array = ddata_array;
    local_gauss_pyr = gauss_pyr;
    clear features;
    clear ddata_array;
    clear gauss_pyr;
    clear dog_pyr;
    parfor feat_index = 1:n
        feat = local_features(feat_index);
        ddata = local_ddata_array(feat.ddata_index);
        gauss_img = local_gauss_pyr{ddata.octv}(:,:,ddata.intvl);
    % computes the 2D array of orientation histograms that form the feature descriptor
        hist_width = 3*ddata.scl_octv;
        radius = round( hist_width * (descr_hist_d + 1) * sqrt(2) / 2 );
        feat_ori = feat.ori;
        ddata_x = ddata.x;
        ddata_y = ddata.y;
        hist = zeros(1,descr_length);
        for i = -radius:radius
            for j = -radius:radius
                j_rot = j*cos(feat_ori) - i*sin(feat_ori);
                i_rot = j*sin(feat_ori) + i*cos(feat_ori);
                r_bin = i_rot/hist_width + descr_hist_d/2 - 0.5;
                c_bin = j_rot/hist_width + descr_hist_d/2 - 0.5;
                if (r_bin > -1 && r_bin < descr_hist_d && c_bin > -1 && c_bin < descr_hist_d)
                    mag_ori = calcGrad(gauss_img,ddata_x+i,ddata_y+j);
                    if (mag_ori(1) ~= -1)
                        ori = mag_ori(2);
                        ori = ori - feat_ori;
                        while (ori < 0)
                            ori = ori + 2*pi;
                        end
                        % i think it's theoretically impossible
                        while (ori >= 2*pi)
                            ori = ori - 2*pi;
                            disp('error, try again');
                        end
                        o_bin = ori * descr_hist_obins / (2*pi);
                        w = exp( -(j_rot*j_rot+i_rot*i_rot) / (2*(0.5*descr_hist_d*hist_width)^2) );
                        hist = interpHistEntry(hist,r_bin,c_bin,o_bin,mag_ori(1)*w,descr_hist_d,descr_hist_obins);
                    end
                end
            end
        end
        local_features(feat_index) = hist2Descr(feat,hist,descr_mag_thr);
    end
    % sort the descriptors by descending scale order
    features_scl = [local_features.scl];
    [~,features_order] = sort(features_scl,'descend');
    % return descriptors and locations
    descrs = zeros(n,descr_length);
    locs = zeros(n,2);
    for i = 1:n
        descrs(i,:) = local_features(features_order(i)).descr;
        locs(i,1) = local_features(features_order(i)).x;
        locs(i,2) = local_features(features_order(i)).y;
    end
end

function [] = drawFeatures( img, loc )
    % Function: Draw sift feature points
    figure;
    imshow(img);
    hold on;
    plot(loc(:,2),loc(:,1),'+g');
end

function [hist] = interpHistEntry(hist,r,c,o,m,d,obins)
    % Function: Interplation for hist entry
    r0 = floor(r);
    c0 = floor(c);
    o0 = floor(o);
    d_r = r - r0;
    d_c = c - c0;
    d_o = o - o0;

    for i = 0:1
        r_index = r0 + i;
        if (r_index >= 0 && r_index < d)
            for j = 0:1
                c_index = c0 + j;
                if (c_index >=0 && c_index < d)
                    for k = 0:1
                        o_index = mod(o0+k,obins);
                        % if i == 0, m*(1-d_r)*...
                        % if i == 1, m*d_r*...
                        value = m * ( 0.5 + (d_r - 0.5)*(2*i-1) ) ...
                            * ( 0.5 + (d_c - 0.5)*(2*j-1) ) ...
                            * ( 0.5 + (d_o - 0.5)*(2*k-1) );
                        hist_index = r_index*d*obins + c_index*obins + o_index +1;
                        hist(hist_index) = hist(hist_index) + value;
                    end
                end
            end
        end
    end
end

function [mag_ori] = calcGrad(img,x,y)
    % Function: Calculate gradients for image pixels
    [height,width] = size(img);
    mag_ori = [0 0];
    if (x > 1 && x < height && y > 1 && y < width)
        % x is vertical, from up to down
        dx = img(x-1,y) - img(x+1,y);
        dy = img(x,y+1) - img(x,y-1);
        mag_ori(1) = sqrt(dx*dx + dy*dy);
        mag_ori(2) = atan2(dx,dy);
    else
        mag_ori = -1;
    end
end
function [ hist ] = oriHist( img, x, y, n, rad, sigma )
    % Function: Calculate orientation histogram
    hist = zeros(n,1);
    exp_factor = 2*sigma*sigma;
    for i = -rad:rad
        for j = -rad:rad
            [mag_ori] = calcGrad(img,x+i,y+j);
            if(mag_ori(1) ~= -1)
                w = exp( -(i*i+j*j)/exp_factor );
                bin = 1 + round( n*(mag_ori(2) + pi)/(2*pi) );
                if(bin == n +1)
                    bin = 1;
                end
                hist(bin) = hist(bin) + w*mag_ori(1);
            end
        end
    end
end
function smoothOriHist(hist,n)
    % Function: Smooth orientation histogram
    for i = 1:n
        if (i==1)
            prev = hist(n);
            next = hist(2);
        elseif (i==n)
            prev = hist(n-1);
            next = hist(1);
        else
            prev = hist(i-1);
            next = hist(i+1);
        end
        hist(i) = 0.25*prev + 0.5*hist(i) + 0.25*next;
    end
end

function [ ddata ] = interpLocation( dog_imgs, height, width, octv, intvl, x, y, img_border, contr_thr, max_interp_steps )
    % Function: Interpolates a scale-space extremum's location and scale
    global init_sigma;
    global intvls;
    i = 1;
    while (i <= max_interp_steps)
        dD = deriv3D(intvl,x,y);
        H = hessian3D(intvl,x,y);
        [U,S,V] = svd(H);
        T=S;
        T(S~=0) = 1./S(S~=0);
        svd_inv_H = V * T' * U';
        x_hat = - svd_inv_H*dD;
        if( abs(x_hat(1)) < 0.5 && abs(x_hat(2)) < 0.5 && abs(x_hat(3)) < 0.5)
            break;
        end
        x = x + round(x_hat(1));
        y = y + round(x_hat(2));
        intvl = intvl + round(x_hat(3));
        if (intvl < 2 || intvl > intvls+1 || x <= img_border || y <= img_border || x > height-img_border || y > width-img_border)
            ddata = [];
            return;
        end
        i = i+1;
    end
    if (i > max_interp_steps)
        ddata = [];
        return;
    end
    contr = dog_imgs(x,y,intvl) + 0.5*dD'*x_hat;
    if ( abs(contr) < contr_thr/intvls )
        ddata = [];
        return;
    end
    ddata.x = x;
    ddata.y = y;
    ddata.octv = octv;
    ddata.intvl = intvl;
    ddata.x_hat = x_hat;
    ddata.scl_octv = init_sigma * power(2,(intvl+x_hat(3)-1)/intvls);

    function [ result ] = deriv3D(intvl, x, y)
        dx = (dog_imgs(x+1,y,intvl) - dog_imgs(x-1,y,intvl))/2;
        dy = (dog_imgs(x,y+1,intvl) - dog_imgs(x,y-1,intvl))/2;
        ds = (dog_imgs(x,y,intvl+1) - dog_imgs(x,y,intvl-1))/2;
        result = [dx,dy,ds]';
    end

    function [ result ] = hessian3D(intvl, x, y)
        center = dog_imgs(x,y,intvl);
        dxx = dog_imgs(x+1,y,intvl) + dog_imgs(x-1,y,intvl) - 2*center;
        dyy = dog_imgs(x,y+1,intvl) + dog_imgs(x,y-1,intvl) - 2*center;
        dss = dog_imgs(x,y,intvl+1) + dog_imgs(x,y,intvl-1) - 2*center;

        dxy = (dog_imgs(x+1,y+1,intvl)+dog_imgs(x-1,y-1,intvl)-dog_imgs(x+1,y-1,intvl)-dog_imgs(x-1,y+1,intvl))/4;
        dxs = (dog_imgs(x+1,y,intvl+1)+dog_imgs(x-1,y,intvl-1)-dog_imgs(x+1,y,intvl-1)-dog_imgs(x-1,y,intvl+1))/4;
        dys = (dog_imgs(x,y+1,intvl+1)+dog_imgs(x,y-1,intvl-1)-dog_imgs(x,y-1,intvl+1)-dog_imgs(x,y+1,intvl-1))/4;

        result = [dxx,dxy,dxs;dxy,dyy,dys;dxs,dys,dss];
    end

end

function [feat] = hist2Descr(feat,descr,descr_mag_thr)
    % Function: Convert histogram to descriptor
    descr = descr/norm(descr);
    descr = min(descr_mag_thr,descr);
    descr = descr/norm(descr);
    feat.descr = descr;
end

function [ flag ] = isEdgeLike( img, x, y, curv_thr )
    % Function: Remove edge-like points
    center = img(x,y);
    dxx = img(x,y+1) + img(x,y-1) - 2*center;
    dyy = img(x+1,y) + img(x-1,y) - 2*center;
    dxy = ( img(x+1,y+1) + img(x-1,y-1) - img(x+1,y-1) - img(x-1,y+1) )/4;
    tr = dxx + dyy;
    det = dxx * dyy - dxy * dxy;

    if ( det <= 0 )
        flag = 1;
        return;
    elseif ( tr^2 / det < (curv_thr + 1)^2 / curv_thr )
        flag = 0;
    else
        flag = 1;
    end
end

function [feat_index] = addOriFeatures(ddata_index,feat_index,ddata,hist,n,ori_peak_ratio)
    % Function: Add good orientation for keypoints
    global features;
    global init_sigma;
    global intvls;
    omax = dominantOri(hist,n);
    for i = 1:n
        if (i==1)
                l = n;
                r = 2;
        elseif (i==n)
            l = n-1;
            r = 1;
        else
            l = i-1;
            r = i+1;
        end
        if ( hist(i) > hist(l) && hist(i) > hist(r) && hist(i) >= ori_peak_ratio*omax )
            bin = i + interp_hist_peak(hist(l),hist(i),hist(r));
            if ( bin -1 <= 0 )
                bin = bin + n;
            % i think it's theoretically impossible
            elseif ( bin -1 > n )
                bin = bin - n;
                disp('###################what the fuck?###################');
            end
            accu_intvl = ddata.intvl + ddata.x_hat(3);
            features(feat_index).ddata_index = ddata_index;
            % first octave is double size
            features(feat_index).x = (ddata.x + ddata.x_hat(1))*2^(ddata.octv-2);
            features(feat_index).y = (ddata.y + ddata.x_hat(2))*2^(ddata.octv-2);
            features(feat_index).scl = init_sigma * power(2,ddata.octv-2 + (accu_intvl-1)/intvls);        
            features(feat_index).ori = (bin-1)/n*2*pi - pi;
            feat_index = feat_index + 1;
        end
    end
end

function [omax] = dominantOri(hist,n)
    omax = hist(1);
    for i = 2:n
        if(hist(i) > omax)
            omax = hist(i);
        end
    end
end

function [position] = interp_hist_peak(l,c,r)
    position = 0.5*(l-r)/(l-2*c+r);
end