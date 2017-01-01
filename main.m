%%======================================================================
%% Script to create a panorama image from a sequence of images in `imagepath' directory.
%%======================================================================

clear;

% run('vlfeat-0.9.19\toolbox\vl_setup.m'); vl_version;

imagepath = 'input_images';   % Assumes images are in order and are the *only* files in this directory.

% Read in the list of filenames of images to be processed
imagelist = dir(imagepath);

% Remove invisible Thumbs.db file that's usually in Windows machines
imagelist = imagelist(arrayfun(@(x) ~strcmp(x.name, 'Thumbs.db'), imagelist));

% Remove files that start with '.', including '.' and '..'
imagelist = imagelist(arrayfun(@(x) x.name(1) ~= '.', imagelist));

% Compute feature points
for i = 1 : length(imagelist)
    filename = fullfile(imagepath, imagelist(i).name);
    input_image = single(rgb2gray(imread(filename)));
    [keypoints{i}, descriptors{i}] = vl_sift(input_image);
end

% Here we compute feature points and the homography matrices between
% adjacent input images. Homography matrices between adjacent input images
% are then stored in a cell array H_list.

for i = 1 : length(imagelist) - 1
    
    % Assign relevant data structures to variables for convenience
    descriptors1 = descriptors{i};
    descriptors2 = descriptors{i + 1};
    keypoints1 = keypoints{i};
    keypoints2 = keypoints{i + 1};
    
    % Find matching feature points between current two images. Note that this
    % code does NOT include the use of RANSAC to find and use only good
    % matches.
    [matches, scores] = vl_ubcmatch(descriptors1, descriptors2) ;
    im1_ftr_pts = keypoints1([2 1], matches(1, :))';
    im2_ftr_pts = keypoints2([2 1], matches(2, :))';
    
    % Calculate 3x3 homography matrix, H, mapping coordinates in image2
    % into coordinates in image1. Function calcHWithRANSAC currently uses all
    % pairs of matching feature points returned by the SIFT algorithm.
    % Modify the calcHWithRANSAC function to add code for implementing RANSAC.
    i
    H_list{i} = calcHWithRANSAC(im1_ftr_pts, im2_ftr_pts);
    i
end


% Compute new homographies H_map that map every other image *directly* to
% the reference image

H_map = {};
for i = 1  : length(H_list) + 1
    H = eye(3);
    for j = 2:i
        H = H * H_list{j-1};
    end
    H_map{i} = H;
end

% Compute the size of the output panorama image
min_row = 1;
min_col = 1;
max_row = 0;
max_col = 0;

% for each input image
for i=1:length(H_map)
    cur_image = imread(fullfile(imagepath, imagelist(i).name));
    [rows,cols,~] = size(cur_image);
    
    % create a matrix with the coordinates of the four corners of the
    % current image
    pt_matrix = cat(3, [1,1,1]', [1,cols,1]', [rows, 1,1]', [rows,cols,1]');
    
    % Map each of the 4 corner's coordinates into the coordinate system of
    % the reference image
    for j=1:4
        result = H_map{i}*pt_matrix(:,:,j);
        
        min_row = floor(min(min_row, result(1)));
        min_col = floor(min(min_col, result(2)));
        max_row = ceil(max(max_row, result(1)));
        max_col = ceil(max(max_col, result(2)));
    end
    
end

% Calculate output image size
panorama_height = max_row - min_row + 1;
panorama_width = max_col - min_col + 1;

% Calculate offset of the upper-left corner of the reference image relative
% to the upper-left corner of the output image
row_offset = 1 - min_row;
col_offset = 1 - min_col;

% Perform inverse mapping for each input image
for i=1:length(H_map)
    
    % Create a list of all pixels' coordinates in output image
    [x,y] = meshgrid(1:panorama_width, 1:panorama_height);
    % Create list of all row coordinates and column coordinates in separate
    % vectors, x and y, including offset
    x = reshape(x,1,[]) - col_offset;
    y = reshape(y,1,[]) - row_offset;
    
    % Create homogeneous coordinates for each pixel in output image
    pan_pts(1,:) = y;
    pan_pts(2,:) = x;
    pan_pts(3,:) = ones(1,size(pan_pts,2));
    
    % Perform inverse warp to compute coordinates in current input image
    image_coords = H_map{i}\pan_pts;
    row_coords = reshape(image_coords(1,:),panorama_height, panorama_width);
    col_coords = reshape(image_coords(2,:),panorama_height, panorama_width);
    
    cur_image = im2double(imread(fullfile(imagepath, imagelist(i).name)));
    
    % Bilinear interpolate color values
    curr_warped_image = zeros(panorama_height, panorama_width, 3);
    for channel = 1 : 3
        curr_warped_image(:, :, channel) = ...
            interp2(cur_image(:,:,channel), ...
            col_coords, row_coords, 'linear', 0);
    end
    
    % Add to output image. No blending done in this version; the current
    % image simply overwrites previous images where there is overlap.
    warped_images{i} = curr_warped_image;
    
end

% Blend the input images into a single panorama.

% Initialize output image to black (0)
panorama_image = zeros(panorama_height, panorama_width, 3);

% Modify the code below to blend warped images together via feathering. The
% following code adds warped images directly to panorama image. This is a
% very bad blending method - implement feathering instead.
%
% Save your final output image as a .jpg file and name it according to
% the directions in the assignment.

panorama_image = warped_images{1};

for i = 2 : length(warped_images)
    panorama_image = blend(warped_images{i},panorama_image);
end

imwrite(panorama_image,'output_images/my_out.jpg');
imshow(panorama_image);

cylindrical_image = zeros(int32(size(panorama_image,1)),int32(size(panorama_image,2)),3);
f = 100;

for i = 1:size(cylindrical_image,1)
    for j = 1:size(cylindrical_image,2)
        for z = 1:3
            [theta,rho,z] = cart2pol(j, i, z);
            x = f*tan(theta);
            y = i * sec(j/f);
            cylindrical_image(j,i,z) = panorama_image(x, y, z);
        end
    end
end
imshow(cylindrical_image);
