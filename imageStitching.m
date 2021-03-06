%% Taking Photographs
img1 = im2double(rgb2gray(imread(path_to_image1)));
img2 = im2double(rgb2gray(imread(path_to_image2)));

%% Obtaininig Correspondences
points1 = detectSURFFeatures(img1);
points2 = detectSURFFeatures(img2);

features1 = extractFeatures(img1,points1);
features2 = extractFeatures(img2,points2);

indexPairs = matchFeatures(features1,features2,'Unique',true);

matchedPoints1 = points1(indexPairs(:,1));
matchedPoints2 = points2(indexPairs(:,2));

img1_points = matchedPoints1.Location;
img2_points = matchedPoints2.Location;

%% Estimating Homography using RANSAC
A_ransac = estimateTransformRANSAC(img1_points,img2_points)

%% Applying Homography Transformation Matrix directly to RGB Image
img2rgb = im2double(imread(path_to_image2));
im2_transformed = transformImage(img2rgb,A_ransac);
%imshow(im2_transformed)

%% Expanding the image1 to match size of image2
img1rgb = im2double(imread('office1.jpg'));
im1_size = size(img1rgb);
im2_transformed_size = size(im2_transformed);
padsize = im2_transformed_size-im1_size;
img1_expanded = padarray(img1rgb,padsize,'post');
%figure
%imshow(img1_expanded)

%% Blending images with RAMP
[x_overlap,y_overlap]=ginput(2); %Taking user input for taking two points for starting and ending of ramp
overlapleft = round(x_overlap(1));
overlapright = round(x_overlap(2));
diff = overlapright-overlapleft;
ramp = [zeros(1,overlapleft-1),0:1/diff:1,ones(1,im2_transformed_size(2)-overlapright)];
im2_blend = im2_transformed .* repmat(ramp,im2_transformed_size(1),1);
%figure
%imshow(im2_blend)

ramp_for_im1 = 1 - ramp;
im1_blend = img1_expanded .* repmat(ramp_for_im1,im2_transformed_size(1),1);
%figure
%imshow(im1_blend)

%% Creating the panorama
impanorama = im1_blend+im2_blend;
figure
imshow(impanorama)
