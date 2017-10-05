# ransac
Applying RANSAC for determining Homography Transformation Matrix for Image Stitching

*Main File* =>            

**imageStiching.m** -> main function for stitching two images to get a panorama

*Supporting Functions* => 

**estimateTransformRANSAC.m** -> Estimating the Homogrpahy Matrix using RANSAC

**estimateTransform.m** -> Estimating the Homography Matrix

**transformForward.m** -> Applying Homography Matrix to matched points in Image 1

**transformbackward.m** -> Applying Inverse Homography Matrix to matched points in Image 2

**transformImage.m** -> Applying inverse Homography matrix to Image 2
