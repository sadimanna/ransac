function A_ransac = estimateTransformRANSAC(img1_points,img2_points)
    %% Intialization
    N_pts = length(img1_points); % total no. of points
    k = 5; %no. of correspondences
    e = 0.5; %outlier ratio = no. of outliers/total no. points
             %              = 1- no. of inliers/total no. of points
    p = 0.99; %probability that a point is an inlier
    N_iter = round(log10(1-p)/log10(1-(1-e)^k)); % no. of iterations
    %distThreshold = sqrt(5.99*sigma) %sigma = expected uncertainty
    %inlierThreshold = 0
    %% Determination of Inliers
    im1InlierCorrPts = [];
    im2InlierCorrPts = [];
    for i = 1:N_iter
        %% Random sample
        randIndexes = randi(N_pts,5,1);
        im1pts = img1_points(randIndexes,:);
        im2pts = img2_points(randIndexes,:);
        %% Homography Matrix estimation
        A = estimateTransform(im1pts,im2pts);
        %% Geometric Error Calculation
        %% Forward Transformation Error
        im1ptsForward = transformForward(img1_points,A);
        errorForward = sum((im1ptsForward-img2_points).^2,2).^0.5;
        totalForwardError = sum(errorForward);
        %% Backward Transformation Error
        im2ptsBackward = transformBackward(img2_points,A);
        errorBackward = sum((im2ptsBackward-img1_points).^2,2).^0.5;
        totalBackwardError = sum(errorBackward);
        %% Total Geometric Error
        totalError = totalForwardError + totalBackwardError;
        %% Expected Error Distribution Std Dev
        sigma = sqrt(totalError/(2*N_pts));
        %% Determining Threshold
        distThreshold = sqrt(5.99)*sigma;
        %% Determining no. of inliers
        nInliers = nnz(errorForward<distThreshold);
        %% Updating Prarameters
        if nInliers > (1-e)*N_pts
            e = 1-(nInliers/N_pts);
            N_iter = round(log10(1-p)/log10(1-(1-e)^k));
            im1InlierCorrPts = im1pts;
            im2InlierCorrPts = im2pts;
        end    
    end
    %% Final Homography Estimation
    A_ransac = estimateTransform(im1InlierCorrPts,im2InlierCorrPts);
end
