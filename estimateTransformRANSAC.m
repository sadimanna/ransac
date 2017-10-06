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
    nmatchedInliers = 0;
    maxMatchedInliers = 0;
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
        logic1Inlier = errorForward<distThreshold;
        logic2Inlier = errorBackward<distThreshold;
        %nInliers1 = nnz(logic1Inlier);
        %nInliers2 = nnz(logic2Inlier);
        im1Inliers = logic1Inlier.*img1_points;
        im2Inliers = logic2Inlier.*img2_points;
        matchedInliers = im1Inliers(:,1)>0 & im2Inliers(:,1)>0; 
        nMatchedInliers = nnz(matchedInliers); %number of matched inliers
        %% Determining the Inliers
        im1MatchedInliers = im1Inliers(matchedInliers,:); %inliers in Image 1
        im2MatchedInliers = im2Inliers(matchedInliers,:); %inliers in image 2
        %% Updating Prarameters
        if nMatchedInliers > maxMatchedInliers
            e = (1-nMatchedInliers/N_pts);
            N_iter = round(log10(1-p)/log10(1-(1-e)^k));
            im1InlierCorrPts = im1MatchedInliers;
            im2InlierCorrPts = im2MatchedInliers;
            maxMatchedInliers = nMatchedInliers
            A_ransac_best = A;
        end 
    end
    %% Final Homography Estimation
    A_ransac = estimateTransform(im1InlierCorrPts,im2InlierCorrPts);
end
