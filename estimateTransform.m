function A = estimateTransform(img1_points,img2_points)
    %% Initialization
    ncorr = length(img1_points);
    %% Setting the P matrix 10x9
    P = [img1_points(1:ncorr,:),ones(ncorr,1),zeros(ncorr,3),...
        -1*img2_points(1:ncorr,1).*img1_points(1:ncorr,:),...
        -1*img2_points(1:ncorr,1);...
        zeros(ncorr,3),-1*img1_points(1:ncorr,:),-1*ones(ncorr,1),...
        img2_points(1:ncorr,2).*img1_points(1:ncorr,:),...
        img2_points(1:ncorr,2)];
    %% Calculating r 8x1
    r = img2_points(:);
    %% SVD Decomposition
    [~,S,V] = svd(P);
    %% Extracting Diagonal elements of S
    sigmas = diag(S);
    %% Detecting minimum diagonal element
    if length(sigmas) >= 9
        minSigma = min(sigmas);
        [~,minSigmaCol] = find(S==minSigma);
        %% Calculating q
        q = double(vpa(V(:,minSigmaCol)));
    elseif length(sigmas)<9
        %% Calculating q
        q = double(vpa(V(:,9)));
    end
    %% Calculating A
    A = reshape(q,[3,3])';
end