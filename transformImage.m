function transformedImage = transformImage(img,A)
    tform = projective2d(inv(A)');
    transformedImage = imwarp(img,tform);
    %imshow(transformedImage)    
end