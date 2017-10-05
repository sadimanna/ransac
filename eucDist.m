function dist = eucDist(im1f,im2f)
    dist = sum((im1f-im2f).^2)^0.5;
end