function [ iB2 ] = thresholding( filename )
%threshold the image to try to identify the pixels that make up the tennis
%balls in the image
    close all
    iB2 = imread(filename);

    h = fspecial('motion', 50, 45); %perform motion blur
    iB2 = imfilter(iB2, h);
    iB2 = rgb2hsv(iB2); %convert to hsv for better thresholding

    width = size(iB2, 1);
    height = size(iB2, 2);
    
    iB2(:,:,3) = (iB2(:,:,1) > .12) & (iB2(:,:,1) < .19) & (iB2(:,:,2) > .3);
    iB2(:,:,1) = 0;
    iB2(:,:,2) = 0;
    %for i = 1:width
    %    for j = 1:height
    %        if iB2(i,j,1) < .12 || iB2(i,j,1) > .19 || iB2(i,j,2) < .3
    %            %threshold at values found by analysis of the hsv image
    %            iB2(i,j,:) = 0;
    %        else
    %            iB2(i,j,1) = 0;
    %            iB2(i,j,2) = 0;
    %            iB2(i,j,3) = 1; %hsv = 0,0,1 -> rgb = white
    %        end
    %    end
    %end
    iB2 = hsv2rgb(iB2);
    
    
    se = strel('line',11,0);
    se2 = strel('line',11,90); %eroding and dilating masks to remove excess
                               %noise
    iB2 = imerode(iB2,se);
    iB2 = imerode(iB2,se2);
    iB2 = imerode(iB2,se);
    iB2 = imerode(iB2,se2);
    iB2 = imdilate(iB2,se);
    iB2 = imdilate(iB2, se2);
    iB2 = imdilate(iB2,se);
    iB2 = imdilate(iB2, se2);
    %perform erosion and dilation twice

end