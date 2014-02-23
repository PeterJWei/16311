function [angles, d] = vision( filename)
%header file that runs the processing pipeline. Threshold, segment,
%analyze, and then find distance to object.

colors = [1 0 0; 0 1 0; 0 0 1; 1 .5 0; .5 1 0; 1 0 .5; .5 0 1; 0 1 .5; 0 .5 1];
close all
im = thresholding(filename);
%[count, im] = segmentation(colors, im);
[count, im] = segmentor(colors, im);

[im, numpix, averagex, averagey] = analysis(count-1, colors, im);

[angles, d] = disttoball6(im, numpix, averagex, averagey);
angles
d
end