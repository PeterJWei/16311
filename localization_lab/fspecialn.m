function [mask] = fspecialn(type, fsize, varargin)
% Produces 3-dimensional masks. Currently only makes Gaussians
%
% Usage:
%   mask = fspecialn('gaussian', fsize, m, c) - Returns a filter of
%       size fsize with covariance c

if strcmpi(type, 'gaussian')
    mask = MakeGaussian(fsize, varargin);
end

    function [mask] = MakeGaussian(fsize, args)
        c = args{1};
        mid = (fsize + 1)/2;
        
        [x,y,z] = meshgrid(1:fsize(1), 1:fsize(2), 1:fsize(3));
        X = [x(:), y(:), z(:)];
        probs = mvnpdf(X, mid(:)', c);
        mask = reshape(probs, fsize);
    end

end