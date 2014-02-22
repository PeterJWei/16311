classdef DiscreteBayesFilter < handle
    % DiscreteBayesFilter is a gridworld recursive Bayes filter
    % transitionFunc should be a handle tF(map, displacement)
    % observationFunc should be a handle oF(map, landmarks, observation)
    %
    % Written by Humphrey Hu (humhu@cmu.edu) for CMU's 16-311
    % v0.1 2/21/2014
    
    properties
        probabilities;      % The probability mass values
        transitionFunc;     % A handle to the transition function
        observationFunc;    % A handle to the observation function
    end
    
    properties(Access = private)
        xWeights;   % Weights for quickly calculating the filter mean
        yWeights;
        tWeights;
    end
    
    methods
        
        % Create a DBF with specified dimensions and optional initial
        % distribution
        %
        % Usage:
        % DiscreteBayesFilter(dims, tFunc, oFunc) - where dims is a vector
        %   of the map dimensions in cells, tFunc is a handle to the transition function,
        %   oFunc is a handle to the observation function, initializes the
        %   map to be uniform
        % DiscreteBayesFilter(dims, tFunc, oFunc, initMap) - Initializes
        %   the probability map to the given map.
        function [obj] = DiscreteBayesFilter(varargin)
            
            if nargin >= 3
                dims =  varargin{1};
                obj.transitionFunc = varargin{2};
                obj.observationFunc = varargin{3};
                obj.probabilities = ones(dims);
            end
            if nargin >= 4
                obj.probabilities = varargin{4};
            end
            
            obj.Normalize();
            obj.InitializeWeights();
            
        end
        
        % Applies a smoothing mask, or uses Laplace (additive) smoothing
        %
        % Usage:
        %   dbf.Smooth('Laplace', alpha) - adds alpha to each cell and
        %       normalizes
        %   dbf.Smooth('Mask', mask) - Convolves the probability matrix
        %       with N-dimensional mask, where N is the dimension of the
        %       probability matrix
        function Smooth(obj, mode, varargin)
            
            if strcmpi(mode, 'Mask')
                mask = varargin{1};
                obj.probabilities = convn(obj.probabilities, mask, 'same');
            elseif strcmpi(mode, 'Laplace')
                alpha = varargin{1};
                obj.probabilities = obj.probabilities + alpha;
            end
            
            obj.Normalize();
            
        end
        
        % Returns the mean estimate in indices. You need to convert this
        % back into (x,y,theta).
        function [m] = GetMean(obj)
            xM = obj.probabilities.*obj.xWeights;
            yM = obj.probabilities.*obj.yWeights;
            tM = obj.probabilities.*obj.tWeights;
            m = [sum(xM(:)); sum(yM(:)); sum(tM(:))];
        end
        
        % Apply the transition function to the filter.
        %
        % Usage:
        % dbf.TransitionUpdate(displacement) - Updates the probabilities
        %   using a measured displacement.
        function TransitionUpdate(obj, displacement)
            obj.probabilities = obj.transitionFunc(obj.probabilities, displacement);
        end
        
        % Apply the observation function to the filter.
        %
        % Usage:
        % dbf.ObservationUpdate(map, measurement) - Updates the
        %   probabilities using a measurement to a tennis ball.
        function ObservationUpdate(obj, map, measurement)
            obj.probabilities = obj.observationFunc(map, measurement, obj.probabilities);
        end
        
        % Plots the first three dimensions of the probability map in slices
        % Accepts an axis handle if needed.
        function [ah] = Visualize(obj, zScale, ah)
            if nargin < 3
                ah = axes;
            end
            if nargin < 2
                zScale = 1.0;
            end
            
            hold(ah, 'on');
            axis(ah, 'equal');
            caxis(ah, [0,1]);
            X = size(obj.probabilities, 1);
            Y = size(obj.probabilities, 2);
            T = size(obj.probabilities, 3);
            
            xSlice = obj.xWeights(:,:,1);
            ySlice = obj.yWeights(:,:,1);
            zSlice = ones(X, Y);
            znames = {};
            for k = 1:T
                surf(ah, xSlice, ySlice, zScale*k*zSlice, obj.probabilities(:,:,k), ...
                    'EdgeColor', 'k');
                znames{k} = num2str(k);
            end
            colorbar;
            view(45, 45);
            xlabel('x');
            ylabel('y');
            zlabel('z');
            set(ah, 'ZTickLabel', znames);
            hold(ah, 'off');
        end
        
    end
    
    methods(Access=private)
        
        function InitializeWeights(obj)
            xgv = 1:size(obj.probabilities,1);
            ygv = 1:size(obj.probabilities,2);
            tgv = 1:size(obj.probabilities,3);
            [obj.xWeights, obj.yWeights, obj.tWeights] = meshgrid(xgv, ygv, tgv);
        end
        
        
        % Normalizes the filter probabilities.
        function Normalize(obj)
            total = sum(obj.probabilities(:));
            obj.probabilities = obj.probabilities/total;
        end
    end
    
end