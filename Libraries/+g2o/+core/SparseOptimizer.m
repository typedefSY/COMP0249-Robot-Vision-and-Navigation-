% This class provides the full optimization support. The name is derived
% from the fact that it is designed to work with cost functions which have
% sparse patterns over variables. To use this class, you have to specify an
% optimization algorithm, which determines how the state variables are
% changed.

classdef SparseOptimizer < g2o.core.OptimizableGraph
    
    properties(Access = protected)
       
        % The optimization algorithm
        optimizationAlgorithm;
        
        % Use sparseinv?
        useSparseInv;
        
        % Estimated number of non-zero elements in the Hessian
        nonZeroElementsHXD;
        nonZeroElementsHXU;
        
        % Cached values
        HX;
        bX;
    end
    
    methods(Access = public)
       
        function obj = SparseOptimizer()
            
            obj = obj@g2o.core.OptimizableGraph();
            
            % If set to true, use the sparse inverse library
            obj.useSparseInv = (exist('sparseinv_mex', 'file') == 3);
        end
    end
    
    methods(Access = public, Sealed)
        
        % This function makes it possible to over-ride the sparse
        % optimizer. Sometimes this can cause matlab to crash due to a mex
        % issue.        
        function useSparseInvIfItExists(obj, useSparseInv)
            if (exist('sparseinv_mex', 'file') == 3)
                obj.useSparseInv = useSparseInv;
            else
                obj.useSparseInv = false;
            end
        end
       
        % Set the optimization algorithm which is used
        function setAlgorithm(obj, optimizationAlgorithm)
            
            assert(isa(optimizationAlgorithm, 'g2o.core.OptimizationAlgorithm') == true, ...
                'g2o:sparseoptimizer:algorithmwrongclass', ...
                ['The algorithm should inhert from g2o.OptimizationAlgorithm.' ...
                'The class is %s'], class(optimizationAlgorithm));
            
            obj.optimizationAlgorithm = optimizationAlgorithm;
            
            optimizationAlgorithm.setGraph(obj);
        end
        
        % Return the optimization algorithm
        function algorithm = algorithm(this)
            algorithm = this.optimizationAlgorithm;
        end
        
        % This method computes the marginals for the specified verties. By
        % marginals, we mean the covariance values. If no vertices are
        % specified, the mean and covariance for the entire graph is
        % computed.
        function [x, Px] = computeMarginals(obj, vertices)
            
            %%%% THIS HAS INCONSISTENT BEHAVIOUR:
            %% SPARSEINV INVERTS ALL NON-SPARSE BLOCKS BUT NOT ENTIRE MATRIX
            %% INV INVERTS WHOLE MATRIX
            %% SUBSET SELECTOR ONLY GIVES COVARIANCE MATRICES ON DIAGONALS
            
            % Check the graph is in a state where we can compute a value
            assert(obj.initializationRequired == false, ...
                'g2o:sparseoptimizer:initializationrequired', ...
                'Call initializeOptimization before extracting the marginals');

            % Special case - if the Hessian is empty, assume nowt
            if (isempty(obj.HX) == true)
                x = zeros(3, 1);
                Px = zeros(3, 3);
                return
            end
            
            % If a set of vertices wasn't specified, return the whole graph
            if (nargin == 1)
                x = obj.X;
                if (obj.useSparseInv == true)
                    Px = sparseinv(obj.HX);
                else 
                    Px = inv(obj.HX);
                end
                return
            end
            
            % If the input was a single vertex, wrap it in a cell to
            % simplify below
            if (iscell(vertices) == false)
                vertices = {vertices};
            end
            
            % Go through and figure out the dimension of the output
            nDims = 0;
            for v = 1 : length(vertices)
                nDims = nDims + vertices{v}.dimension();
            end
            
            % Preallocate
            x = zeros(nDims, 1);
            Px = sparse(nDims, nDims);
            
            % Compute the inverse (might have a smarter way to do it in the
            % future!)
            if (obj.useSparseInv == true)
                PX = sparseinv(obj.HX);
            else 
                PX = inv(obj.HX);
            end
            
            idx = 1;
            for v = 1 : length(vertices)
                d = vertices{v}.dimension();
                id = idx:(idx+d-1);
                if (vertices{v}.conditioned == true)
                    x(id) = vertices{v}.estimate();
                    Px(id, id) = zeros(d, d);
                else
                    x(id) = obj.X(vertices{v}.iX);
                    Px(id, id) = PX(vertices{v}.iX, vertices{v}.iX);
                end
                idx = idx + d;
            end
        end
        
        % This returns the computed information vector and information
        % matrix
        function [bX, HX] = computeHessian(obj)

            % Check the graph is in a state where we can compute a value
            assert(obj.initializationRequired == false, ...
                'g2o:sparseoptimizer:initializationrequired', ...
                'Call initializeOptimization before extracting the marginals');

            % Get the state and information matrix
            [HX, bX] = obj.computeHB(obj.X);
        end
    end
    
    methods(Access = protected)
        function numIterations = runOptimization(obj, maximumNumberOfIterations)
            
            assert(isempty(obj.optimizationAlgorithm) == false, ...
                'g2o:sparseoptimizer:optimizationalgorithmnotset', ...
                'The optimization algorithm is not set');
            
            [X, numIterations] = obj.optimizationAlgorithm.solve(obj.X, maximumNumberOfIterations);
            obj.X = X;
        end

    end
    
    % These are various methods which only the optimization algorithm uses.
    % They are used to compute various quantities needed in Algorithm 1 of
    % "A Tutorial on Graph-Based SLAM" by Grisetti et al.
    
    methods(Access = {?g2o.core.OptimizationAlgorithm,?g2o.sampling.RiemannianHamiltonianSampler})
        
        function [HX,bX] = computeHB(obj, X)
            
            % Assign the state value to all the vertices. This is a bit
            % inefficient, but it simplifies the API because the edge only
            % needs to take the current vertex estimate.
            obj.assignXToVertices(X);
            
            n = length(obj.X);
            
            % Construct the Hessians used to compute the update. To speed
            % up construction, we compute the "upper triangle" and the
            % diagonal sub-blocks separately. We assemble them together
            % afterwards.
            HXD = spalloc(n, n, obj.nonZeroElementsHXD);
            HXU = spalloc(n, n, obj.nonZeroElementsHXU);

            % Construct the correction vector. This isn't sparse, so just
            % allocate it as a normal array.
            if (nargout == 2)
                obj.bX = zeros(n, 1);
            else
                obj.bX = NaN(n, 1);
            end
                        
            % Iterate over all the edges and get the H and b values for
            % each edge. Assemble them into HX and bX for the entire graph.
            % Note that HX is symmetric and only the upper right blocks are
            % filled in by each edge. Therefore, we only fill the upper
            % right triangle and then copy this over right at the end.
            % Note that the idx is empty for fixed vertices, so we skip
            % them.
            edges = values(obj.edgesMap);            
            for e = 1 : length(edges)
                edge = edges{e};
                [H, b] = edge.computeHB();
                for i = 1 : length(edge.edgeVertices)
                    idx = edge.edgeVertices{i}.iX;
                    if (isempty(idx) == true)
                        continue;
                    end
                    if (nargout == 2)
                        obj.bX(idx) = obj.bX(idx) + b{i};
                    end
                    HXD(idx, idx) = HXD(idx, idx) + H{i, i};
                    for j = i + 1 : length(edge.edgeVertices)
                        jdx = edge.edgeVertices{j}.iX;
                        HXU(idx, jdx) = HXU(idx, jdx) + H{i, j};
                    end
                end
            end
            
            % Construct the full Hessian
            obj.HX = HXU + HXU' + HXD;
            
            if (nargout > 1)
                HX = obj.HX;
            end
            if (nargout == 2)
                bX = obj.bX;
            end
        end
                
        % Iterate through and set all the vertex states from the state
        % vector.
        function assignXToVertices(obj, X)
            
            % Check dimensions are the same
            assert(length(obj.X) == length(X), 'g2osparseoptimizer:assignxtovertices:xinconsistent', ...
                'The dimensions of X are inconsistent; length(this.X)=%d, length(X)=%d', ...
                length(obj.X), length(X));

            % Iterate over all the vertices and assign the values to the
            % state vectors. This is not very efficient, but it means that
            % the state is set up correctly for each vertex.
            for v = 1 : obj.numNonFixedVertices
                obj.nonFixedVertices{v}.x = X(obj.nonFixedVertices{v}.iX);
            end
            
            % Update our local store of X
            obj.X = X;
        end
        
        % Iterate through and set all vertex states from the state vector
        % and the perturbed matrix. This uses the oplus operator on each
        % vertex. XdX is the global state vector updated with the new
        % values.
        function XdX = assignXToVerticesWithPerturbation(obj, X, dX)
            
            % Check dimensions are the same
            assert(length(obj.X) == length(X), 'g2osparseoptimizer:assignxtovertices:xinconsistent', ...
                'The dimensions of X are inconsistent; length(this.X)=%d, length(X)=%d', ...
                length(obj.X), length(X));
            
            assert(length(obj.X) == length(dX), 'g2osparseoptimizer:assignxtovertices:xinconsistent', ...
                'The dimensions of X and dX are inconsistent; length(this.X)=%d, length(dX)=%d', ...
                length(obj.X), length(dX));
            
            % Cheesy way to initialise the output vector to the right size
            XdX = X * 0;
            
            % Iterate over all the vertices and assign the values to the
            % state vectors. This is not very efficient, but it means that
            % the oplus-added state values are available for the optimizer
            % to use in its next step.
            for v = 1 : obj.numNonFixedVertices
                obj.nonFixedVertices{v}.x = X(obj.nonFixedVertices{v}.iX);
                obj.nonFixedVertices{v}.oplus(dX(obj.nonFixedVertices{v}.iX));
                XdX(obj.nonFixedVertices{v}.iX) = obj.nonFixedVertices{v}.x;
            end
        end
    end
    
    methods(Access = protected)
        
        function buildStructure(obj)
            
            % Call the base class constructor
            buildStructure@g2o.core.OptimizableGraph(obj);
            
            % Now go through and estimate the number of non-zero elements
            % in the Hessian for preallocation of the sparse arrays.
            nzd = 0;
            nzu = 0;
            
            edges = values(obj.edgesMap);            
            for e = 1 : length(edges)
                edge = edges{e};
                for i = 1 : length(edge.edgeVertices)
                    idx = edge.edgeVertices{i}.iX;
                    if (isempty(idx) == true)
                        continue;
                    end
                    nzd = nzd + length(idx) * length(idx);
                    for j = i + 1 : length(edge.edgeVertices)
                        jdx = edge.edgeVertices{j}.iX;
                        nzu = nzu + length(idx) * length(jdx);
                    end
                end
            end
            % Estimated statistics; actually not very accurate
            %fprintf('Estimated number of non zero elements (d=%d,u=%d)\n', nzd, nzu);
            %fprintf('Estimated density %3.2f%%\n', 100 * (nzd + 2 * nzu) / length(this.X)^2);
            obj.nonZeroElementsHXD = nzd;
            obj.nonZeroElementsHXU = nzu;
        end            
    end
    
end
