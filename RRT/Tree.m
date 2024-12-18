classdef Tree < handle
    % Represents a tree data structure. The implementation uses an array of
    % TreeNode instead of a structure linked using pointers, since MATLAB
    % does not have support for pointers.
    
    properties
        nodes; % array of TreeNode
        numNodes; % current number of nodes
        maxNodes; % maximum number of nodes supported by the tree
    end
    
    methods
        function self = Tree(maxNodes)
            % self = Tree(maxNodes) constructs a tree. The input maxNodes
            % defines the maximum number of nodes the tree will support.
            nodes(1:maxNodes) = TreeNode();
            self.nodes = nodes;
            self.numNodes = 0;
            self.maxNodes = maxNodes;
        end
        
        function reset(self)
            % reset(self) resets the tree (remove all nodes).
            self.numNodes = 0;
        end
        
        function insert(self, position, parentIndex)
            % insert(self, position, parentIndex) inserts a new node into
            % the tree. The node is represented by its position and the
            % index of its parent in the nodes array.
            self.numNodes = self.numNodes + 1;
            self.nodes(self.numNodes) = TreeNode();
            self.nodes(self.numNodes).position = position;
            self.nodes(self.numNodes).index = self.numNodes;
            self.nodes(self.numNodes).parentIndex = parentIndex;
        end
        
        function path = reconstructPath(self, goalIndex)
            % path = reconstructPath(self, goalIndex) reconstructs a path
            % from the root to a given node. The goalIndex is the index of
            % the given node (usually the goal of a search). The output
            % path is a 2 x p matrix representing a path of p positions.
            
            % The implementation here is different from the one suggested
            % in the theory since MATLAB does not have support for
            % pointers.
            index = goalIndex;
            pathSize = 0;
            path = zeros(2, self.maxNodes);
            while index ~= 0
                node = self.nodes(index);
                pathSize = pathSize + 1;
                path(:, pathSize) = node.position;
                index = node.parentIndex;
            end
            % Inverts the path since it was constructed from the goal to 
            % the root
            path = fliplr(path(:, 1:pathSize));
        end
        
        function nearest = findNearest(self, position)
            % nearest = findNearest(self, position) finds the nearest node
            % to position. The output nearest is a TreeNode.
            % implement
            indexNearest = 0;
            distanceNearest = inf;
            index = self.numNodes;
            while index ~= 0
                node = self.nodes(index);
                distance2 = (node.position(1) - position(1))^2 + (node.position(2) - position(2))^2;
                if distance2 < distanceNearest
                    indexNearest = index;
                    distanceNearest = distance2;
                end
                index = index - 1;
            end
            nearest = self.nodes(indexNearest);
        end
    end
end