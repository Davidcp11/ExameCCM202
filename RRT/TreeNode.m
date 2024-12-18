classdef TreeNode < handle
    % Represents a tree node.
    properties
        position; % the tree node's position
        index; % the index in the array of nodes
        parentIndex; % the index of the parent of this tree node
    end
    
    methods
        function self = TreeNode()
            % self = TreeNode() constructs a tree node.
            self.position = [];
            self.index = -1;
            self.parentIndex = -1;
        end
        
        function newPosition = extend(self, position, delta)
            % newPosition = extend(self, position, delta) extends the
            % position of this node in the direction of the input position.
            % The extension is limited by delta. The output newPosition is 
            % the result of the extension.
            
            % Implement
            distance = norm(position-self.position);
            if distance > delta
                newPosition = self.position + delta * (position - self.position)/distance;
            else
                newPosition = position;
            end
        end
    end
end