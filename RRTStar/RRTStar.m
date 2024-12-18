classdef RRTStar < handle
    % Represents the Rapidly-Exploring Random Tree (RRT) algorithm.
    properties
        tree; % the tree constructed during path planning
        obstacles; % the obstacles
        goalBias; % the goal bias probability
        delta; % the delta used in the extend operation
        limits; % the bounds of the search space
    end
    
    methods
        function self = RRTStar(obstacles, goalBias, delta, limits)
            % self = RRT(obstacles, goalBias, delta, limits) constructs the
            % Rapidly-Exploring Random Tree (RRT) algorithm. The inputs
            % are:
            % obstacles: the obstacles (array of Obstacle).
            % goalBias: goal bias probability.
            % delta: the delta used in the extend operation.
            % limits: the limits of the search space. It is a struct with 
            %         the fields xMin, xMax, yMin, and yMax representing
            %         the minimum x coordinate, maximum x coordinate,
            %         minimum y coordinate, and maximum y coordinate,
            %         respectively.
            self.tree = Tree(10000);
            self.obstacles = obstacles;
            self.goalBias = goalBias;
            self.delta = delta;
            self.limits = limits;
        end
        
        function collisionFree = checkCollisionFree(self, position)
            % collisionFree = checkCollisionFree(self, position) checks if
            % a given position is collision free. The output collisionFree
            % is a boolean.
            
            % Implement
            sizeObs = size(self.obstacles);
            for i = 1:sizeObs(2)
                collisionFree = ~self.obstacles(i).checkCollision(position);
                if ~collisionFree
                    break;
                end
            end

        end
        
        function sample = sampleFreeSpace(self)
            % sample = sampleFreeSpace(self) samples from the free space
            % using reject sampling, i.e. the algorithm keeps on sampling
            % new points until the sampled point belongs to the free space.
            % The output sample is a 2x1 column vector.
            
            % Implement
            freeSpace = false;
            while freeSpace == false
                x = self.limits.xMin + rand() * (self.limits.xMax - self.limits.xMin);
                y = self.limits.yMin + rand() * (self.limits.yMax - self.limits.yMin);
                freeSpace = self.checkCollisionFree([x, y]);
            end
            sample = [x ; y];
        end
        
        function [path, length, numIterations, cost] = planPath(self, source, goal, maxIterations)
            % [path, length, numIterations] = planPath(self, source, goal, 
            % maxIterations) plans a path from the source to the goal. The 
            % variable maxIterations represents the maximum number of 
            %^iterations for the RRT algorithm. The outputs path and length
            % represent the planned path (as a 2 x p matrix representing 
            % a path of p positions) and the path length, respectively.
            % Finally, numItertations is the number of iterations the
            % algorithm ran until stopping.
            self.tree.reset(); % to clear the tree
            % 0 is used here to represent null, i.e. to represent that the 
            % root does not have a parent
            self.tree.insert(source, 0, 0);
            eps = 10^(-3);
            findGoal = false;
            goalIndex = 0;
            for i=1:maxIterations
                % Implement RRT iteration
                numIterations = i;
                if rand() < self.goalBias
                    random_node_position = goal;
                else
                    random_node_position = self.sampleFreeSpace();
                end
                nearest = self.tree.findNearest(random_node_position);
                new_node_position = nearest.extend(random_node_position, self.delta);
                %new_node = self.tree.nodes(self.tree.numNodes);
                if self.checkCollisionFree(new_node_position)
                    [parentIndex, parentCost] = self.tree.select_parent(new_node_position, nearest, 3*self.delta);
                    self.tree.insert(new_node_position, parentIndex, parentCost);
                    new_node = self.tree.nodes(self.tree.numNodes);
                    self.tree.rewire(new_node, 3*self.delta);
                    if norm(new_node_position - goal) < eps 
                        %[path, cost] = self.tree.reconstructPath(self.tree.nodes(self.tree.numNodes).index);
                        if ~findGoal
                            goalIndex = self.tree.nodes(self.tree.numNodes).index;
                            goal_node = self.tree.nodes(goalIndex);
                            costGoal = self.tree.costNode(goal_node);
                        else
                            index = self.tree.nodes(self.tree.numNodes).index;
                            node = self.tree.nodes(index);
                            costNode = self.tree.costNode(node);
                            if costNode < costGoal
                                goalIndex = index;
                            end
                        end
                        %pathSize = size(path);
                        %length = pathSize(2);
                        findGoal = true;
                    end
                end
                if findGoal
                    goal_node = self.tree.nodes(goalIndex);
                    cost = self.tree.costNode(goal_node);
                    
                end
                % Break the loop if the goal has reached
            end
            self.tree.nodes(goalIndex).cost = cost;
            [path, cost] = self.tree.reconstructPath(goalIndex);
            pathSize = size(path);
            length = pathSize(2);
            % If the path was not found, return an empty path and an
            % infinite length (use MATLAB's Inf for infinite)
            % Implement
            if numIterations == maxIterations & ~findGoal
                path = [];
                length = inf;
            end
        end
        
        function [path, length] = reconstructPath(self)
            % [path, length] = reconstructPath(self) reconstructs the path
            % planned by the RRT. The outputs path and length are the
            % planned path ()
            path = self.tree.reconstructPath(self.tree.numNodes);
            length = 0.0;
            for i=2:size(path, 2)
                length = length + norm(path(:, i) - path(:, i - 1));
            end
        end
    end
end