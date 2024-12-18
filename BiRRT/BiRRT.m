classdef BiRRT < handle
    % Represents the Rapidly-Exploring Random Tree (RRT) algorithm.
    properties
        tree1; % the tree constructed during path planning
        tree2;
        obstacles; % the obstacles
        goalBias; % the goal bias probability
        delta; % the delta used in the extend operation
        limits; % the bounds of the search space
    end
    
    methods
        function self = BiRRT(obstacles, goalBias, delta, limits)
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
            self.tree1 = Tree(10000);
            self.tree2 = Tree(10000);
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
            % [path, length, numIterations, cost] = planPath(self, source, goal, 
            % maxIterations) plans a path from the source to the goal. The 
            % variable maxIterations represents the maximum number of 
            %^iterations for the Bi-RRT algorithm. The outputs path and length
            % represent the planned path (as a 2 x p matrix representing 
            % a path of p positions) and the path length, respectively.
            % Finally, numItertations is the number of iterations the
            % algorithm ran until stopping and cost is the euclidean distance of the path.
            self.tree1.reset(); % to clear the tree
            self.tree2.reset();
            % 0 is used here to represent null, i.e. to represent that the 
            % root does not have a parent
            self.tree1.insert(source, 0, 1);
            self.tree2.insert(goal, 0, 2);
            eps = 10^(-3);
            findGoal1 = false;
            findGoal2 = false;
            contact = false;
            team1Index = 0;
            team2Index = 0;
            for i=1:maxIterations
                % Implement RRT iteration
                numIterations = i;
                if mod(i,2) == 0
                    if rand() < self.goalBias
                        random_node = goal;
                    else
                        random_node = self.sampleFreeSpace();
                    end
                    nearest = self.tree1.findNearest(random_node);
                    new_node = nearest.extend(random_node, self.delta);
                    if self.checkCollisionFree(new_node)
                        self.tree1.insert(new_node, nearest.index, 1);
                        team1Index = self.tree1.numNodes;
                        [contact, enemyIndex] = self.contactTeam(new_node, 1);
                        if norm(new_node - goal) < eps                        
                            findGoal1 = true;
                            break
                        end
                        
                        if contact
                            team2Index = enemyIndex;
                            break
                        end
                    end
                else
                    if rand() < self.goalBias
                        random_node = source;
                    else
                        random_node = self.sampleFreeSpace();
                    end
                    nearest = self.tree2.findNearest(random_node);
                    new_node = nearest.extend(random_node, self.delta);
                    if self.checkCollisionFree(new_node)
                        self.tree2.insert(new_node, nearest.index, 2);
                        team2Index = self.tree2.numNodes;
                        [contact, enemyIndex] = self.contactTeam(new_node, 2);
                        
                        if norm(new_node - source) < eps
                            findGoal2 = true;
                            break;
                        end
                        if contact
                            team1Index = enemyIndex;
                            break
                        end
                    end
                end
                % Break the loop if the goal has reached
            end
            % If the path was not found, return an empty path and an
            % infinite length (use MATLAB's Inf for infinite)
            % Implement
            path1 = self.tree1.reconstructPath(team1Index);
            pathSize1 = size(path1);
            length1 = pathSize1(2);

            path2 = self.tree2.reconstructPath(team2Index);
            pathSize2 = size(path2);
            length2 = pathSize2(2);
            if contact
                path = [path1, fliplr(path2(:, 1:pathSize2(2)))];
                length = length1 + length2;
                cost = self.cost(path);
            end

            if numIterations == maxIterations & ~findGoal1 & ~findGoal2
                path = [];
                length = inf;
                cost = inf;
            elseif findGoal1 & ~findGoal2
                path = path1;
                length = length1;
                cost = self.cost(path);
            elseif ~findGoal1 & findGoal2
                path = path2;
                length = length2;
                cost = self.cost(path);
            end
        end
        
        function [path, length] = reconstructPath(self)
            % [path, length] = reconstructPath(self) reconstructs the path
            % planned by the RRT. The outputs path and length are the
            % planned path ()
            path1 = self.tree1.reconstructPath(self.tree1.numNodes);
            path2 = self.tree2.reconstructPath(self.tree2.numNodes);
            path2 = fliplr(path2(:, 1:end));
            path = [path1, flip(path2)];
            length = 0.0;
            for i=2:size(path, 2)
                length = length + norm(path(:, i) - path(:, i - 1));
            end
        end

        function [find, contactIndexEnemy] = contactTeam(self, node_position, team)
            % [find, contactIndexEnemy] = contactTeam(self, node_position, team)
            % This function determines if a node in the specified team is
            % within a certain distance (delta) from the given node_position.
            % If such a node is found, the function returns true for the find
            % flag and provides the index of the nearest enemy node in contactIndexEnemy.
            % The function considers two teams (team 1 and team 2) represented
            % by self.tree2 and self.tree1 respectively.
            find = false;
            contactIndexEnemy = 0;
            if team == 1    
                for i = 1:self.tree2.numNodes
                    nodeEnemy = self.tree2.nodes(i);
                    distance = norm(nodeEnemy.position - node_position);
                    if distance < self.delta
                        find = true;
                        contactIndexEnemy = nodeEnemy.index;
                        break
                    end
                end
            else
                for i = 1:self.tree1.numNodes
                    nodeEnemy = self.tree1.nodes(i);
                    distance = norm(nodeEnemy.position - node_position);
                    if distance < self.delta
                        find = true;
                        contactIndexEnemy = nodeEnemy.index;
                        break
                    end
                end
            end
        end

        function cost = cost(self, path)
            % cost = cost(self, path). The cost is the euclidean distance
            % of the path.
            cost = 0;
            pathSize = size(path);
            for i=1:pathSize(2)-1
                cost = cost+norm(path(:,i)-path(:,i+1));
            end
        end
    end
end