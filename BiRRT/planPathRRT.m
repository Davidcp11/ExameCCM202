function planPathRRT(testCase)
% Plans a path for a given test case and shows an animation.

% figFormat = 'png'; % for Word users
figFormat = 'eps'; % for LaTeX users

% Limits for the search space
limits.xMin = 0.0;
limits.xMax = 10.0;
limits.yMin = 0.0;
limits.yMax = 10.0;

goalBias = 0.1;
delta = 0.2;

%% Defining the test case
if testCase == 'a'
    source = [1.0; 1.0];
    goal = [9.0; 9.0];
    obstacles(1) = Obstacle([5.0; 5.0], 1.0);
elseif testCase == 'b'
    source = [1.0; 9.0];
    goal = [9.0; 1.0];
    obstacles(1) = Obstacle([5.0; 5.0], 1.0);
    obstacles(2) = Obstacle([3.0; 3.0], 1.0);
    obstacles(3) = Obstacle([7.0; 7.0], 1.0);
elseif testCase == 'c'
    source = [1.0; 9.0];
    goal = [9.0; 1.0];
    obstacles(1) = Obstacle([4.0; 4.0], 1.0);
    obstacles(2) = Obstacle([4.0; 6.0], 1.0);
    obstacles(3) = Obstacle([6.0; 4.0], 1.0);
    obstacles(4) = Obstacle([6.0; 6.0], 1.0);
elseif testCase == 'd'
    source = [9.0; 9.0];
    goal = [9.0; 1.0];
    obstacles(1) = Obstacle([1.0; 5.0], 0.8);
    obstacles(2) = Obstacle([3.0; 5.0], 1.05);
    obstacles(3) = Obstacle([5.0; 5.0], 1.05);
    obstacles(4) = Obstacle([7.0; 5.0], 1.05);
    obstacles(5) = Obstacle([9.0; 5.0], 1.05);
end
%% Plan the path
rrt = BiRRT(obstacles, goalBias, delta, limits);

maxIterations = 1000;
path = rrt.planPath(source, goal, maxIterations);

%% Plot the scenario
figure;
for i=1:length(obstacles)
    circle(obstacles(i).center(1), obstacles(i).center(2), obstacles(i).radius, [0.5, 0.5, 0.5]);
end
xlabel('$x$ ($m$)', 'FontSize', 14, 'interpreter', 'latex');
ylabel('$y$ ($m$)', 'FontSize', 14, 'interpreter', 'latex');
set(gca, 'FontSize', 14, 'TickLabelInterpreter', 'latex');
xlim([limits.xMin, limits.xMax]);
ylim([limits.yMin, limits.yMax]);
axis square;
hold on;

plot(goal(1), goal(2), 'xr');

%% Plot the tree nodes
i=1;
j=1;
while i <= rrt.tree1.numNodes || j <= rrt.tree2.numNodes
    if i <= rrt.tree1.numNodes
        node = rrt.tree1.nodes(i);
        plot(node.position(1), node.position(2), '.b');
        if node.parentIndex ~= 0
            parent = rrt.tree1.nodes(node.parentIndex);
            plot([node.position(1), parent.position(1)],...
                [node.position(2), parent.position(2)], 'b');
        end
        i=i+1;
        pause(0.01);
    end
    if j <= rrt.tree2.numNodes
        node = rrt.tree2.nodes(j);
        plot(node.position(1), node.position(2), '.b');
        if node.parentIndex ~= 0
            parent = rrt.tree2.nodes(node.parentIndex);
            plot([node.position(1), parent.position(1)],...
                [node.position(2), parent.position(2)], 'b');
        end
        j=j+1;
        pause(0.01);
    end
end

%% Plot the path
if ~isempty(path)
    plot(path(1, :), path(2, :), 'r');
end

saveFig('rrt_planning', figFormat);

end