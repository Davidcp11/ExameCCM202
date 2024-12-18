function monteCarloRRTStar(testCase)
% Runs a Monte Carlo to evaluate the RRT algorithm in a given test case.

rng(42);

% Maximum number of iterations during planning
maxIterations = 1000;

% Number of Monte Carlo runs
numRuns = 200;

% Limits for sampling
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

%% Running the Monte Carlo
rrt = RRTStar(obstacles, goalBias, delta, limits);

lengths = [];
numIterationss = [];
numSolved = 0;
costs = [];

for i=1:numRuns
    [path, length, numIterations, cost] = rrt.planPath(source, goal, maxIterations);
    if ~isempty(path)
        numSolved = numSolved + 1;
        lengths(end + 1) = length;
        costs(end + 1) = cost;
    end
    numIterationss(end + 1) = numIterations;
    fprintf('Runs: %d/%d\n', i, numRuns);
end

%% Printing statistics
fprintf('Percentage of solved instances: %g%%\n', 100 * numSolved / numRuns);
fprintf('Mean length of the found paths: %f\n', mean(lengths));
fprintf('Mean cost of the found paths: %f\n', mean(costs));
fprintf('Std of length of the found paths: %f\n', std(lengths));
fprintf('Mean of number of iterations until stopping: %g\n', mean(numIterationss));
fprintf('Std of number of iterations until stopping: %g\n', std(numIterationss));


end