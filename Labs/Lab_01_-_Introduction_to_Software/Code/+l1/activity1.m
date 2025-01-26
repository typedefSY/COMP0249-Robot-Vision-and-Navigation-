% This script shows how to set up the individual pieces of the code and
% start running them

import ebe.core.*;
import ebe.graphics.*;
import l1.pointbot.*;

% Find, load and parse the configuration file
config = ebe.utils.readJSONFile('config/activity1.json');

% Set up the simulator and start it
simulator = Simulator(config);
simulator.start();

% Set up the figure in which we draw everything
fig = FigureManager.getFigure("Simulator Output");
clf
hold on
axis([-100 100 -100 100])
axis square

% Set up the view which show the output of the simulator and start it
simulatorViewer = ebe.graphics.ViewManager(config);
simulatorViewer.addView(SimulatorView(config, simulator));
simulatorViewer.start();

% This is the main loop - while the simulator says we should keep going, we
% step the simulator, extract the events which were generated, and
% visualize them.
while (simulator.keepRunning()  == true)
    simulator.step();
    events = simulator.events();
    simulatorViewer.visualize(events);
    drawnow
end

[TSimulator, XTrueHistory] = simulator.history();

xPositions = XTrueHistory(1, :);
yPositions = XTrueHistory(3, :);
xVelocities = XTrueHistory(2, :);
yVelocities = XTrueHistory(4, :);

xMin = min(xPositions);
xMax = max(xPositions);
yMin = min(yPositions);
yMax = max(yPositions);
maxVelocity = max(sqrt(xVelocities.^2 + yVelocities.^2));

% Plot the trajectory of the robot
plot(xPositions, yPositions, 'b-')

disp(['maxVelocity: ', num2str(maxVelocity)]);
disp(['xMin: ', num2str(xMin)]);
disp(['xMax: ', num2str(xMax)]);
disp(['yMin: ', num2str(yMin)]);
disp(['yMax: ', num2str(yMax)]);
