% This task runs the simulator for the SLAM system

import ebe.core.*;
import ebe.graphics.*;
import l2.*;

% Find, load and parse the configuration file
config = ebe.utils.readJSONFile('config/activity0.json');

% Create the mainloop object, which manages everything
mainLoop = ebe.MainLoop(config);

% Create the simulator and register it
simulator = dotbot.Simulator(config);
mainLoop.setEventGenerator(simulator);

% Create the store for estimates
resultsAccumulator = ebe.slam.XPPlatformAccumulator();
mainLoop.addResultsAccumulator(resultsAccumulator);

% Set up the figure in which we draw everything
fig = FigureManager.getFigure("Scenario Output");
clf
hold on
axis([-30 30 -30 30])
axis square

% Set up the views which show the output of the simulator
simulatorViewer = ebe.graphics.ViewManager(config);
simulatorViewer.addView(l2.dotbot.SimulatorView(config, simulator));

% Register the viewer with the mainloop
mainLoop.addViewer(simulatorViewer);

% Run the main loop until it terminates
mainLoop.run();

THistory = resultsAccumulator.timeStore;
XTrueHistory = resultsAccumulator.xTrueStore;

% Plot out state information
sigmaErrorBounds = ebe.graphics.FigureManager.getFigure('Ground Truth State Values');
clf

stateLabels = {'$x$','$y$'};

plot(THistory, XTrueHistory, 'LineWidth', 2);
xlabel('Time')
ylabel('State value')
legend(stateLabels, 'Interpreter', 'latex')