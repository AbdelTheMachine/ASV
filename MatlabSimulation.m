%Final: April 12, 2025         CEE 4402 - Project


%=========================
%Code for SLAM using Matlab through both Visual Odometry (VO) and
%Visual-Inertial Odometry (VIO).
%Order of execution: graph appears --> user asked for input --> graph
%automatically starts simulation. User must click back on the graph window
%--> once simulation duration is completed, trajectory plot appears
%automatically, no user interference to stop the execution.
%=========================

close all; clc; clear

%Create the driving scenario:
scene = drivingScenario;
groundTruthVehicle = vehicle(scene, 'PlotColor', [0 0.4470 0.7410]);
estVehicle = vehicle(scene, 'PlotColor', [0 0.4470 0.7410]);

%% Now define the trajectory that the car will move on.

%Start with a random path to try:

sampleRate = 100; %Sampling rate for IMU (1 cycle = 100 data points)
wayPoints = [  0    0  0;   % Start point
             100    0  0;   % Right side
             200   50  0;   % Top-right curve
             200  150  0;   % Top
             100  200  0;   % Top-middle
               0  150  0;   % Top-left curve
            -100  100  0;   % Left side
            -200   50  0;   % Bottom-left curve
            -100    0  0;   % Bottom-middle
               0    0  0];  % Back to start


t = [0 20 40 60 80 100 120 140 160 180].'; % Time of arrival for each waypoint

speed = 10;
velocities = [ speed     0  0;   % Move right
               speed     0  0;   % Move right
                   0  speed  0;   % Move up
              -speed     0  0;   % Move left
                   0 -speed  0;   % Move down
              -speed     0  0;   % Continue moving left
                   0 -speed  0;   % Move downward on the left curve
               speed     0  0;   % Move right along bottom
               speed     0  0;   % Move back to start
                   0      0  0]; % End at origin

traj = waypointTrajectory(wayPoints, 'TimeOfArrival', t, ...
    'Velocities', velocities, 'SampleRate', sampleRate);

%% Add building and road to visualize:

%Can't use the built-in function since only runs in the 2018-19 documentation
%example.
%helperPopulateScene(scene, groundTruthVehicle);

roadCenters = wayPoints; % Use the same oval shape for road path since
                         % car should follow same path. 


road(scene, roadCenters, 'Lanes', lanespec([1 2]));

buildingColor = [250 235 215] / 255;

% Add buildings surrounding the elliptical path
actor(scene, ...
    'Length', 100, ...
    'Width', 30, ...
    'Height', 80, ...
    'Yaw', 170, ...
    'Position', [280 0 0], ... % Outside right side
    'PlotColor', buildingColor);

actor(scene, ...
    'Length', 30, ...
    'Width', 20, ...
    'Height', 100, ...
    'Position', [250 200 0], ... % Top-right corner
    'PlotColor', buildingColor);

actor(scene, ...
    'Length', 100, ...
    'Width', 30, ...
    'Height', 80, ...
    'Yaw', 45, ...
    'Position', [0 230 0], ... % Top-center
    'PlotColor', buildingColor);

actor(scene, ...
    'Length', 175, ...
    'Width', 35, ...
    'Height', 60, ...
    'Yaw', 30, ...
    'Position', [-150 150 0], ... % Top-left corner
    'PlotColor', buildingColor);

actor(scene, ...
    'Length', 50, ...
    'Width', 100, ...
    'Height', 80, ...
    'Position', [-260 0 0], ... % Outside left side
    'PlotColor', buildingColor);

actor(scene, ...
    'Length', 100, ...
    'Width', 30, ...
    'Height', 100, ...
    'Position', [-150 -30 0], ... % Bottom-left corner
    'PlotColor', buildingColor);

actor(scene, ...
    'Length', 30, ...
    'Width', 50, ...
    'Height', 100, ...
    'Position', [-150 -40 0], ... % Bottom-left corner
    'PlotColor', buildingColor);

actor(scene, ...
    'Length', 30, ...
    'Width', 100, ...
    'Height', 80, ...
    'Position', [245 100 0], ... % Bottom-center
    'PlotColor', buildingColor);

actor(scene, ...
    'Length', 180, ...
    'Width', 30, ...
    'Height', 100, ...
    'Position', [75 -40 0], ... % Bottom-right corner
    'PlotColor', buildingColor);



figScene = findall(0, 'Type', 'Figure', 'Tag', 'VIODemoDisplay');

if isempty(figScene)
    figScene = figure('Name', 'Driving Scenario', ...
        'Tag', 'VIODemoDisplay', 'Position', [0, 0, 1032, 600]);
    movegui(figScene, [0 -1]);
else
    figure(figScene);
end
clf(figScene);

% Plot the top view of the scene.
hTopViewPanel = uipanel(figScene, 'Position', [0 0 0.5 1], ...
    'Title', 'Bird''s Eye View');
hTopViewAxes = axes(hTopViewPanel);
plot(scene, 'Parent', hTopViewAxes);
xlim(hTopViewAxes,[-20 350])
ylim(hTopViewAxes,[-50 350])
zlim(hTopViewAxes,[0 200])
view(hTopViewAxes,-85,60)
unifyVehicleColors(hTopViewAxes);

%% Plot the aerial/whole landscape view & the isometric 3D view of the cube:

% Plot the chase (3D isometric) view of the scene from the rear of the vehicle.
hChaseViewPanel = uipanel(figScene, 'Position', [0.5 0 0.5 1], ...
    'Title', 'Chase View');
hChaseViewAxes = axes(hChaseViewPanel);
chasePlot(groundTruthVehicle, 'Parent', hChaseViewAxes);
xlim(hChaseViewAxes,[-20 350])
ylim(hChaseViewAxes,[-50 350])
zlim(hChaseViewAxes,[0 200])

%function definition is at the end of the code.
unifyVehicleColors(hChaseViewAxes);

drawnow;


%% Create a Fusion filter to fuse IMU and visual odometry measurements:
filt = insfilterErrorState('IMUSampleRate', sampleRate, ...
    'ReferenceFrame', 'ENU');

helperInitialize(filt, traj);


%% Specify the visual odometry model = the camera properties:

prompt = 'Type true if using camera in addition to IMU, or type false for not camera & just IMU for odometry: ';
answer = input(prompt);
if answer == true
    useVO = true;
else
    useVO = false;
end

paramsVO.scale = 2;
paramsVO.sigmaN = 0.139;
paramsVO.tau = 232;
paramsVO.sigmaB = sqrt(1.34);
paramsVO.driftBias = [0 0 0];


%% Specify the IMU sensor:

%Define an IMU sensor model:

rng('default') %Like a random seed.

imu = imuSensor('SampleRate', sampleRate, 'ReferenceFrame', 'ENU' );

%Accelerometer:
imu.Accelerometer.MeasurementRange = 19.6; %m/s^2
imu.Accelerometer.Resolution = 0.0024; %m/s^2/LSB
imu.Accelerometer.NoiseDensity = 0.01; %(m/s^2)/sqrt(Hz)

%Gyroscope
imu.Gyroscope.MeasurementRange = deg2rad(250);
imu.Gyroscope.Resolution = deg2rad(0.0625);
imu.Gyroscope.NoiseDensity = deg2rad(0.0573);
imu.Gyroscope.ConstantBias = deg2rad(2);

%% Set up the simulation:

%Simulation duration: 60 seconds:
numSecondsToSimulate = 60;
numIMUSamples = numSecondsToSimulate*sampleRate;

%Define the visual odometry sampling rate:
imuSamplesPerCamera = 4;
numCameraSamples = ceil(numIMUSamples / imuSamplesPerCamera);

[pos, orient, vel, acc, angvel, ...
    posVO, orientVO, ...
    posEst, orientEst, velEst] = helperPreallocateData(numIMUSamples, numCameraSamples);


%Set measurment noise parameters for the visual odometry fusion:
RposVO = 0.1;
RorientVO = 0.1;


cameraIdx = 1;
for i = 1:numIMUSamples

    %Generate ground truth trajectory values.
    [pos(i,:), orient(i,:), vel(i,:), acc(i,:), angvel(i,:)] = traj();

    %Generate accelerometer and gyroscope measurements from the ground
    %truth trajectory values
    [accelMeas, gyroMeas] = imu(acc(i,:), angvel(i,:), orient(i));

    predict(filt, accelMeas, gyroMeas);

    if (1 == mod(i, imuSamplesPerCamera)) && useVO

        %Generate a visual odometry pose estimate from the ground truth
        %values and visual odometry model.
        [posVO(cameraIdx,:), orientVO(cameraIdx,:), paramsVO] = ...
            helperVisualOdometryModel(pos(i,:), orient(i,:), paramsVO);

        fusemvo(filt, posVO(cameraIdx,:), RposVO, ...
            orientVO(cameraIdx), RorientVO);

        cameraIdx = cameraIdx +1;

    end

    [posEst(i,:), orientEst(i,:), velEst(i,:)] = pose(filt);

    %Update the estimated vehicle pose:
    helperUpdatePose(estVehicle, posEst(i,:), velEst(i,:), orientEst(i));

    %Update estimated vehicle pose:
    helperUpdatePose(estVehicle, posEst(i,:), velEst(i,:), orientEst(i));

    %Update ground truth vehicle pose:
    helperUpdatePose(groundTruthVehicle, pos(i,:), vel(i,:), orient(i));

    %Update the driving scenario visualization:
    updatePlots(scene);
    drawnow limitrate;
end

%% Plot the ground truth vehicle trajectory, visual odometry estimate, 
% and the fusion filter estimate:

%IMPORTANT: THE PLOT APPEARS BY ITSELF ONLY **AFTER** THE DURATION OF SIMULATION.

figure

if useVO == true
    plot3(pos(:,1), pos(:,2), pos(:,3), '-.', ...
        posVO(:,1), posVO(:,2), posVO(:,3), ...
        posEst(:,1), posEst(:,2), posEst(:,3), ...
        'LineWidth', 3)
    legend('Ground Truth', 'Visual Odometry (VO)', ...
        'Visual-Inertial Odometry (VIO)', 'Location', 'northeast', 'FontSize', 14 )
else
    plot3(pos(:,1), pos(:,2), pos(:,3), '-.', ...
        posEst(:,1), posEst(:,2), posEst(:,3), ...
        'LineWidth', 3)
    legend('Ground Truth', 'IMU Pose Estimate', 'FontSize', 14 )
end

view(-90, 90)
title('Comparison of Mapping Techniques Using Visual Odometry and Visual-Inertial Odometry ', 'FontSize', 24)
xlabel('X (m)', 'FontSize', 20)
ylabel('Y (m)', 'FontSize', 20 )
legend show
grid on


%% Function definitions (must be at end of a Matlab file):

function unifyVehicleColors(ax)
%UNIFYVEHICLECOLORS Match all vehicle colors in the input axes.

    for i = 1:numel(ax.Children)
        p = ax.Children(i);
        numStr = extractAfter(p.Tag, 'ActorPatch');
        if ~isempty(numStr)
            num = str2double(numStr);
            if (num == 2)
                p.FaceColor = ax.ColorOrder(1,:);
                p.EdgeColor = 0.8*ax.ColorOrder(1,:);
                p.FaceAlpha = 0.3;
                p.EdgeAlpha = 0.3;
            end
        end
    end
end



function helperInitialize(filt, traj)

%Get the initial position, orientation, and velocity from
%the trajectory object, & reset the internal states:
[pos, orient, vel] = traj();
reset(traj);

%Set the initial orientation, position, and velocity:
filt.State(1:4) = compact(orient(1)).';
filt.State(5:7) = pos(1,:).';
filt.State(8:10) = vel(1,:).';

%Set the gyroscope bias & visual odometry scale factor covariance to large
%values corresponding to low confidence:
filt.StateCovariance(10:12, 10:12) = 1e6;
filt.StateCovariance(end) = 2e2;
end


function [posVO, orientVO, paramsVO] ...
    = helperVisualOdometryModel(pos, orient, paramsVO)

%Extract model parameters:
scaleVO = paramsVO.scale;
sigmaN = paramsVO.sigmaN;
tau = paramsVO.tau;
sigmaB = paramsVO.sigmaB;
sigmaA = sqrt((2/tau) + 1/(tau*tau))*sigmaB;
b = paramsVO.driftBias;

%Calculate drift:
b = (1 - 1/tau).*b + randn(1, 3)*sigmaA;
drift = randn(1,3)*sigmaN + b;
paramsVO.driftBias = b;

%Calculate visual odometry measurements:
posVO = scaleVO*pos + drift;
orientVO = orient;

end


function [pos, orient, vel, acc, angvel, ...
    posVO, orientVO, ...
    posEst, orientEst, velEst]...
    = helperPreallocateData(numIMUSamples, numCameraSamples)

%specify the ground truth:
pos = zeros(numIMUSamples, 3);
orient = quaternion.zeros(numIMUSamples, 1);
vel = zeros(numIMUSamples, 3);
acc = zeros(numIMUSamples, 3);
angvel = zeros(numIMUSamples, 3);

%Visual odometry output:
posVO = zeros(numCameraSamples, 3);
orientVO = quaternion.zeros(numCameraSamples, 1);

%Filter the output:
posEst = zeros(numIMUSamples, 3);
orientEst = quaternion.zeros(numIMUSamples, 1);
velEst = zeros(numIMUSamples, 3);

end

function helperUpdatePose(veh, pos, vel, orient)

veh.Position = pos;
veh.Velocity = vel;
rpy = eulerd(orient, 'ZYX', 'frame');
veh.Yaw = rpy(1);
veh.Pitch = rpy(2);
veh.Roll = rpy(3);

end