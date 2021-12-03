%% Trajectory Generation - Motion between Waypoints - In jnt coordinates
% A function of some polynomial trajectory scheme and a search algorithm
% that ensures compliance with motion constrains, here defined as joint
% position, velocity and acceleration, but a more appropiate definition is
% torque Path definition as a series of waypoints accompanied by 
% constraints on their motion. Generate trajectory over segment A,B,C....
% Deine waypoints as [4x4] HTs in cartesian space, define maximum joint
% velocities and acceleration for each joint or for all joints
% Initialization
clear;load('Initialization.mat');
disp('Starting Trajectry Generation')
path_gen = struct('Segment',[],'Jnt',[],'tvct',[]); 
sgmnt_gen = struct('Start',[],'End',[],'Vwmax',[],'Awmax',[],'t0',0,...
    'tf',0.1);
% Initialize # of paths and segments
path = path_gen;segment1 = sgmnt_gen;segment2 = sgmnt_gen;

% Pick a joint configuration just to test - this ideally comes from the
% waypoints

PathObj.Segment = [];

% Waypoints:
rest = 50; 
jseq(:,1) = [0, 0, 0, 0, 0, 0];
jseq(:,2) = [-0.77, 1.10, 0.21, 0, 0, 0];
jseq(:,3) = [0, 1.01, 0.40, 0, 0, 0];
jseq(:,4) = [0, 0, 1.57, 0, 0, 0];
jseq(:,5) = [0, 0, 0, 0, 0, 0];

% Joint Velocity Limits:
vel_lim = min([6.5448, 5.4977, 7.1558, 9. 55, 9.55, 17]);

% Joint Acceleration Limits:
% *difficult to say b/c depends on payload, but here's some guesses...

acc_lim = min([3, 3, 3, 3, 3, 3]);

for ii = 1:size(jseq,2)-1
    sgmnt_gen.Start = jseq(:,ii);
    sgmnt_gen.End =  jseq(:,ii+1);
    sgmnt_gen.Vwmax = vel_lim;
    sgmnt_gen.Awmax = acc_lim;
    PathObj.Segment = [PathObj.Segment,sgmnt_gen];
end


% Create the trajectory, mified from original code to not need frdw/inv
% kinematics

hrz = rest; 

[TrjObj,TT] = jnttrjgn(PathObj,rest);
TT.Properties.VariableUnits = {'rads','rads/sec','rads/(sec^2)'};
disp('Wrtign CSV')
writetimetable(TT,'Trajectory.csv','Delimiter','bar');
disp('Plotting trajecotry')
figure
stackedplot(TT); grid on;

% Vizualisation
r = rateControl(240);
figure(9)
show(GP7,double(jseq(:,1))')
pause(1)
for ii = 1:length(TrjObj.tvct)
    config = TrjObj.Jnt.jnt(:,ii)';
    show(GP7,config);title(ii)
    waitfor(r);
end

set(0,'DefaultFigureWindowStyle','normal')
