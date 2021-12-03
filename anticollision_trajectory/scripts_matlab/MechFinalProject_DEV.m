%% Definition
% This program is meant to run using Matlab R2021b and the following% available addons:
%   Robotic System Toolbox (R) Matlab
%   Peter Corke's Robotics System Toolbox for Matlab [https://github.com/petercorke/robotics-toolbox-matlab/tree/bd7a9d75176c660f43fc799b24d838f70b02250c]
%   Peter Corke's Spatial Math [Matlab Addon Interface 2021 version]
%   Simulink and Simulink ROS package [Matlab Addon Interface]
%   Simscape and Simulink Animation [Matlab Addon Interface]
%   
% To the best extent of our abilities most relevant code
% is included in this master file

%% Initialization Uncomment in needed
clear;clc;
resj = 20;                                                                  % Resolution of the sampling grids in joint space in degrees
resc = 0.5;                                                                 % Resolution of the sampling grids in cartesian space in distance units
rest = 50;                                                                  % Resolution of the sampling grids in time space in hrz
inintGP7(resj,resc,rest)
clear; clc;
%% Search Algorithms
% Problem can be solved using a variety of search/optimization algorithms
% The start of any solution begins by defining an ocupancy map in some
% fashion, we use a potential field metholodoloy.
% Load workspace
clear;clc;
load('Initialization.mat')
disp('Start')
set(0,'DefaultFigureWindowStyle','docked')
figure_1 = show(GP7);                                                       % Figure to show the robot's reachability
figure(2);subplot(1,2,1);subplot(1,2,2);figure_2 = figure(2);               % Figure used to create the manipulator potential fields

% Define some symbolic operators from the loaded worskapce
q = sym('q', size(q),'real');                                              % Position Variables
qd = sym('qd', size(q),'real');                                            % Velocity Variables
qdd = sym('qdd', size(q),'real');                                          % Acceleration Variables

T_EE = symfun(Tbe,q');                                                      % EE homogeneous transformation Tbe is loaded from .m file
Rotbe = Tbe(1:3,1:3); Rot_EE = symfun(Rotbe,q');                            % EE rotation independently
Pbe = Tbe(1:3,4); XYZ_EE = symfun(Pbe,q');                                  % EE postion independently

% Joint Axes - Imported and subtituting the 'pi' symbol by pi for evltn 
j1 = subs(robot.Jnts(1).lmts,'pi',pi); n1 = numel(j1);
j2 = subs(robot.Jnts(2).lmts,'pi',pi); n2 = numel(j2);
j3 = subs(robot.Jnts(3).lmts,'pi',pi); n3 = numel(j3);
j4 = subs(robot.Jnts(4).lmts,'pi',pi); n4 = numel(j4);
j5 = subs(robot.Jnts(5).lmts,'pi',pi); n5 = numel(j5);
j6 = subs(robot.Jnts(6).lmts,'pi',pi); n6 = numel(j6);

complete_jspace_nmelmnts = n1*n2*n3*n4*n5*n6;

[j1g,j2g,j3g] = ndgrid(j1,j2,j3);
j4g = zeros(size(j1g)); j5g = j4g; j6g = j4g;
disp('Done Defining volume in J space')
% View Dexterous workspace
disp('View Dexterous workspace')
XYZc = XYZ_EE(j1g,j2g,j3g,zeros(size(j1g)),zeros(size(j1g)),zeros(size(j1g)));
X = ((XYZc{1}(:)./1000)); Y = ((XYZc{2}(:)./1000)); Z = ((XYZc{3}(:)./1000));
figure(1)
pause(0.5)
show(GP7)
hold on
scatter3(X,Y,Z,'b.')
disp('Done Dexterous workspace')
pause(0.5)
% The potential fields are defined as follows:
%
%   Pg = Potential due to positioning error defined as the 2K power of the
%   distance between the end effector and the goal. Calcualted only for the
%   end effector. Pg = ( rho .* dist(XYZ_goal , XYZ_EE(:)).^(2*k) )
%   
%   Ps = Potential due to cartesian proximity to the starting position,
%   meant to penalize returning to the initial joint configuration.
%   Calculated only for the EE. Ps = ( sig .* (1./(XYZ_srt , XYZ_EE(:)).^(2)) )
%   
%   Pf = Floor potential due to the desire to not get close to the floor,
%   floor is defined as [X = any;Y = Any; Z = some real], The smaller the
%   difference in Z, the greater the force. Calculated for the output of a 
%   joint. Pf = ( mu.* (1./(Z_flr , Z_EE(:))).^(2)) )
%
%   Po = Obstacle potential field due to proximity to the boundaries of an 
%   obstacle, the closer the XYZ is to the boundaries of an object the
%   higher the value is, if its inside an object its just very large.
%   Piecesies function shown bellow
%
%   Pa = Potential due to manipulability potential  caused due to some
%   notion of manipulability
%
%

% Define the start/end configurations in joint space
ofst = 5;                                                                   % Ofster from the end of the jnt limits given the rest number of steps
Jntstrt = sym([ j1(ofst)   ,  j2(ofst)   ,   j3(ofst)  , 0 , 0 , 0]);       % Start Configuration
Jntgoal = sym([j1(end-ofst), j2(end-ofst), j3(end-ofst), 0 , 0 , 0]);       % End Configuration

% Convert these markers into cartesian space values
XYZ_start = simplify(XYZ_EE(Jntstrt(1),Jntstrt(2),Jntstrt(3),...
                            Jntstrt(4),Jntstrt(5),Jntstrt(6))./1000);

XYZ_goal = simplify(XYZ_EE(Jntgoal(1),Jntgoal(2),Jntgoal(3),...
                           Jntgoal(4),Jntgoal(5),Jntgoal(6))./1000);
disp('Potential functions')
% Potential Fucntions of not obstacle concerns
rho = sym('1'); K = sym('2');                                               % Constants to controll the growth of the  potential field
Pg = ( rho .* cdist(XYZ_goal , [Xc;Yc;Zc]).^(2*K) );                        % Potential to the goal
Pgf = symfun(Pg,[Xc,Yc,Zc]); 
sig= sym('1/2');sft = sym('1/1000');                                          % Constants to controll the growth of the  potential field
Ps = ( sig .* (1./(cdist(XYZ_start , [Xc;Yc;Zc])+sft)).^(2));               % Potential from the start
Psf = symfun(Ps,[Xc,Yc,Zc]);
mu = sym('1/2'); Z_flr = sym('0');                                            % Constants to controll the growth of the  potential field
Pf = ( mu .* (1./(cdist(Z_flr , Zc)+sft)).^(2));                            % Potential from the floor
Pff = symfun(Pf,[Xc,Yc,Zc]);

cntg = 1; cnts = 1; cntf = 1;                                               % Select the weights to assign each potential field
Pt = (cntg*Pg + cnts*Ps + cntf*Pf);
Ptf = symfun(Pt,[Xc,Yc,Zc]);                                                % Creates a separate symbolic function      
gPt = gradient(Pt);
gPtf = gradient(Ptf);

% Insert the forward kineamtics equations into the cartesian potential 
% fields the division by 1000 are to scale eerything to the meters
jnt2crt_Pg_EE = Pgf(Pbe(1)/1000,Pbe(2)/1000,Pbe(3)/1000);
jnt2crt_Pf_EE = Pff(Pbe(1)/1000,Pbe(2)/1000,Pbe(3)/1000);
jnt2crt_Pf_jnt1b = Pff(robot.Jnts(1).Tb(1,4)/1000,...
                       robot.Jnts(1).Tb(2,4)/1000,...
                       robot.Jnts(1).Tb(3,4)/1000);
jnt2crt_Pf_jnt2b = Pff(robot.Jnts(2).Tb(1,4)/1000,...
                       robot.Jnts(2).Tb(2,4)/1000,...
                       robot.Jnts(2).Tb(3,4)/1000);
jnt2crt_Pf_jnt3b = Pff(robot.Jnts(3).Tb(1,4)/1000,...
                       robot.Jnts(3).Tb(2,4)/1000,...
                       robot.Jnts(3).Tb(3,4)/1000);
jnt2crt_Pf_jnt4b = Pff(robot.Jnts(4).Tb(1,4)/1000,...
                       robot.Jnts(4).Tb(2,4)/1000,...
                       robot.Jnts(4).Tb(3,4)/1000);
jnt2crt_Pf_jnt5b = Pff(robot.Jnts(5).Tb(1,4)/1000,...
                       robot.Jnts(5).Tb(2,4)/1000,...
                       robot.Jnts(5).Tb(3,4)/1000);
jnt2crt_Pf_jnt6b = Pff(robot.Jnts(6).Tb(1,4)/1000,...
                       robot.Jnts(6).Tb(2,4)/1000,...
                       robot.Jnts(6).Tb(3,4)/1000);

jnt2crt_Pf_jntb = jnt2crt_Pf_jnt1b + jnt2crt_Pf_jnt2b + jnt2crt_Pf_jnt3b + jnt2crt_Pf_jnt4b + jnt2crt_Pf_jnt5b + jnt2crt_Pf_jnt6b;

jnt2crt_Pt = jnt2crt_Pg_EE + jnt2crt_Pf_EE + jnt2crt_Pf_jntb;
jnt2crt_Ptf = symfun(jnt2crt_Pt,q);
% The previous function remains a function of all 6 joints making it hard
% to anser the porblme under the "lock j5" assumption but the following
% lines fix that
jnt2crt_Pt = jnt2crt_Ptf(q(1),q(2),q(3),0,0,0);
jnt2crt_Ptf = symfun(jnt2crt_Pt,[q(1),q(2),q(3)]);

g_jntPt = gradient(jnt2crt_Pt);
g_jntPtf = gradient(jnt2crt_Ptf);

disp('Calculating samples of potential fields')
% % This section can be ignored if no time is to be wasted rendering a couple
% % thousands points
X = reshape(X,resj,resj,resj);
Y = reshape(Y,resj,resj,resj);
Z = reshape(Z,resj,resj,resj);
cartPt = Ptf(X,Y,Z);
cartPtg = gPtf(X,Y,Z); 
jntPt = jnt2crt_Ptf(j1g,j2g,j3g); 
jntPtg = g_jntPtf(j1g,j2g,j3g); 

n = numel(cartPt);

%%
% c1 = cartPtg{1}(:);
% c2 = cartPtg{2}(:);
% c3 = cartPtg{3}(:);
% jt1 = jntPtg{1}(:);
% jt2 = jntPtg{2}(:);
% jt3 = jntPtg{3}(:);
% 
% cartPt = nan(1,n);
% parfor ii = 1
%     cartPt(ii) = double(cartPt(ii));
% end
% jntPt = cartPt;
% 
% parfor ii = 1
%     jntPt(ii) = double(jntPt(ii));
% end
% cartPtg1 = cartPt;
% cartPtg2 = cartPt;
% cartPtg3 = cartPt;
% parfor ii = 1
%     cartPtg1(ii) = double(c1(ii));
%     cartPtg2(ii) = double(c2(ii));
%     cartPtg3(ii) = double(c3(ii));
% end
% jntPtgt1 = cartPt;
% jntPtgt2 = cartPt;
% jntPtgt3 = cartPt;
% parfor ii = 1
%     jntPtgt1(ii) = double(jt1(ii)); 
%     jntPtgt2(ii) = double(jt2(ii));
%     jntPtgt3(ii) = double(jt3(ii));
% end
% 


%%
cartPt(:) = double(cartPt(:));jntPt(:) = double(jntPt(:));
cartPtg1 = double(cartPtg{1}(:)); 
cartPtg2 = double(cartPtg{2}(:));
cartPtg3 = double(cartPtg{3}(:));
jntPtgt1 = double(jntPtg{1}(:)); 
jntPtgt2 = double(jntPtg{2}(:));
jntPtgt3 = double(jntPtg{3}(:));
disp('Done with that')
disp('Saving it')
%%
figure
pause(0.5)
scatter3(X(:),Y(:),Z(:),2,cartPt(:)); hold on; colorbar; hold on
pause(0.5)
quiver3(X(:),Y(:),Z(:),cartPtg1(:),cartPtg2(:),cartPtg3(:));
pause(0.1)
figure
scatter3(j1g(:),j2g(:),j3g(:),jntPt(:)); hold on; colorbar; hold on
pause(0.5)
quiver3(j1g(:),j2g(:),j3g(:),jntPtgt1(:),jntPtgt2(:),jntPtgt3(:))
pause(0.5)

%save('disspace.mat','cartPt','cartPtg','jntPt','jntPtg','Ptf','gPtf','jnt2crt_Ptf','g_jntPtf')
%clear
%%
disp('Ploting Potential fields')
%load("disspace.mat")
%load('Initialization.mat')

figure
pause(0.1)
scatter3(X(:),Y(:),Z(:),2,cartPt(:)); hold on; colorbar; hold on
pause(0.1)
quiver3(X(:),Y(:),Z(:),2,cartPtg(1,:),cartPtg(2,:),cartPtg(3,:));
pause(0.1)
figure
scatter3(j1g(:),j2g(:),j3g(:),2,jntPt(:)); hold on; colorbar; hold on
pause(0.5)
quiver3(j1g(:),j2g(:),j3g(:),2,jntPtg(1,:),jntPtg(2,:),jntPtg(3,:))
pause(0.5)
%
clc
%%
disp('Starting Gradient Descent')
Gama.toll = 1e-3;
Gama.gama = 1e-13;
coordaxes = struct('dir',[]);
coordaxes(1).dir = j1; coordaxes(2).dir = j2; coordaxes(3).dir = j3;
Startj = Jntstrt; Startc = XYZ_start;
Endj = Jntgoal; Endc = XYZ_goal;
jseq = gradDscnt(Startj,Endj,Startc,Endc,jnt2crt_Ptf,g_jntPtf,coordaxes,Gama,XYZ_EE);

disp('Finished Gradient Descent')
XYZ = zeros(3,length(jseq));

for ii = 1:length(jseq)
    XYZ(:,ii) = (Tf(jseq(1,ii),jseq(2,ii),jseq(3,ii),jseq(4,ii),jseq(5,ii),jseq(6,ii)))./1000;
end

figure(7)
show(GP7); hold on
scatter3(XYZ(1,:),XYZ(2,:),XYZ(3,:))
plot3(XYZ(1,:),XYZ(2,:),XYZ(3,:))
hold off

set(0,'DefaultFigureWindowStyle','normal')

%% Trajectory Generation - Motion between Waypoints - In jnt coordinates
% A function of some polynomial trajectory scheme and a search algorithm
% that ensures compliance with motion constrains, here defined as joint
% position, velocity and acceleration, but a more appropiate definition is
% torque Path definition as a series of waypoints accompanied by 
% constraints on their motion. Generate trajectory over segment A,B,C....
% Deine waypoints as [4x4] HTs in cartesian space, define maximum joint
% velocities and acceleration for each joint or for all joints
disp('Starting Trajectry Generation')

path_gen = struct('Segment',[],'Jnt',[],'tvct',[]); 
sgmnt_gen = struct('Start',[],'End',[],'Vwmax',[],'Awmax',[],'t0',0,...
    'tf',[]);
% Initialize # of paths and segments
path = path_gen;segment1 = sgmnt_gen;segment2 = sgmnt_gen;

% Pick a joint configuration just to test - this ideally comes from the
% waypoints

Vwmax = 0.25  ;Awmax = 0.25;

PathObj.Segment = [];
for ii = 1:length(jseq)-1
    sgmnt_gen.Start = jseq(:,ii);
    sgmnt_gen.End =  jseq(:,ii+1);
    sgmnt_gen.Vwmax = Vwmax;
    sgmnt_gen.Awmax = Awmax;
    PathObj.Segment = [PathObj.Segment,sgmnt_gen];
end


% Create the trajectory, mified from original code to not need frdw/inv
% kinematics

hrz = rest; 

[TrjObj,TT] = jnttrjgn(PathObj,hrz);
TT.Properties.VariableUnits = {'rads','rads/sec','rads/(sec^2)'};
disp('Wrtign CSV')
writetimetable(TT,'Trajectory.csv','Delimiter','bar');
disp('Plotting trajecotry')
figure
stackedplot(TT); grid on;

% Vizualisation
r = rateControl(120);
figure(9)
show(GP7,double(jseq(:,1)))
pause
for ii = 1:length(TrjObj.tvct)
    config = TrjObj.Jnt.jnt(:,ii);
    show(GP7,config);title(ii)
    waitfor(r);
end

set(0,'DefaultFigureWindowStyle','normal')

%% Notes and good links
% https://stackoverflow.com/questions/51158078/load-stl-file-in-matlab-and-convert-to-a-3d-array
%
%