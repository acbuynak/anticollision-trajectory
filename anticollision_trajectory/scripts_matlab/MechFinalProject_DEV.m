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
resj = 30;                                                                  % Resolution of the sampling grids in joint space in degrees
resc = 0.5;                                                                 % Resolution of the sampling grids in cartesian space in distance units
rest = 50;                                                                  % Resolution of the sampling grids in time space in hrz
inintGP7(resj,resc,rest)
clear; clc;
%% Load workspace
clear;clc;
load('Initialization.mat')
set(0,'DefaultFigureWindowStyle','docked')
figure_1 = show(GP7);                                                       % Figure to show the robot's reachability
Figure(2);subplot(1,2,1);subplot(1,2,2);figure_2 = figure(2);               % Figure used to create the manipulator potential fields
%% Search Algorithms
% Problem can be solved using a variety of search/optimization algorithms
% The start of any solution begins by defining an ocupancy map in some
% fashion, we use a potential field metholodoloy. The potential fields are
% as follows:
%
%   Pg = Potential due to positioning error defined as the 2K power of the
%   distance between the end effector and the goal. Calcualted only for the
%   end effector. Pe = ( rho .* dist(XYZ_goal - XYZ_EE(:)).^(2*k) )
%   
%   Ps = Potential due to cartesian proximity to the starting position,
%   meant to penalize returning to the initial joint configuration.
%   Calculated only for the EE. Ps = ( sig .* (1./(XYZ_srt - XYZ_EE(:)).^(2)) )
%   
%   Pf = Floor potential due to the desire to not get close to the floor,
%   floor is defined as [X = any;Y = Any; Z = some real], The smaller the
%   difference in Z, the greater the force. Calculated for the output of a 
%   joint. Pf = ( mu.* (1./(Z_flr - Z_EE(:))).^(2)) )
%
%   Po = Obstacle potential field due to proximity to the boundaries of an 
%   obstacle, the closer the XYZ is to the boundaries of an object the
%   higher the value is, if its inside an object its just very large.
%   Piecesies function shown bellow

% Define some symbolic operators from the loaded worskapce
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

% Define the start/end configurations in joint space
ofst = 3;                                                                   % Ofster from the end of the jnt limits given the rest number of steps
Jntstrt = sym([ j1(ofst)   ,  j2(ofst)   ,   j3(ofst)  , 0 , 0 , 0]);       % Start Configuration
Jntgoal = sym([j1(end-ofst), j2(end-ofst), j3(end-ofst), 0 , 0 , 0]);       % End Configuration

% Convert these markers into cartesian space values
XYZ_start = XYZ_EE(Jntstrt(1),Jntstrt(2),Jntstrt(3),Jntstrt(4),Jntstrt(5),Jntstrt(6));
XYZ_goal = XYZ_EE(Jntgoal(1),Jntgoal(2),Jntgoal(3),Jntgoal(4),Jntgoal(5),Jntgoal(6));

% Obstacle Potential Fields

LIMS = [-3, 3, -3,3,-2,5];

Floor(x,y,z) =  piecewise(z < 0,9,0);

Ob1(x,y,z) = piecewise(  0 < x & x < 1 &...
                         0 < y & y < 1 &...
                         0 < z & z < 1,9,0);

Ob2(x,y,z) = piecewise(0 < x & x < 2 &...
                      -1 < y & y < 1 &...
                       0 < z & z < 2,9,0);
Obt = Ob1+Ob2;

% View Obstacle potential fileds

% figure(4) % Trust me this is a waste of memory
% fimplicit3(Obt,LIMS,"MeshDensity",30);
% axis(LIMS)
% xlabel('X');ylabel('Y');zlabel('Z');
% title('Obstacle Potential Field')
% grid on
% pause(0.5)
set(0,'DefaultFigureWindowStyle','normal')
% Do Forward Kinematics and get weights
set(0,'DefaultFigureWindowStyle','docked')
clc
n1 = numel(robot.Jnts(1).lmts);n2 = numel(robot.Jnts(2).lmts);
n3 = numel(robot.Jnts(3).lmts);n4 = numel(robot.Jnts(4).lmts);
n5 = numel(robot.Jnts(5).lmts);n6 = numel(robot.Jnts(6).lmts);

jPspace = nan(n1,n2,n3);

nj = numel(jPspace);

j1 = subs(robot.Jnts(1).lmts,'pi',pi);j2 = subs(robot.Jnts(2).lmts,'pi',pi);
j3 = subs(robot.Jnts(3).lmts,'pi',pi);j4 = subs(robot.Jnts(4).lmts,'pi',pi);
j5 = subs(robot.Jnts(5).lmts,'pi',pi);j6 = subs(robot.Jnts(6).lmts,'pi',pi);

[j1g,j2g,j3g] = ndgrid(j1,j2,j3);

disp([n1,n2,n3,n4,n5,n6,nj]);
disp('ready for Pt')

% This step takes a bit to process but its actually really good all things
% considered, only tought for 4 joints
% [j1g,j2g,j3g,j5g] = ndgrid(j1,j2,j3,j5);
% jspace = nan(n1,n2,n3,n5);
% Xgoal = 0; Ygoal = 0; Zgoal = 0;
% Pt = nan(n1,n2,n3,n5);
% parfor ii = 1:n1
%     for jj = 1:n2
%         for kk = 1:n3
%             for ll = 1:n5
%                 C = Tf(j1(ii),j2(jj),j3(kk),0,j5(ll),0);
%                 Po = Obt(C{1,1}./1000,C{2,1}./1000,C{3,1}./1000);
%                 Pd = (Xgoal - C{1,1}./1000).^2  +  (Ygoal - C{2,1}./1000).^2  +...  
%                      (Zgoal - C{3,1}./1000).^2 ;
%                 Pt(ii,jj,kk,ll) = double(Po + Pd)
%             end
%         end
%     end
% end

ii = 3;                                                                     % Define how close to the edges of the joint ranges you want to do the motion


C = Tf(j1g,j2g,j3g,zeros(size(j1g)),zeros(size(j1g)),zeros(size(j1g)));

%
clc
X = double(C{1,1})./1000; Y = double(C{2,1})./1000; Z = double(C{3,1})./1000;
%
disp('half way Pt1')
Po = double(Obt(C{1,1}./1000,C{2,1}./1000,C{3,1}./1000));                   % Is the point at an obstacle or not

k = 2;

Pf = double( (1/2)*((1./(C{3,1})./1000).^2) );                              % Force Pushing away from the floor                       

Pd = double( (sqrt(((XYZgoal(1) - C{1,1})./1000).^2 +...                           
                   ((XYZgoal(2) - C{2,1})./1000).^2 + ...                   % Weight of the distance to the goal configuration, further out more potential
                   ((XYZgoal(3) - C{3,1})./1000).^2)).^(2*k) );              

disp('half way Pt2')
% Normalize Po, Pf and Pd so they can be added up with weights later
% minPo = min(Po(:));maxPo = max(Po(:));
% Po = (Po - minPo)./(maxPo - minPo);
% minPf = min(Pf(:));maxPf = max(Pf(:));
% Pf = (Pf - minPf)./(maxPf - minPf);
% minPd = min(Pd(:));maxPd = max(Pd(:));
% Pd = (Pd - minPo)./(maxPd - minPd);

% Superimpose all potential fields 
disp('almost there Pt')
cnto = 0; cntd = 1; cntf = 1;                                               % select the weights to assign each potential field
Pt = double(cnto*Po + cntd*Pd + cntf*Pf);

% Get the gradient of each potential field, for the cale of generating a
% vizual
[Po1,Po2,Po3] = gradient(Po); 
[Pf1,Pf2,Pf3] = gradient(Pf);
[Pd1,Pd2,Pd3] = gradient(Pd);
[Pt1,Pt2,Pt3] = gradient(Pt);
clc
disp('done with Potential fields')
pause(0.5)
check = sum([sum(isinf(Pt1(:)));sum(isinf(Pt2(:)));
             sum(isinf(Pt3(:)));sum(isnan(Pt1(:)));
             sum(isnan(Pt2(:)));sum(isnan(Pt3(:)))]);

if check == 1
    return 
end

set(0,'DefaultFigureWindowStyle','docked')

disp('Ploting Potential fields')
figure(5)
scatter3(X(:),Y(:),Z(:),2,Pt(:));colorbar;hold on; grid on
xlabel('X');ylabel('Y');zlabel('Z');
quiver3(C{1,1}(:)./1000,C{2,1}(:)./1000,C{3,1}(:)./1000,Pt1(:),Pt2(:),Pt3(:))
hold off
figure(6)
scatter3(j1g(:),j2g(:),j3g(:),2,Pt(:));colorbar
hold on
quiver3(j1g(:),j2g(:),j3g(:),Pt1(:),Pt2(:),Pt3(:))
xlabel('Joint 1');ylabel('Joint 2');zlabel('Joint 3');
set(0,'DefaultFigureWindowStyle','normal')
hold off
clc
%%
clc
Gama.toll = 1e-3;
Gama.gama = 0.4;
Pfield = Pt;
coordaxes = struct('dir',[]);
coordaxes(1).dir = j1; coordaxes(2).dir = j2; coordaxes(3).dir = j3;
Startj = Jntstrt; Startc = XYZstart;
Endj = Jntgoal; Endc = XYZgoal;
jseq = gradDscnt(Startj,Endj,Startc,Endc,Pfield,coordaxes,Gama,Tf);
XYZ = zeros(3,length(jseq));

for ii = 1:length(jseq)
    XYZ(:,ii) = double(Tf(jseq(1,ii),jseq(2,ii),jseq(3,ii),jseq(4,ii),jseq(5,ii),jseq(6,ii)))./1000;
end
set(0,'DefaultFigureWindowStyle','docked')
figure(7)
show(GP7); hold on
scatter3(XYZ(1,:),XYZ(2,:),XYZ(3,:))
plot3(XYZ(1,:),XYZ(2,:),XYZ(3,:))
hold off
set(0,'DefaultFigureWindowStyle','normal')

%%% Figure
% This next figure looses meaning when seen in higher than 3 dimensions, if
% I keep j5 in the mix its better to suppress this output
% figure(7)
% scatter3(j1g(:),j2g(:),j3g(:),2,P(:))

clc;count = 0;

%%% Adjacency Map - Written in case we wanted to use a graph search
%%% algorithm
% Create a matrix of n by n where every colum is a given node and its
% corresponding rows are 1 of the node touches the nth node or 0 if it does
% not
% Examples: https://stackoverflow.com/questions/30465259/create-an-adjacency-matrix-matlab
% https://stackoverflow.com/questions/3277541/construct-adjacency-matrix-in-matlab
%
% sz = [n1,n2,n3];
% [ii, jj] = adjmatrix(sz, 2, inf, 1);
% Conectivity = sparse(ii, jj, ones(1,numel(ii)), nj, nj);
% Weight(:) = (Pt(ii)+Pt(jj));
% G = digraph(Conectivity);
% G.Edges.Weight = Weight';
% plot(G)

set(0,'DefaultFigureWindowStyle','normal')
%% Trajectory Generation - Motion between Waypoints - In jnt coordinates
% A function of some polynomial trajectory scheme and a search algorithm
% that ensures compliance with motion constrains, here defined as joint
% position, velocity and acceleration, but a more appropiate definition is
% torque Path definition as a series of waypoints accompanied by 
% constraints on their motion. Generate trajectory over segment A,B,C....
% Deine waypoints as [4x4] HTs in cartesian space, define maximum joint
% velocities and acceleration for each joint or for all joints
clc;
set(0,'DefaultFigureWindowStyle','docked')
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
writetimetable(TT,'Trajectory.csv','Delimiter','bar');
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

clear ii

set(0,'DefaultFigureWindowStyle','normal')
%% Notes and good links
% https://stackoverflow.com/questions/51158078/load-stl-file-in-matlab-and-convert-to-a-3d-array
%
%