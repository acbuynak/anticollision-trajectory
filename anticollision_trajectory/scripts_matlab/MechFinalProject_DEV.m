%% Definition
% This program is meant to run using Matlab R2021b and the following% available addons:
%   Robotic System Toolbox (R) Matlab
%   Peter Corke's Robotics System Toolbox for Matlab [https://github.com/petercorke/robotics-toolbox-matlab/tree/bd7a9d75176c660f43fc799b24d838f70b02250c]
%   Peter Corke's Spatial Math [Matlab Addon Interface 2021 version]
%   Simulink and Simulink ROS package [Matlab Addon Interface]
%   Simscape and Simulink Animation [Matlab Addon Interface]
%   
%   
%   
% To the best extent of our abilities most relevant code
% is included in this master file

% Initialization
% Define variables and matrixes needed to solve the porblem

clear;clc;
syms pi real                                                                % Numeric variables
syms x y z real                                                             % Computational Variables
R_E = sym('RE_', [3,3], 'real');                                            % Desired End Effector Orientation
P_E = sym('PE_', [3,1], 'real');                                            % Desired End Effector Position
EE = sym([R_E,P_E;[0,0,0,1]]);                                              % Desired End Effector Transformation

% Ressolution, in radians, of the joint limits for sampling
resj = 10;                                                                  % Resolution of the sampling grids in joint space in degrees
resc = 0.5;                                                                 % Resolution of the sampling grids in cartesian space in distance units
rest = 50;                                                                  % Resolution of the sampling grids in time space in hrz

% Robot URDF Import
% [READ] this file format needs to be converted to URDF before it can be
% used, a method for converting XACRO to URDF using a ROS terminal is
% outlined in https://www.mathworks.com/matlabcentral/answers/422381-how-do-i-import-xacro-files-as-rigid-body-trees-in-robotics-system-toolbox
% 
GP7 = importrobot('gp7.urdf','DataFormat','column');
show(GP7)
viz = interactiveRigidBodyTree(GP7);
% Initialization

% Forward Kinematics of Desired Robot

% Initialize

% Base Frame X Y Z Directions
xb = ([sym('1.0'),0,0]'); 
yb = ([0,sym('1.0'),0]'); 
zb = ([0,0,sym('1.0')]');                 

jntg = struct('s',[],'h',[],'m',[],'I',[],'qb',[],'qc',[],'qf',[],...       % General Initialization of a Joint
             'Rc',[],'Rf',[],'nSens',[],...
'S',[],'Vs',[],'sS',[],'esS',[],'Tp',[],'Tf',[],'Tcm',[],'Vb',[],'Gb',[]);  % These laset ones are meant to be outputs

SwivelBase = jntg; LowerArm = jntg; UpperArm = jntg;
ArmRoll = jntg; WristBend = jntg; ToolFlange = jntg;                        % Initialize as many joints as needed

% Define joint parameters in Null position if a joint is prismatic, define
% its pitch as Inf, else, 0 for no linear motion and a real number for some
% amount of linear motion

% Joint 1 - Swivel Base
SwivelBase.s = zb;                                                          % Screw Axis direction in C axis
SwivelBase.h = 0;                                                           % Pitch of joint
SwivelBase.m = sym('m1','real');                                            % Total mass of the link
SwivelBase.I = sym('I1',[3,1],'real');                                      % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
SwivelBase.qb = sym([ 0, 0, sym('330.0')]');                                % Location the joint wrt. the base frame
SwivelBase.qc = sym('rcom1',[3,1],'real');                                  % Vector from joint to COM in jnt base frame
SwivelBase.qf = sym([ sym('40.0'), 0, sym('330.0')]');                      % Location of the output frame wrt joint base frame
SwivelBase.Rc = sym('Rcom1',[3,3],'real');                                  % Rotation from the joint base frame to COM frame
SwivelBase.Rf = sym('Rf1',[3,3],'real');                                    % Rotation from the joint base frame to output frame
SwivelBase.lmts = sym((-170:resj:170)*(pi/180));                            % Joint Limits [min,max]
% Brings in the STL information as arrays
[F, V, C] = ftread('\robot_support\motoman_gp7_support\meshes\VertexFiles\gp7_s_axis.stl.stl'); 
SwivelBase.F = F; SwivelBase.V = sym(round(V,4)); SwivelBase.C = C;         % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 2 - Lower Arm
LowerArm.s = yb;                                                            % Screw Axis direction in C axis
LowerArm.h = 0;                                                             % Pitch of joint
LowerArm.m = sym('m2','real');                                              % Total mass of the link
LowerArm.I = sym('I2',[3,1],'real');                                        % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
LowerArm.qb = ([ sym('40.0'), 0, sym('330.0')]');                           % Location the joint wrt. the base frame
LowerArm.qc = sym('rcom2',[3,1],'real');                                    % Vector from joint to COM in jnt base frame
LowerArm.qf = ([ sym('40.0'), 0, sym('715.0')]');                           % Location of the output frame wrt joint base frame
LowerArm.Rc = sym('Rcom2',[3,3],'real');                                    % Rotation from the joint base frame to COM frame
LowerArm.Rf = sym('Rf2',[3,3],'real');                                      % Rotation from the joint base frame to output frame
LowerArm.lmts = sym((-65:resj:145)*(pi/180));                               % Joint Limits [min,max]
% Brings in the STL information as arrays
[F, V, C] = ftread('\robot_support\motoman_gp7_support\meshes\VertexFiles\gp7_l_axis.stl.stl');    
LowerArm.F = F; LowerArm.V = sym(round(V,4)); LowerArm.C = C;               % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 3 - Upper Arm
UpperArm.s = -yb;                                                           % Screw Axis direction in base frame
UpperArm.h = 0;                                                             % Pitch of joint
UpperArm.m = sym('m3','real');                                              % Total mass of the link
UpperArm.I = sym('I3',[3,1],'real');                                        % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
UpperArm.qb = ([ sym('40.0'), 0, sym('715.0')]');                           % Location the joint wrt. the base frame
UpperArm.qc = sym('rcom3',[3,1],'real');                                    % Vector from joint to COM in jnt base frame
UpperArm.qf = ([ sym('40.0'), 0, sym('715.0')]');                           % Location of the output frame wrt joint base frame
UpperArm.Rc = sym('Rcom3',[3,3],'real');                                    % Rotation from the joint base frame to COM frame
UpperArm.Rf = sym('Rf3',[3,3],'real');                                      % Rotation from the joint base frame to output frame
UpperArm.lmts = sym((-70:resj:190)*(pi/180));                               % Joint Limits [min,max]
% Brings in the STL information as arrays
[F, V, C] = ftread('\robot_support\motoman_gp7_support\meshes\VertexFiles\gp7_u_axis.stl.stl');    
UpperArm.F = F; UpperArm.V = sym(round(V,4)); UpperArm.C = C;               % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 4 - Arm Roll
ArmRoll.s = -xb;                                                            % Screw Axis direction in base frame
ArmRoll.h = 0;                                                              % Pitch of joint
ArmRoll.m = sym('m4','real');                                               % Total mass of the link
ArmRoll.I = sym('I4',[3,1],'real');                                         % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
ArmRoll.qb = ([ sym('40.0'), 0, sym('715.0')]');                            % Location the joint wrt. the base frame
ArmRoll.qc = sym('rcom4',[3,1],'real');                                     % Vector from joint to COM in jnt base frame
ArmRoll.qf = ([sym('380'), 0, sym('715.0')]');                              % Location of the output frame wrt joint base frame
ArmRoll.Rc = sym('Rcom4',[3,3],'real');                                     % Rotation from the joint base frame to COM frame
ArmRoll.Rf = sym('Rf4',[3,3],'real');                                       % Rotation from the joint base frame to output frame
ArmRoll.lmts = sym((-190:resj:190)*(pi/180));                               % Joint Limits [min,max]
% Brings in the STL information as arrays
[F, V, C] = ftread('\robot_support\motoman_gp7_support\meshes\VertexFiles\gp7_r_axis.stl.stl');  
ArmRoll.F = F; ArmRoll.V = sym(round(V,4)); ArmRoll.C = C;                  % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 5 - Wrist Bend
WristBend.s = -yb;                                                          % Screw Axis direction in base frame
WristBend.h = 0;                                                            % Pitch of joint
WristBend.m = sym('m5','real');                                             % Total mass of the link
WristBend.I = sym('I5',[3,1],'real');                                       % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
WristBend.qb = ([sym('380'), 0, sym('715.0')]');                            % Location the joint wrt. the base frame
WristBend.qc = sym('rcom5',[3,1],'real');                                   % Vector from joint to COM in jnt base frame
WristBend.qf = ([sym('460'), 0, sym('715.0')]');                            % Location of the output frame wrt joint base frame
WristBend.Rc = sym('Rcom5',[3,3],'real');                                   % Rotation from the joint base frame to COM frame
WristBend.Rf = sym('Rf5',[3,3],'real');                                     % Rotation from the joint base frame to output frame
WristBend.lmts = sym((-135:resj:135)*(pi/180));                             % Joint Limits [min,max]
% Brings in the STL information as arrays
[F, V, C] = ftread('\robot_support\motoman_gp7_support\meshes\VertexFiles\gp7_b_axis.stl.stl');                                   
WristBend.F = F; WristBend.V = sym(round(V,4)); WristBend.C = C;            % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 6 - Tool Flange
ToolFlange.s = -xb;                                                         % Screw Axis direction in base frame
ToolFlange.h = 0;                                                           % Pitch of joint
ToolFlange.m = sym('m6','real');                                            % Total mass of the link
ToolFlange.I = sym('I6',[3,1],'real');                                      % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
ToolFlange.qb = ([sym('460'), 0, sym('715.0')]');                           % Location the joint wrt. the base frame
ToolFlange.qc = sym('rcom6',[3,1],'real');                                  % Vector from joint to COM in jnt base frame
ToolFlange.qf = ([sym('460'), 0, sym('715.0')]');                           % Location of the output frame wrt joint base frame
ToolFlange.Rc = sym('Rcom6',[3,3],'real');                                  % Rotation from the joint base frame to COM frame
ToolFlange.Rf = sym('Rf6',[3,3],'real');                                    % Rotation from the joint base frame to output frame
ToolFlange.lmts = sym((-360:resj:360)*(pi/180));                            % Joint Limits [min,max]
% Brings in the STL information as arrays
[F, V, C] = ftread('\robot_support\motoman_gp7_support\meshes\VertexFiles\gp7_t_axis.stl.stl');
ToolFlange.F = F; ToolFlange.V = sym(round(V,4)); ToolFlange.C = C;         % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% EE frame in null configuration
R_M = sym([zb,yb,-xb]);                                                     % Desired End Effector Orientation
P_M = ([sym('460'), 0, sym('715.0')]');                                     % Desired End Effector Position
M_M =sym([R_M,P_M;[0,0,0,1]]);                                              % Composed Null Configuration EE Frame wrt base frame

% Define the order of the joints
jnt = [SwivelBase,LowerArm,UpperArm,ArmRoll,WristBend,ToolFlange,[]];       % Set the order of the joints in the robot

% Perform Forward Kinematics
[robot,q,qd,qdd] = FrwKin(jnt,M_M);

% Redefine some things for direct use later
% Define the numbers that need substituting  and their numerical version,
% idk if vpa is bettee but i like this way
% inputs
subsi = {'80.0','-80.0','340.0','-340.0','40.0','-40.0','385.0','-385.0','340.0','-340.0','330.0','-330.0','715.0','-715.0','-1.0','1.0','-0.5','0.5','160.0','-160.0','680.0','-680.0','770.0','-770.0','320.0','-320.0','1360.0','-1360.0','380.0','-380.0','1430.0','-1430.0','640.0','-640.0','760.0','-760.0'};
% Outputs
subso = { 80,    -80,    340,    -340,    40,    -40,    385,    -385,    340,    -340,    330,    -330,    715,    -715,    -1,    1,    -0.5,  0.5,  160,    -160,    680,    -680,    770,    -770,    320.    -320,    1360,    -1360,    380,    -380,    1430,    -1430,    640,    -640,    760,    -760};
Tbe = simplify(subs(robot.T.be,subsi,subso));Teb = simplify(subs(robot.T.eb,subsi,subso));
Js = robot.Js;Jb = robot.Jb;

for ii = 1:length(robot.Js)
    Js(:,ii) = simplify(collect(subs(Js(:,ii),subsi,subso),[sin(q)',cos(q)']));
    Jb(:,ii) = simplify(collect(subs(Jb(:,ii),subsi,subso),[sin(q)',cos(q)']));
    robot.Jnts(ii).Tb = simplify(collect(subs(robot.Jnts(ii).Tb,subsi,subso),[sin(q)',cos(q)']));
    robot.Jnts(ii).Tb = symfun(robot.Jnts(ii).Tb,(q'));
    robot.Jnts(ii).esS = simplify(subs(robot.Jnts(ii).esS,subsi,subso));
end

    A = ((Jb)*(transpose(Jb))); Av = (Js(4:6,:))*transpose(Js(4:6,:)); Aw = (Js(1:3,:))*transpose(Js(1:3,:));

for ii = 1:3
    Av(:,ii) = simplify(Av(:,ii));    
    Aw(:,ii) = simplify(Aw(:,ii));
end

Af = symfun(A,q'); Avf = symfun(Av,q'); Awf = symfun(Aw,q');
Tbef = symfun(Tbe,(q')); Tf = symfun(Tbe(1:3,4),(q'));

clear pi;
clear SwivelBase LowerArm UpperArm ArmRoll WristBend ToolFlange

% Some pretty visuals
figure(1)
subplot(2,3,1)
patch('Faces', jnt(1).F, 'Vertices', eval(jnt(1).V), 'FaceVertexCData', (1:size(jnt(1).F,1))', 'FaceColor', 'flat');
grid on
subplot(2,3,2)
patch('Faces', jnt(2).F, 'Vertices', eval(jnt(2).V), 'FaceVertexCData', (1:size(jnt(2).F,1))', 'FaceColor', 'flat');
grid on
subplot(2,3,3)
patch('Faces', jnt(3).F, 'Vertices', eval(jnt(3).V), 'FaceVertexCData', (1:size(jnt(3).F,1))', 'FaceColor', 'flat');
grid on
subplot(2,3,4)
patch('Faces', jnt(4).F, 'Vertices', eval(jnt(4).V), 'FaceVertexCData', (1:size(jnt(4).F,1))', 'FaceColor', 'flat');
grid on
subplot(2,3,5)
patch('Faces', jnt(5).F, 'Vertices', eval(jnt(5).V), 'FaceVertexCData', (1:size(jnt(5).F,1))', 'FaceColor', 'flat');
grid on
subplot(2,3,6)
patch('Faces', jnt(6).F, 'Vertices', eval(jnt(6).V), 'FaceVertexCData', (1:size(jnt(6).F,1))', 'FaceColor', 'flat');
grid on

clc
disp('Done Section 1')

%% Symbolicaly this next operation takes a big computational toll, it is
% usefull since the total potential field benefits from the manipulability
% cost, It is included here for completeness but i dont recomend doing it
% dA = det(A);

% Define the Potential Fields
% Obstacle Potential Fields

LIMS = [-3, 3, -3,3,-2,5];

Floor(x,y,z) =  piecewise(z < 0,1,0);

Ob1(x,y,z) = piecewise(  0 < x & x < 1 &...
                         0 < y & y < 1 &...
                         0 < z & z < 1,9,0);

Ob2(x,y,z) = piecewise(0 < x & x < 2 &...
                      -1 < y & y < 1 &...
                       0 < z & z < 2,9,0);
Obt = Floor+Ob1+Ob2;

% View Obstacle potential fileds
figure(2)
hold off
fimplicit3(Obt,LIMS,"MeshDensity",35);
axis(LIMS)
xlabel('X');ylabel('Y');zlabel('Z');
title('Obstacle Potential Field')
grid on
hold on

% Do Forward Kinematics and get weights
n1 = numel(robot.Jnts(1).lmts);n2 = numel(robot.Jnts(2).lmts);
n3 = numel(robot.Jnts(3).lmts);n4 = numel(robot.Jnts(4).lmts);
n5 = numel(robot.Jnts(5).lmts);n6 = numel(robot.Jnts(6).lmts);

jspace = nan(n1,n2,n3);
nj = numel(jspace);

j1 = subs(robot.Jnts(1).lmts,'pi',pi);j2 = subs(robot.Jnts(2).lmts,'pi',pi);
j3 = subs(robot.Jnts(3).lmts,'pi',pi);j4 = subs(robot.Jnts(4).lmts,'pi',pi);
j5 = subs(robot.Jnts(5).lmts,'pi',pi);j6 = subs(robot.Jnts(6).lmts,'pi',pi);

[j1g,j2g,j3g] = ndgrid(j1,j2,j3);

disp([n1,n2,n3,n4,n5,n6,nj]);

% This step takes a bit to process but its actually really good all things
% considered, only for 4 joints
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

Xgoal = 0; Ygoal = 0; Zgoal = 0;

C = Tf(j1g,j2g,j3g,zeros(size(j1g)),zeros(size(j1g)),zeros(size(j1g)));
Po = Obt(C{1,1}./1000,C{2,1}./1000,C{3,1}./1000);                           % is the point at an obstacle or not
Pd = (Xgoal - C{1,1}./1000).^2  +  (Ygoal - C{2,1}./1000).^2  +...          % Weight of the distance to the goal configuration
     (Zgoal - C{3,1}./1000).^2 ;

Pt = Po + Pd;Pt = double(Pt);
[Pj1,Pj2,Pj3] = gradient(Pt);

figure(3)
scatter3(C{1,1}(:),C{2,1}(:),C{3,1}(:),2,Pt(:))
figure(4)
scatter3(j1g(:),j2g(:),j3g(:),2,P(:))

% This next figure looses meaning when seen in higher than 3 dimensions, if
% I keep j5 in the mix its better to suppress this output
% figure(4)
% scatter3(j1g(:),j2g(:),j3g(:),2,P(:))

clc;count = 0;

%% Adjacency Map
% Create a matrix of n by n where every colum is a given node and its
% corresponding rows are 1 of the node touches the nth node or 0 if it does
% not
% Examples: https://stackoverflow.com/questions/30465259/create-an-adjacency-matrix-matlab
% https://stackoverflow.com/questions/3277541/construct-adjacency-matrix-in-matlab
%

sz = [n1,n2,n3];
[ii, jj] = adjmatrix(sz, 2, inf, 1);
A = sparse(ii, jj, ones(1,numel(ii)), nj, nj);
Weight(:) = (Pt(ii)+Pt(jj));
G = digraph(A);
G.Edges.Weight = Weight';
% plot(G)

%% Trajectory Generation - Motion between Waypoints - In jnt coordinates
% A function of some polynomial trajectory scheme and a search algorithm
% that ensures compliance with motion constrains, here defined as joint
% position, velocity and acceleration, but a more appropiate definition is
% torque
% Path definition as a series of waypoints accompanied by constraints on
% their motion. Generate trajectory over segment A,B,C....
% Deine waypoints as [4x4] HTs in cartesian space, define maximum joint
% velocities and acceleration for each joint or for all joints
clc;
path_gen = struct('Segment',[],'Jnt',[],'tvct',[]); 
sgmnt_gen = struct('Start',[],'End',[],'Vwmax',[],'Awmax',[],'t0',0,...
    'tf',[]);
% Initialize # of paths and segments
path = path_gen;segment1 = sgmnt_gen;segment2 = sgmnt_gen;

% Pick a joint configuration just to test - this ideally comes from the
% waypoints

QtstA = sym([   0 ,  0 ,   0 ,  0 ,  0 ,   0 ]);
QtstB = sym([ pi/2,pi/2, pi/2,pi/2,pi/2, pi/2]);
QtstC = sym([-pi/2, pi ,-pi/2,pi/2,pi  ,-pi/2]);

% update the segment #
Ap = Tbef(QtstA(1),QtstA(2),QtstA(3),QtstA(4),QtstA(5),QtstA(6));
Vwmax1 = 1  ;Awmax1 = 0.5;
Bp = Tbef(QtstB(1),QtstB(2),QtstB(3),QtstB(4),QtstB(5),QtstB(6));
Vwmax2 = 0.5;Awmax2 = 1;
Cp = Tbef(QtstC(1),QtstC(2),QtstC(3),QtstC(4),QtstC(5),QtstC(6));

segment1.Start = QtstA';
segment1.End = QtstB';
segment1.Vwmax= Vwmax1;
segment1.Awmax= Awmax1;

segment2.Start = QtstB';
segment2.End = QtstC';
segment2.Vwmax= Vwmax2;
segment2.Awmax= Awmax2;

% A path is a sequence of segments

PathObj.Segment = [segment1,segment2,[],[]];

% Create the trajectory, mified from original code to not need frdw/inv
% kinematics

hrz = rest; 

[TrjObj,TT] = jnttrjgn(PathObj,hrz);
TT.Properties.VariableUnits = {'rads','rads/sec','rads/(sec^2)'};
writetimetable(TT,'Trajectory.csv','Delimiter','bar');

%% vizualisation
for ii = 1:length(TrjObj.tvct)
    config = [TrjObj.Jnt.jnt(:,ii)]
    show(GP7,config);
    pause(1/rest)
end
%% Included Functions

function AdT = Adjoint(T)
% from lynch

[R, p] = TransToRp(T);
AdT = [R, zeros(3); VecToso3(p) * R, R];
end


function so3mat = VecToso3(omg)
% from lynch
so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
end


function [R, p] = TransToRp(T)
% From Lynch

R = T(1: 3, 1: 3);
p = T(1: 3, 4);
end


function skew = vect2skew(vect)
% takes a 3 dimensional vector and converts it to its skew symetric
% representation
    skew = [    0   ,-vect(3), vect(2);
             vect(3),   0    ,-vect(1);
            -vect(2), vect(1),   0];
end


function skew = screw2skew(screw)
% 6x1 screw vector to skre symetric representation
    skew = [vect2skew(screw(1:3)),screw(4:6);0,0,0,0];
end

% Inverse Homogeneous Transform
function iHT = invHT(HT)
% takes a homogeneous transformation T and calculates its inverse using
% properties of T and not inv(T)
    
    R = HT(1:3,1:3);
    P = HT(1:3,4);
    
    iHT = [transpose(R),-transpose(R)*P;0,0,0,1];

end

% Forward Kineamtics
function [robot,q,qd,qdd] = FrwKin(jnt,M)
% Takes the key components of a robot, its joints and null frame, and
% creates said robot while also expanding the joint information to include
% other data
njnt = length(jnt);                                                         % Get the number of joints in the robot
Tp = eye(4);                                                                % Initialize Transformation Buffer

% Initilize Variable Vectors
q = sym('q', [njnt,1],'real');                                              % Position Variables
qd = sym('qd', [njnt,1],'real');                                            % Velocity Variables
qdd = sym('qdd', [njnt,1],'real');                                          % Acceleration Variables

% Initialize some data containers depending on 
robot = struct("Jnts",[],"Js",sym(zeros(6,njnt)),"Jb",sym(zeros(6,njnt)),"T",[]);   
temp = struct([]);
Js = sym(zeros(6,njnt));                                                    % Initialize the Jacobian

% Takes a joint series as a structure containing (I Inputs and gives O
% outputs):
% jnt = struct('s',[I],'h',[I],'m',[I],'I',[I],'qb',[I],'qc',[I],'qf',[I],... 
%              'Rc',[I],'Rf',[I],'nSens',[I],...
% 'S',[O],'Vs',[O],'sS',[O],'esS',[O],'Tp',[O],'Tf',[O],'Tcm',[O],'Vb',[O],'Gb',[O]);

    for ii = 1:njnt
%         count = 1;
        if isinf(jnt(ii).h) % Checks if this is a prismatic joint and creates the angular and linear twist of the axis depending on this
             w = sym([0,0,0]');
             v = sym([jnt(ii).s(1); ...
                      jnt(ii).s(2); ...
                      jnt(ii).s(3)]);
        else
            w =  sym([jnt(ii).s(1); ...
                      jnt(ii).s(2); ...
                      jnt(ii).s(3)]);
            v =  sym(-cross(w,[jnt(ii).qb(1);jnt(ii).qb(2);jnt(ii).qb(3)])...
                  +((jnt(ii).h)*(w)));
        end

        temp(ii).S = [w;v];                                                 % Creates the screw axis in space frame
        temp(ii).Sv = temp(ii).S*q(ii,1);                                   % Creates the screw axis in space frame with the variable
        temp(ii).Vs = temp(ii).S*qd(ii,1);                                  % Creates the screw axis in space frame with the variable
        temp(ii).sS = screw2skew(temp(ii).Sv);                              % Represents the screw axis in skew symetric form
        temp(ii).esS = expm(temp(ii).sS);                                   % Takes the matrix exponential of the skew symetric screw axis 
        Tp = collect(Tp*temp(ii).esS,[sin(q)',cos(q)']);                                     % Collects the value of the exponential for future use across all joints
        temp(ii).Tp = subs((Tp),{'-1.0','1.0','-0.5','0.5'},{-1,1,-0.5,0.5});                                                   % Stores the previous value for this joint
        temp(ii).Tb = (Tp * [eye(3),[jnt(ii).qb(1);
                                     jnt(ii).qb(2);
                                     jnt(ii).qb(3)];...
                             [0,0,0,1]]);                                   % Transformation from the base frame to the base of the joint     
        temp(ii).Tb = subs((temp(ii).Tb),{'-1.0','1.0','-0.5','0.5'},{-1,1,-0.5,0.5});
        temp(ii).Tcm = (Tp * [jnt(ii).Rc,[jnt(ii).qc(1);
                                          jnt(ii).qc(2);
                                          jnt(ii).qc(3)];...
                                    [0,0,0,1]]);                            % Transformation from the base to the Center of Mass
        temp(ii).Tcm = subs((temp(ii).Tcm),{'-1.0','1.0','-0.5','0.5'},{-1,1,-0.5,0.5});
        temp(ii).Tf = (Tp * [jnt(ii).Rf,[jnt(ii).qf(1);
                                         jnt(ii).qf(2);
                                         jnt(ii).qf(3)];...
                                  [0,0,0,1]]);                              % Transformation from the base frame to the output of the joint
        temp(ii).Tf = subs((temp(ii).Tf),{'-1.0','1.0','-0.5','0.5'},{-1,1,-0.5,0.5});
        temp(ii).lmts = jnt(ii).lmts;
        % Another loop (for/end) needs to be created here to calculate a
        % series of homogeneous transformations from the base to the
        % n-number of sensor points we choose to use
        %         [w,l] = size(jnt(ii).rsns);                                         % Get the amount of points in the sensor array
        %         
        %         for jj = 1:w
        %             jnt(ii).rsns(:,jj)
        %         end
        
        temp(ii).V = jnt(ii).V; temp(ii).C = jnt(ii).C; temp(ii).F = jnt(ii).F;
        
        if ii == 1
            Js(:,ii) = temp(ii).S;
        else
            Js(:,ii) = Adjoint(temp(ii-1).Tp)*temp(ii).S;
        end

    end
    %temp
    %pause

    robot.Jnts = temp;
    robot.T.be = subs((Tp*M),{'-1.0','1.0','-0.5','0.5'},{-1,1,-0.5,0.5});                              % Transformation from the base frame to the EE frame
    robot.T.eb = invHT(robot.T.be);                                         % Transformation from the EE frame to the base frame
    robot.Js = collect(subs((Js),{'-1.0','1.0','-0.5','0.5'},{-1,1,-0.5,0.5}),[sin(q)',cos(q)']);                                                          % Store the Space Jacobian
    robot.Jb = collect(subs((Adjoint(robot.T.eb)*Js),{'-1.0','1.0','-0.5','0.5'},{-1,1,-0.5,0.5}),[sin(q)',cos(q)']);                         % Calculate and store the Body Jacobian
    % Calcualte EE twist in space and body frame
    robot.Vs = robot.Js*qd; robot.Vb = robot.Jb*qd;
end

% Trajectory Generation
function [PathObj,TT] = jnttrjgn(PathObj,hrz)
% path_gen = struct('Segment',[],'Jnt',[],'tvct',[]); 
% sgmnt_gen = struct('Start',[],'End',[],'Vwmax',[],'Awmax',[],'t0',[],...
%                    'tf',[]);  
n = length(PathObj.Segment);
njnt = size([PathObj.Segment(1).Start],1);

    for ii = 1:n

        Start(:,1) = PathObj.Segment(ii).Start;
        End(:,1) = PathObj.Segment(ii).End;
        Vmax = PathObj.Segment(ii).Vwmax;
        Amax = PathObj.Segment(ii).Awmax;
        ta = Vmax/Amax; tf = 2*ta;

        PathObj.Jnt.jnt(njnt,hrz*tf) = Inf;
        PathObj.Jnt.jntd(njnt,hrz*tf) = Inf;
        PathObj.Jnt.jntdd(njnt,hrz*tf) = Inf;
        
        s.s = PathObj.Jnt.jnt;
        s.sd = PathObj.Jnt.jntd;
        s.sdd = PathObj.Jnt.jntdd;

        s.T = [];

        while max(max(abs(s.sd))) > Vmax || max(max(abs(s.sdd))) > Amax
            %disp(['searching trj.'])
            N = size(s.s,2);
            s = LSPB(double([Start,End]),tf,N);
            
            if max(max(abs(s.sd))) > Vmax || max(max(abs(s.sdd))) > Amax
                tf = tf + 0.1;
            end

        figure(80+ii)
        subplot(3,1,1)
        plot(s.T,s.s(1,:),'r*',s.T,s.s(2,:),'b*',s.T,s.s(3,:),'g*',s.T,s.s(4,:),'r-',s.T,s.s(5,:),'b-',s.T,s.s(6,:),'g-')
        legend('q1','q2','q3','q4','q5','q6')
        grid on
        subplot(3,1,2)
        plot(s.T,s.sd(1,:),'r*',s.T,s.sd(2,:),'b*',s.T,s.sd(3,:),'g*',s.T,s.sd(4,:),'r-',s.T,s.sd(5,:),'b-',s.T,s.sd(6,:),'g-')
        legend('q1','q2','q3','q4','q5','q6')
        grid on
        subplot(3,1,3)
        plot(s.T,s.sdd(1,:),'r*',s.T,s.sdd(2,:),'b*',s.T,s.sdd(3,:),'g*',s.T,s.sdd(4,:),'r-',s.T,s.sdd(5,:),'b-',s.T,s.sdd(6,:),'g-')
        legend('q1','q2','q3','q4','q5','q6')
        grid on
        pause(0.1);
        
        end

        PathObj.Segment(ii).jnt = s.s;
        PathObj.Segment(ii).jntd = s.sd;
        PathObj.Segment(ii).jntdd = s.sdd;
        PathObj.Segment(ii).tvct = s.T;

        if ii == 1
            PathObj.Jnt.jnt = s.s;
            PathObj.Jnt.jntd = s.sd;
            PathObj.Jnt.jntdd = s.sdd;
            PathObj.tvct = s.T;
        else
            PathObj.Jnt.jnt = [[PathObj.Jnt.jnt],[s.s]];
            PathObj.Jnt.jntd = [PathObj.Jnt.jntd,s.sd];
            PathObj.Jnt.jntdd = [PathObj.Jnt.jntdd,s.sdd];
            PathObj.tvct = [PathObj.tvct,s.T(1:end)+PathObj.tvct(end)];
        end

        figure(80+njnt)
        subplot(3,1,1)
        plot(PathObj.tvct,PathObj.Jnt.jnt(1,:),'r-',...
             PathObj.tvct,PathObj.Jnt.jnt(2,:),'b-',...
             PathObj.tvct,PathObj.Jnt.jnt(3,:),'g-',...
             PathObj.tvct,PathObj.Jnt.jnt(4,:),'y-',...
             PathObj.tvct,PathObj.Jnt.jnt(5,:),'p-',...
             PathObj.tvct,PathObj.Jnt.jnt(6,:),'o-')
        legend('q1','q2','q3','q4','q5','q6')
        ylabel('Position Radians')
        xlabel('Time seconds')
        grid on
        subplot(3,1,2)
        plot( PathObj.tvct,PathObj.Jnt.jntd(1,:),'r-',...
              PathObj.tvct,PathObj.Jnt.jntd(2,:),'b-',...
              PathObj.tvct,PathObj.Jnt.jntd(3,:),'g-',...
              PathObj.tvct,PathObj.Jnt.jntd(4,:),'y-',...
              PathObj.tvct,PathObj.Jnt.jntd(5,:),'p-',...
              PathObj.tvct,PathObj.Jnt.jntd(6,:),'o-')
        legend('qd1','qd2','qd3','qd4','qd5','qd6')
        ylabel('Velocity')
        xlabel('Time seconds')
        grid on
        subplot(3,1,3)
        plot( PathObj.tvct,PathObj.Jnt.jntdd(1,:),'r-',...
              PathObj.tvct,PathObj.Jnt.jntdd(2,:),'b-',...
              PathObj.tvct,PathObj.Jnt.jntdd(3,:),'g-',...
              PathObj.tvct,PathObj.Jnt.jntdd(4,:),'y-',...
              PathObj.tvct,PathObj.Jnt.jntdd(5,:),'p-',...
              PathObj.tvct,PathObj.Jnt.jntdd(6,:),'o-')
        legend('qdd1','qdd2','qdd3','qdd4','qdd5','qdd6')
        ylabel('Acceleration')
        xlabel('Time seconds')
        grid on

    end

TT = timetable(PathObj.Jnt.jnt',PathObj.Jnt.jntd',PathObj.Jnt.jntdd','SampleRate',hrz);
TT.Properties.VariableNames = {'Position','Velocity','Acceleration'};

end

% Timescaling
function s = LSPB(W,tf,N)
% given the number of samples generates timescaling vectors s sd and sdd
    [s,sd,sdd,T] = trapveltraj(W,N,'EndTime',tf);
    s = struct('s',s,'sd',sd,'sdd',sdd,'T',T);
end

% Read STL
function [fout, vout, cout] = ftread(filename)
% Reads CAD STL ASCII files, which most CAD programs can export.
% Used to create Matlab patches of CAD 3D data.
% Returns a vertex list and face list, for Matlab patch command.
% 
% filename = 'hook.stl';  % Example file.
%
    fid=fopen(filename, 'r'); %Open the file, assumes STL ASCII format.
    if fid == -1 
        error('File could not be opened, check name or path.')
    end

% Render files take the form:
%   
%solid BLOCK
%  color 1.000 1.000 1.000
%  facet
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%    outer loop
%      vertex 5.000000e-01 -5.000000e-01 -5.000000e-01
%      vertex -5.000000e-01 -5.000000e-01 -5.000000e-01
%      vertex -5.000000e-01 5.000000e-01 -5.000000e-01
%    endloop
% endfacet
% The first line is object name, then comes multiple facet and vertex lines.
% A color specifier is next, followed by those faces of that color, until
% next color line.

    CAD_object_name = sscanf(fgetl(fid), '%*s %s');  %CAD object name, if needed.
                                                     %Some STLs have it, some don't.   
    vnum=0;       %Vertex number counter.
    report_num=0; %Report the status as we go.
    VColor = 0;
    %
    while feof(fid) == 0                    % test for end of file, if not then do stuff
        tline = fgetl(fid);                 % reads a line of data from file.
        fword = sscanf(tline, '%s ');       % make the line a character string
    % Check for color
        if strncmpi(fword, 'c',1) == 1;    % Checking if a "C"olor line, as "C" is 1st char.
           VColor = sscanf(tline, '%*s %f %f %f'); % & if a C, get the RGB color data of the face.
        end                                % Keep this color, until the next color is used.
        if strncmpi(fword, 'v',1) == 1;    % Checking if a "V"ertex line, as "V" is 1st char.
           vnum = vnum + 1;                % If a V we count the # of V's
           report_num = report_num + 1;    % Report a counter, so long files show status
           if report_num > 249;
               % disp(sprintf('Reading vertix num: %d.',vnum));
               report_num = 0;
           end
           v(:,vnum) = sscanf(tline, '%*s %f %f %f'); % & if a V, get the XYZ data of it.
           c(:,vnum) = VColor;              % A color for each vertex, which will color the faces.
        end                                 % we "*s" skip the name "color" and get the data.                                          
    end
    %   Build face list; The vertices are in order, so just number them.
    %
    fnum = vnum/3;      %Number of faces, vnum is number of vertices.  STL is triangles.
    flist = 1:vnum;     %Face list of vertices, all in order.
    F = reshape(flist, 3,fnum); %Make a "3 by fnum" matrix of face list data.
    %
    %   Return the faces and vertexs.
    %
    fout = F';  %Orients the array for direct use in patch.
    vout = v';  % "
    cout = c';
    %
    fclose(fid);
end

% Make Adjecency Matrix
function [ii, jj] = adjmatrix(sz, r, p, st)
    % https://github.com/shaibagon/GCMex/blob/master/sparse_adj_matrix.m
    % Construct sparse adjacency matrix (provides ii and jj indices into the
    % matrix)
    %
    % Usage:
    %   [ii jj] = sparse_adj_matrix(sz, r, p, st)
    %
    % inputs:
    %   sz - grid size (determine the number of variables n=prod(sz), and the
    %        geometry)
    %   r  - the radius around each point for which edges are formed
    %   p  - in what p-norm to measure the r-ball, can be 1,2 or 'inf'
    %   st - integer step size in making the neighborhood: st=1, full neighborhood, 
    %        for r > st > 1 the neighborhood is uniformly sampled
    %
    % outputs
    %   ii, jj - linear indices into adjacency matrix (for each pair (m,n)
    %   there is also the pair (n,m))
    %
    % How to construct the adjacency matrix?
    % >> A = sparse(ii, jj, ones(1,numel(ii)), prod(sz), prod(sz));
    %
    %
    % Example:
    % >> [ii jj] = sparse_adj_matrix([10 20], 1, inf);
    % construct indices for 200x200 adjacency matrix for 8-connect graph over a
    % grid of 10x20 nodes.
    % To visualize the graph:
    % >> [r c]=ndgrid(1:10,1:20);
    % >> A = sparse(ii, jj, ones(1,numel(ii)), 200, 200);;
    % >> gplot(A, [r(:) c(:)]);
    %
    %
    %
    % Copyright (c) Bagon Shai
    % Department of Computer Science and Applied Mathmatics
    % Wiezmann Institute of Science
    % http://www.wisdom.weizmann.ac.il/
    % 
    % Permission is hereby granted, free of charge, to any person obtaining a copy
    % of this software and associated documentation files (the "Software"), to deal
    % in the Software without restriction, subject to the following conditions:
    % 
    % The above copyright notice and this permission notice shall be included in 
    % all copies or substantial portions of the Software.
    %
    % The Software is provided "as is", without warranty of any kind.
    %
    % Sep. 2010
    %
    
    if nargin < 4
        st = 1;
    end
    
    % number of variables
    n = prod(sz);
    % number of dimensions
    ndim = numel(sz);
    
    tovec = @(x) x(:);
    N=cell(ndim,1);
    I=cell(ndim,1);
    % construct the neighborhood
    fr=floor(r);
    for di=1:ndim
    %     tmp = unique( round( logspace(0, log10(fr), st)-1 ) );    
        N{di}=  -fr:st:fr;
        I{di}=1:sz(di);
    end
    [N{1:ndim}]=ndgrid(N{:});
    [I{1:ndim}]=ndgrid(I{:});
    N = cellfun(tovec, N, 'UniformOutput',false);
    N=[N{:}];
    I = cellfun(tovec, I, 'UniformOutput',false);
    I=[I{:}];
    
    % compute N radius according to p
    switch lower(p)
        case {'1','l1',1}
            R = sum(abs(N),2);
        case {'2','l2',2}
            R = sum(N.*N,2);
            r=r*r;
        case {'inf',inf}
            R = max(abs(N),[],2);
        otherwise
            error('sparse_adj_matrix:norm_type','Unknown norm p (should be either 1,2 or inf');
    end
    N = N(R<=r+eps,:);
    
    % "to"-index (not linear indices)
    ti = bsxfun(@plus, permute(I,[1 3 2]), permute(N, [3 1 2]));
    sel = all(ti >= 1, 3) & all( bsxfun(@le, ti, permute(sz, [1 3 2])), 3);
    csz = cumprod([1 sz(1:(ndim-1))]);
    jj = sum( bsxfun(@times, ti-1, permute(csz, [1 3 2])), 3)+1; % convert to linear indices
    ii = repmat( (1:n)', [1 size(jj,2)]);
    jj = jj(sel(:));
    ii = ii(sel(:));
end


%% Notes and good links
% https://stackoverflow.com/questions/51158078/load-stl-file-in-matlab-and-convert-to-a-3d-array
%
%
%
%
%
%
%
%
%
%