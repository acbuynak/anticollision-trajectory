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
%% Robot URDF Import
% [READ] this file format needs to be converted to URDF before it can be
% used, a method for converting XACRO to URDF using a ROS terminal is
% outlined in https://www.mathworks.com/matlabcentral/answers/422381-how-do-i-import-xacro-files-as-rigid-body-trees-in-robotics-system-toolbox
% 
% GP7urdf = urdfparse("GP7 Suppot Files\urdf\gp7.xacro");
%% Initialization
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

% Forward Kinematics of Desired Robot

% Initialize

% Base Frame X Y Z Directions
xb = ([sym('1.0'),sym('0.0'),sym('0.0')]'); 
yb = ([sym('0.0'),sym('1.0'),sym('0.0')]'); 
zb = ([sym('0.0'),sym('0.0'),sym('1.0')]');                 

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
SwivelBase.h =  sym('0.0');                                                 % Pitch of joint
SwivelBase.m = sym('m1','real');                                            % Total mass of the link
SwivelBase.I = sym('I1',[3,1],'real');                                      % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
SwivelBase.qb = sym([ sym('0.0'), sym('0.0'), sym('330.0')]');              % Location the joint wrt. the base frame
SwivelBase.qc = sym('rcom1',[3,1],'real');                                  % Vector from joint to COM in jnt base frame
SwivelBase.qf = sym([ sym('40.0'), sym('0.0'), sym('330.0')]');             % Location of the output frame wrt joint base frame
SwivelBase.Rc = sym('Rcom1',[3,3],'real');                                  % Rotation from the joint base frame to COM frame
SwivelBase.Rf = sym('Rf1',[3,3],'real');                                    % Rotation from the joint base frame to output frame
SwivelBase.lmts = sym((-170:resj:170)*(pi/180));                            % Joint Limits [min,max]
[F, V, C] = ftread('gp7_s_axis.stl.stl');                                   % Brings in the STL information as arrays
SwivelBase.F = F; SwivelBase.V = sym(round(V,4)); SwivelBase.C = C;         % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 2 - Lower Arm
LowerArm.s = yb;                                                            % Screw Axis direction in C axis
LowerArm.h =  sym('0.0');                                                   % Pitch of joint
LowerArm.m = sym('m2','real');                                              % Total mass of the link
LowerArm.I = sym('I2',[3,1],'real');                                        % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
LowerArm.qb = ([ sym('40.0'), sym('0.0'), sym('330.0')]');                  % Location the joint wrt. the base frame
LowerArm.qc = sym('rcom2',[3,1],'real');                                    % Vector from joint to COM in jnt base frame
LowerArm.qf = ([ sym('40.0'), sym('0.0'), sym('715.0')]');                  % Location of the output frame wrt joint base frame
LowerArm.Rc = sym('Rcom2',[3,3],'real');                                    % Rotation from the joint base frame to COM frame
LowerArm.Rf = sym('Rf2',[3,3],'real');                                      % Rotation from the joint base frame to output frame
LowerArm.lmts = sym((-65:resj:145)*(pi/180));                               % Joint Limits [min,max]
[F, V, C] = ftread('gp7_l_axis.stl.stl');                                   % Brings in the STL information as arrays
LowerArm.F = F; LowerArm.V = sym(round(V,4)); LowerArm.C = C;               % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 3 - Upper Arm
UpperArm.s = yb;                                                            % Screw Axis direction in base frame
UpperArm.h =  sym('0.0');                                                   % Pitch of joint
UpperArm.m = sym('m3','real');                                              % Total mass of the link
UpperArm.I = sym('I3',[3,1],'real');                                        % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
UpperArm.qb = ([ sym('40.0'), sym('0.0'), sym('715.0')]');                  % Location the joint wrt. the base frame
UpperArm.qc = sym('rcom3',[3,1],'real');                                    % Vector from joint to COM in jnt base frame
UpperArm.qf = ([ sym('40.0'), sym('0.0'), sym('715.0')]');                  % Location of the output frame wrt joint base frame
UpperArm.Rc = sym('Rcom3',[3,3],'real');                                    % Rotation from the joint base frame to COM frame
UpperArm.Rf = sym('Rf3',[3,3],'real');                                      % Rotation from the joint base frame to output frame
UpperArm.lmts = sym((-70:resj:190)*(pi/180));                               % Joint Limits [min,max]
[F, V, C] = ftread('gp7_u_axis.stl.stl');                                   % Brings in the STL information as arrays
UpperArm.F = F; UpperArm.V = sym(round(V,4)); UpperArm.C = C;               % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 4 - Arm Roll
ArmRoll.s = xb;                                                             % Screw Axis direction in base frame
ArmRoll.h =  sym('0.0');                                                    % Pitch of joint
ArmRoll.m = sym('m4','real');                                               % Total mass of the link
ArmRoll.I = sym('I4',[3,1],'real');                                         % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
ArmRoll.qb = ([ sym('40.0'), sym('0.0'), sym('715.0')]');                   % Location the joint wrt. the base frame
ArmRoll.qc = sym('rcom4',[3,1],'real');                                     % Vector from joint to COM in jnt base frame
ArmRoll.qf = ([sym('380'), sym('0.0'), sym('715.0')]');                     % Location of the output frame wrt joint base frame
ArmRoll.Rc = sym('Rcom4',[3,3],'real');                                     % Rotation from the joint base frame to COM frame
ArmRoll.Rf = sym('Rf4',[3,3],'real');                                       % Rotation from the joint base frame to output frame
ArmRoll.lmts = sym((-190:resj:190)*(pi/180));                               % Joint Limits [min,max]
[F, V, C] = ftread('gp7_r_axis.stl.stl');                                   % Brings in the STL information as arrays
ArmRoll.F = F; ArmRoll.V = sym(round(V,4)); ArmRoll.C = C;                  % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 5 - Wrist Bend
WristBend.s = yb;                                                           % Screw Axis direction in base frame
WristBend.h =  sym('0.0');                                                  % Pitch of joint
WristBend.m = sym('m5','real');                                             % Total mass of the link
WristBend.I = sym('I5',[3,1],'real');                                       % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
WristBend.qb = ([sym('380'), sym('0.0'), sym('715.0')]');                   % Location the joint wrt. the base frame
WristBend.qc = sym('rcom5',[3,1],'real');                                   % Vector from joint to COM in jnt base frame
WristBend.qf = ([sym('460'), sym('0.0'), sym('715.0')]');                   % Location of the output frame wrt joint base frame
WristBend.Rc = sym('Rcom5',[3,3],'real');                                   % Rotation from the joint base frame to COM frame
WristBend.Rf = sym('Rf5',[3,3],'real');                                     % Rotation from the joint base frame to output frame
WristBend.lmts = sym((-135:resj:135)*(pi/180));                             % Joint Limits [min,max]
[F, V, C] = ftread('gp7_b_axis.stl.stl');                                   % Brings in the STL information as arrays
WristBend.F = F; WristBend.V = sym(round(V,4)); WristBend.C = C;            % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 6 - Tool Flange
ToolFlange.s = xb;                                                          % Screw Axis direction in base frame
ToolFlange.h =  sym('0.0');                                                 % Pitch of joint
ToolFlange.m = sym('m6','real');                                            % Total mass of the link
ToolFlange.I = sym('I6',[3,1],'real');                                      % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
ToolFlange.qb = ([sym('460'), sym('0.0'), sym('715.0')]');                  % Location the joint wrt. the base frame
ToolFlange.qc = sym('rcom6',[3,1],'real');                                  % Vector from joint to COM in jnt base frame
ToolFlange.qf = ([sym('460'), sym('0.0'), sym('715.0')]');                  % Location of the output frame wrt joint base frame
ToolFlange.Rc = sym('Rcom6',[3,3],'real');                                  % Rotation from the joint base frame to COM frame
ToolFlange.Rf = sym('Rf6',[3,3],'real');                                    % Rotation from the joint base frame to output frame
ToolFlange.lmts = sym((-360:resj:360)*(pi/180));                            % Joint Limits [min,max]
[F, V, C] = ftread('gp7_t_axis.stl.stl');                                   % Brings in the STL information as arrays
ToolFlange.F = F; ToolFlange.V = sym(round(V,4)); ToolFlange.C = C;         % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% EE frame in null configuration
R_M = sym([zb,yb,-xb]);                                                     % Desired End Effector Orientation
P_M = ([sym('460'), sym('0.0'), sym('715.0')]');                            % Desired End Effector Position
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
end

A = ((Jb)*(transpose(Jb))); Av = (Js(4:6,:))*transpose(Js(4:6,:)); Aw = (Js(1:3,:))*transpose(Js(1:3,:));
for ii = 1:3
    Av(:,ii) = simplify(Av(:,ii));    Aw(:,ii) = simplify(Aw(:,ii));
end

Af = symfun(A,q'); Avf = symfun(Av,q'); Awf = symfun(Aw,q');
Tbef = symfun(Tbe,(q')); Tf = symfun(Tbe(1:3,4),(q'));

clear pi;
clear SwivelBase LowerArm UpperArm ArmRoll WristBend ToolFlange

disp('Done Section 1')

% Symbolicaly this next operation takes a big computational toll, it is
% usefull since the total potential field benefits from the manipulability
% cost, It is included here for completeness but i dont recomend doing it
% dA = det(A);

%% Play with visuals
qtst = [0,0,0,0,0];

%% Inverse Kinematics of Desired Robot
% really meant to be defined bellow in the "Included Functions" section but
% called out here due to importance

%% Define the Potential Fields 
% Gerate Grids of joint ranges
[J1g1,J2g1,J3g1] = ndgrid(robot.Jnts(1).lmts,robot.Jnts(2).lmts,robot.Jnts(3).lmts);

% 
% do not uncomment here unless you want to burn your pc 
% [J1g2,J2g2,J3g2,J4g2,J5g2,J6g2] = ndgrid(robot.Jnts(1).lmts,robot.Jnts(2).lmts,robot.Jnts(3).lmts,robot.Jnts(4).lmts,robot.Jnts(5).lmts,robot.Jnts(6).lmts);
%

disp('Done making grids')

%%
% Potential Field of the obstacles - Cartesian Space
% Requires some foretought on how you define the obstacles, but assuming
% them to be boxes at locations, 1 means its inside the box, 0 means
% outside the box

% Function Defined in Included Functions

LIMS = [-3, 3, -3,3,-2,5];

Floor(x,y,z) =  piecewise(z < 0,1,0);

Ob1(x,y,z) = piecewise(  0 < x & x < 1 &...
                         0 < y & y < 1 &...
                         0 < z & z < 1,1,0);

Ob2(x,y,z) = piecewise(0 < x & x < 2 &...
                      -1 < y & y < 1 &...
                       0 < z & z < 2,1,0);
Obt = Floor+Ob1+Ob2;

%% View Obstacle potential fileds
figure(99)
hold off
fimplicit3(Obt,LIMS,"MeshDensity",20);
axis(LIMS)
xlabel('X');ylabel('Y');zlabel('Z');
title('Obstacle Potential Field')
hold on

% Potential Field of the Joints - Joint Space
% If we want to include a potential field for the Jacobian as expressed by
% the Handbook of Robotics.

% Pmanipulability(q1,q2,q3) = det(Jacobian)

% Performing forward/inverse kineamtics on every joint grid value and 
% saving the value off all potential fields for any given joint

%% Forward kinematics
n1 = numel(J1g1);n2 = numel(J2g1);n3 = numel(J3g1);n = min([n1,n2,n3]);
d = 1;
n = floor(n/d);

% Tbf(q1, q2, q3, q4, q5, q6) =
C = Tf(J1g1(1:n),J2g1(1:n),J3g1(1:n),zeros(1,n),zeros(1,n),zeros(1,n));
Xp = eval(C{1,1});Yp = eval(C{2,1});Zp = eval(C{3,1});
X = round(Xp./1000,5); Y = round(Yp./1000,5); Z = round(Zp./1000,5);
Xmin = min(X);Ymin = min(Y);Zmin = min(Z);
Xmax = max(X);Ymax = max(Y);Zmax = max(Z);
%%
P = Obt(X,Y,Z);P = [X;Y;Z;eval(P)];
P2 = [eval(J1g1(1:end));eval(J2g1(1:end));eval(J3g1(1:end));P(4,1:end)];
figure(2)
scatter3(X,Y,Z,2,P(4,:))
figure(3)
bubblechart3(X,Y,Z,P(4,:))
bubblesize([min(P(4,:))+1,max(P(4,:))+2]*3)

%C = eval(Tbef(J1g1(1),J2g1(1),J3g1(1),0,0,0));Cc = [C(1:3,4)./1000;1]
%P()Obt(Cc(1),Cc(2),Cc(3))
% jntgrid(q1,q2,q3) -> FrwKin -> Obt(x,y,z) = 1 or 0 > Pobsjnts(q1,q2,q3)
% Total Potential Field
% The sum of all potential fields
% P = Pfield(q1,q2,q3) = Pobsjnts + Pmanipulability

%% Gradient Descent - Waypoint Planing
% Performs gradient descent on the total potential field of the joints and
% generates points corresponding to the desired waypoints on both joint and
% cartesian coordinates
% x = q1 ; y = q2 ; z = q3; val = P

%% Trajectory Generation - Motion between Waypoints - In jnt coordinates
% A function of some polynomial trajectory scheme and a search algorithm
% that ensures compliance with motion constrains, here defined as joint
% position, velocity and acceleration, but a more appropiate definition is
% torque
% Path definition as a series of waypoints accompanied by constraints on
% their motion. Generate trajectory over segment A,B,C....
% Deine waypoints as [4x4] HTs in cartesian space, define maximum joint
% velocities and acceleration for each joint or for all joints
path_gen = struct('Segment',[],'Jnt',[],'tvct',[]); sgmnt_gen = struct('Start',[],'End',[],'Vwmax',[],'Awmax',[],'t0',0,...
    'tf',0.5);
% Initialize # of paths and segments
path = path_gen;segment1 = sgmnt_gen;

% Pick a joint configuration just to test - this ideally comes from the
% waypoints

QtstA = sym([0,0,0,-2]);
QtstB = sym([pi,-pi/2,2*pi/3,2]);

% update the segment #
Ap = forwardKinScara(QtstA(1),QtstA(2),QtstA(3),QtstA(4));
Bp = forwardKinScara(QtstB(1),QtstB(2),QtstB(3),QtstB(4));
Vwmax = sym('1');Awmax = sym('0.5');

segment1.Start = Ap;
segment1.End = Bp;
segment1.Vwmax= Vwmax;
segment1.Awmax= Awmax;

% A path is a sequence of segments

pathobj.Segment = [segment1,[],[],[]];

% Create a timescale vector
hrz = rest; 

TrjObj = jnttrjgn(PathObj,hrz,IKIN,FKIN);


%% Included Functions
function s = LSPB(W,tf,N)
% given the number of samples generates timescaling vectors s sd and sdd
    [s,sd,sdd,T] = trapveltraj(W,N,'EndTime',tf);
    s = struct('s',s,'sd',sd,'sdd',sdd,'T',T);
end


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


function iHT = invHT(HT)
% takes a homogeneous transformation T and calculates its inverse using
% properties of T and not inv(T)
    
    R = HT(1:3,1:3);
    P = HT(1:3,4);
    
    iHT = [transpose(R),-transpose(R)*P;0,0,0,1];

end


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


% function [Q,Qs] = InvKin(HT,FrwKin)
% This needs to be defined given the choosen robot architecture
%
%
%
%
%
% end


% Inertia code is glitchy and would porbably benefit from a second pair of eyes but i dont expect to need it
% function [robot] = inertia(robot,g)
% % Given the robot output of the above functions, it will generate the total
% % Kinetic Energy, total potential energy, and update the robot to give the
% % mass matrix
%     n = length(robot.jnts);
%     [w,l] = size(robot.Jb);
%     for ii = 1:n
%              Jp = sym(zeros(w,l));
%         Jp(:,1:ii) = sym(robot.Jb(:,1:ii));
%         robot.jnts(ii).mI = sym(robot.jnts(ii).m*eye(3));
%         Ib = sym(([robot.jnts(ii).I(1),0,0;
%                 0,robot.jnts(ii).I(2),0;
%                 0,0,robot.jnts(ii).I(3)]));
%         Iq =  Ib + (robot.jnts(ii).m*(...
%                ((robot.jnts(ii).rcom')*(robot.jnts(ii).rcom)*eye(3)) - ...
%                ((robot.jnts(ii).rcom)*(robot.jnts(ii).rcom'))...
%                     ));
%         robot.jnts(ii).Iq = sym(Iq);
%         Tcm = invHT(robot.jnts(ii).Tcm);
%         robot.jnts(ii).M = simplify(((Jp(4:6,:)')*(robot.jnts(ii).mI)*(Jp(4:6,:))) + ...
%                            ((Jp(1:3,:)')*(Tcm(1:3,1:3))*(robot.jnts(ii).mI)*(Tcm(1:3,1:3)')*(Jp(1:3,:))));
%         
%         robot.jnts(ii).Up = (robot.jnts(ii).m)*(g')*(robot.jnts(ii).Tcm(1:3,4));
%     end
%     robot.M  = sym(zeros(n));
%     robot.Up = sym(0);
%     for ii = 1:n
%         robot.M = robot.M + robot.jnts(ii).M;
%         robot.Up = robot.Up + robot.jnts(ii).Up;
%     end
%     robot.M = collect(robot.M);
%     robot.C = sym(zeros(size(robot.M)));
%     for kk = 1:length(robot.M)
%         for jj = 1:length(robot.M)
%             for ii = 1:length(robot.M)
% 
%                 Crtl(ii,jj,kk) = (1/2)*(...
%                     diff( robot.M(kk,jj) , robot.q(ii))  +   ...
%                          diff( robot.M(kk,ii) , robot.q(jj))   -   ...
%                                diff( robot.M(ii,jj) , robot.q(kk)));
%                                 
%                 robot.C(kk,jj) = robot.C(kk,jj) + Crtl(ii,jj,kk)*robot.qd(ii);
%                 
%             end
%         end
%     end
% 
%     robot.C = expand(robot.C);
%     robot.G(n,1) = 0;
% 
%     for ii = 1:n
%         robot.G(ii,1) = diff(robot.Up,robot.q(ii));
%     end
%     tau(1:n,1) = sym(0);
%     for ii = 1:n
%         disp('trq fun.')
%         m = 0; c = 0;
%         for jj = 1:n
%             disp('trq fun..')
%             for kk = 1:n
%                 disp('trq fun...')
%                 m = m + robot.M(ii,jj)*robot.qdd(jj);
%                 c = c + Crtl(ii,jj,kk)*robot.qdd(jj)*robot.qdd(kk);
%                 u = diff(robot.Up,robot.q(ii));
% 
%                 tau(ii,1) = m + c + u;
% 
%             end
%         end
%     end
% robot.tau = tau;
% 
% M = robot.M ; C = robot.C; G = robot.G;
% Torqe = matlabFunction(((M*robot.qdd) + ((robot.qd')*C*robot.qd) + G),'Vars', [robot.q, robot.qd, robot.qdd]);
% robot.Torque = Torqe;
% end
% 


function PathObj = jnttrjgn(PathObj,hrz,IKIN,FKIN)

% takes a Path object, time scaling vectors, and an inverse kinematics
% solver as inputs and generates a jointspace trajectory represented a
% vector of position, speed, and acceleration for LSPB.jntnum is the number
% of joints
% sgmnt_gen = struct('Start',[],'End',[],'Vwmax',[],'Awmax',[]);
    
t0 = 0; 
n = length(PathObj);
    for ii = 1:n
        disp(['searching trj.'])
        [QStr,QsStr] = IKIN(PathObj.Segment(ii).Start,[0,0,0,0]);
        [QEnd,QsEnd] = IKIN(PathObj.Segment(ii).End,QsStr');
        Vmax = PathObj.Segment(ii).Vwmax;Amax = PathObj.Segment(ii).Awmax;
        ta = Vmax/Amax; tf = t0+(2*ta);
        PathObj.Jnt(ii).jnt(:,1:hrz*tf) = Inf;
        PathObj.Jnt(ii).jntd(:,1:hrz*tf) = Inf;
        PathObj.Jnt(ii).jntdd(:,1:hrz*tf) = Inf;
        tf = 0;
        s.s = Inf;s.sd = Inf; s.sdd = Inf;s.T = [];
        while max(max(abs(s.sd))) > Vmax || max(max(abs(s.sdd))) > Amax
            disp(['searching trj.'])
            tf = tf + 0.25;
            N = ceil(tf*hrz);
            s = LSPB(eval([QsStr,QsEnd]),tf,N);
            % Show Pretty Lines
            for Fuselage = 1
                figure(1)
                subplot(2,2,1)
                plot(s.T,s.s(1,:),'r-');
                grid on; hold on; 
                plot(s.T,s.sd(1,:),'g-'); 
                plot(s.T,s.sdd(1,:),'b-');
                legend('Q','Qd','Qdd')
                title('Joint1')
                hold off
                subplot(2,2,2)
                plot(s.T,s.s(2,:),'r-');
                grid on; hold on; 
                plot(s.T,s.sd(2,:),'g-'); 
                plot(s.T,s.sdd(2,:),'b-');
                legend('Q','Qd','Qdd')
                title('Joint2')
                hold off
                subplot(2,2,3)
                plot(s.T,s.s(3,:),'r-');
                grid on; hold on; 
                plot(s.T,s.sd(3,:),'g-'); 
                plot(s.T,s.sdd(3,:),'b-');
                legend('Q','Qd','Qdd')
                title('Joint3')
                hold off
                subplot(2,2,4)
                plot(s.T,s.s(4,:),'r-');
                grid on; hold on; 
                plot(s.T,s.sd(4,:),'g-'); 
                plot(s.T,s.sdd(4,:),'b-');
                legend('Q','Qd','Qdd')
                title('Joint4')
                hold off
                pause(0.0001)
            end

        disp(['searching trj..'])
        end
        PathObj.Jnt(ii).jnt = s.s;PathObj.Jnt(ii).jntd = s.sd;PathObj.Jnt(ii).jntdd = s.sdd;
        PathObj(ii).tvct = s.T;
        disp(['searching trj...'])
    end

end

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
%
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
%
% The first line is object name, then comes multiple facet and vertex lines.
% A color specifier is next, followed by those faces of that color, until
% next color line.
%
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
               disp(sprintf('Reading vertix num: %d.',vnum));
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