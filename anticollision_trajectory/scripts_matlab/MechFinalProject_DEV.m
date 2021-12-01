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

% Initialization
% Define variables and matrixes needed to solve the porblem

clear;clc;
set(0,'DefaultFigureWindowStyle','docked')
pi = sym(pi);                                                               % Numeric variables
syms x y z real                                                             % Computational Variables
R_E = sym('RE_', [3,3], 'real');                                            % Desired End Effector Orientation
P_E = sym('PE_', [3,1], 'real');                                            % Desired End Effector Position
EE = sym([R_E,P_E;[0,0,0,1]]);                                              % Desired End Effector Transformation

% Ressolution, in radians, of the joint limits for sampling
resj = 25;                                                                  % Resolution of the sampling grids in joint space in degrees
resc = 0.5;                                                                 % Resolution of the sampling grids in cartesian space in distance units
rest = 50;                                                                  % Resolution of the sampling grids in time space in hrz

% Robot URDF Import
GP7 = importrobot('gp7.urdf','DataFormat','column');
figure(1)
show(GP7);
pause(0.5)
% figure(2)
viz = interactiveRigidBodyTree(GP7);
pause(0.5)

% Forward Kinematics of Desired Robot - Define diferent characteristics
% of the robot in Product Of Exponential convention w.r.t the baseframe
% of the robot, unless specificed are w.r.t the robot's base, some
% values, like those relevant to the center of mass, or sensors nodes,
% are defined w.r.t. the joint's base frame

% Base Frame X Y Z Directions
xb = ([sym('1.0'),0,0]');
yb = ([0,sym('1.0'),0]'); 
zb = ([0,0,sym('1.0')]');                 

jntg = struct('s',[],'h',[],'m',[],'I',[],'qb',[],'qc',[],'qf',[],...       % General Initialization of a Joint
             'Rc',[],'Rf',[],'nSens',[],...
'S',[],'Vs',[],'sS',[],'esS',[],'Tp',[],'Tf',[],'Tcm',[],'Vb',[],'Gb',[]);  % These laset ones are meant to be outputs

SwivelBase = jntg; LowerArm = jntg; UpperArm = jntg;
ArmRoll = jntg; WristBend = jntg; ToolFlange = jntg;                        % Initialize as many joints as needed

%%% Define joint parameters in Null position if a joint is prismatic,
%%% define its pitch as Inf, else, 0 for no linear motion and a real number
%%% for some amount of linear motion

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
SwivelBase.lmts = linspace(-170,170,resj)*(pi/180);                         % Joint Limits [min,max]
% Brings in the STL information as arrays
[F, V, C] = ftread('robot_support\motoman_gp7_support\meshes\VertexFiles\gp7_s_axis.stl.stl'); 
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
LowerArm.lmts = linspace(-65,145,resj)*(pi/180);                            % Joint Limits [min,max]
% Brings in the STL information as arrays
[F, V, C] = ftread('robot_support\motoman_gp7_support\meshes\VertexFiles\gp7_l_axis.stl.stl');    
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
UpperArm.lmts = linspace(-70,190,resj)*(pi/180);                            % Joint Limits [min,max]
% Brings in the STL information as arrays
[F, V, C] = ftread('robot_support\motoman_gp7_support\meshes\VertexFiles\gp7_u_axis.stl.stl');    
UpperArm.F = F; UpperArm.V = sym(round(V,4)); UpperArm.C = C;               % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 4 - Arm Roll
ArmRoll.s = -xb;                                                            % Screw Axis direction in base frame
ArmRoll.h = 0;                                                              % Pitch of joint
ArmRoll.m = sym('m4','real');                                               % Total mass of the link
ArmRoll.I = sym('I4',[3,1],'real');                                         % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
ArmRoll.qb = ([ sym('40.0'), 0, sym('715.0')]');                            % Location the joint wrt. the base frame
ArmRoll.qc = sym('rcom4',[3,1],'real');                                     % Vector from joint to COM in jnt base frame
ArmRoll.qf = ([sym('380.0'), 0, sym('715.0')]');                            % Location of the output frame wrt joint base frame
ArmRoll.Rc = sym('Rcom4',[3,3],'real');                                     % Rotation from the joint base frame to COM frame
ArmRoll.Rf = sym('Rf4',[3,3],'real');                                       % Rotation from the joint base frame to output frame
ArmRoll.lmts = linspace(-190,190,resj)*(pi/180);                            % Joint Limits [min,max]
% Brings in the STL information as arrays
[F, V, C] = ftread('robot_support\motoman_gp7_support\meshes\VertexFiles\gp7_r_axis.stl.stl');  
ArmRoll.F = F; ArmRoll.V = sym(round(V,4)); ArmRoll.C = C;                  % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 5 - Wrist Bend
WristBend.s = -yb;                                                          % Screw Axis direction in base frame
WristBend.h = 0;                                                            % Pitch of joint
WristBend.m = sym('m5','real');                                             % Total mass of the link
WristBend.I = sym('I5',[3,1],'real');                                       % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
WristBend.qb = ([sym('380.0'), 0, sym('715.0')]');                          % Location the joint wrt. the base frame
WristBend.qc = sym('rcom5',[3,1],'real');                                   % Vector from joint to COM in jnt base frame
WristBend.qf = ([sym('460.0'), 0, sym('715.0')]');                          % Location of the output frame wrt joint base frame
WristBend.Rc = sym('Rcom5',[3,3],'real');                                   % Rotation from the joint base frame to COM frame
WristBend.Rf = sym('Rf5',[3,3],'real');                                     % Rotation from the joint base frame to output frame
WristBend.lmts = linspace(-135,135,resj)*(pi/180);                          % Joint Limits [min,max]
% Brings in the STL information as arrays
[F, V, C] = ftread('robot_support\motoman_gp7_support\meshes\VertexFiles\gp7_b_axis.stl.stl');                                   
WristBend.F = F; WristBend.V = sym(round(V,4)); WristBend.C = C;            % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 6 - Tool Flange
ToolFlange.s = -xb;                                                         % Screw Axis direction in base frame
ToolFlange.h = 0;                                                           % Pitch of joint
ToolFlange.m = sym('m6','real');                                            % Total mass of the link
ToolFlange.I = sym('I6',[3,1],'real');                                      % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
ToolFlange.qb = ([sym('460.0'), 0, sym('715.0')]');                         % Location the joint wrt. the base frame
ToolFlange.qc = sym('rcom6',[3,1],'real');                                  % Vector from joint to COM in jnt base frame
ToolFlange.qf = ([sym('460.0'), 0, sym('715.0')]');                         % Location of the output frame wrt joint base frame
ToolFlange.Rc = sym('Rcom6',[3,3],'real');                                  % Rotation from the joint base frame to COM frame
ToolFlange.Rf = sym('Rf6',[3,3],'real');                                    % Rotation from the joint base frame to output frame
ToolFlange.lmts = linspace(-360,360,resj)*(pi/180);                         % Joint Limits [min,max]
% Brings in the STL information as arrays
[F, V, C] = ftread('robot_support\motoman_gp7_support\meshes\VertexFiles\gp7_t_axis.stl.stl');
ToolFlange.F = F; ToolFlange.V = sym(round(V,4)); ToolFlange.C = C;         % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% EE frame in null configuration
R_M = sym([zb,yb,-xb]);                                                     % Desired End Effector Orientation
P_M = ([sym('460.0'), 0, sym('715.0')]');                                   % Desired End Effector Position
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
    robot.Jnts(ii).Tcm = simplify(collect(subs(robot.Jnts(ii).Tcm,subsi,subso),[sin(q)',cos(q)']));
    robot.Jnts(ii).Tb = symfun(robot.Jnts(ii).Tb,(q'));
    robot.Jnts(ii).Tcm = symfun(robot.Jnts(ii).Tcm,(q'));
    robot.Jnts(ii).esS = simplify(subs(robot.Jnts(ii).esS,subsi,subso));
end

    A = ((Jb)*(transpose(Jb))); Av = (Js(4:6,:))*transpose(Js(4:6,:)); Aw = (Js(1:3,:))*transpose(Js(1:3,:));

% for ii = 1:3
%     Av(:,ii) = simplify(Av(:,ii));    
%     Aw(:,ii) = simplify(Aw(:,ii));
% end

Af = symfun(A,q'); Avf = symfun(Av,q'); Awf = symfun(Aw,q');
Tbef = symfun(Tbe,(q')); Tf = symfun(Tbe(1:3,4),(q'));

% Some pretty visuals
% figure(3)
% subplot(2,3,1)
% patch('Faces', jnt(1).F, 'Vertices', eval(jnt(1).V), 'FaceVertexCData', (1:size(jnt(1).F,1))', 'FaceColor', 'flat');
% grid on
% subplot(2,3,2)
% patch('Faces', jnt(2).F, 'Vertices', eval(jnt(2).V), 'FaceVertexCData', (1:size(jnt(2).F,1))', 'FaceColor', 'flat');
% grid on
% subplot(2,3,3)
% patch('Faces', jnt(3).F, 'Vertices', eval(jnt(3).V), 'FaceVertexCData', (1:size(jnt(3).F,1))', 'FaceColor', 'flat');
% grid on
% subplot(2,3,4)
% patch('Faces', jnt(4).F, 'Vertices', eval(jnt(4).V), 'FaceVertexCData', (1:size(jnt(4).F,1))', 'FaceColor', 'flat');
% grid on
% subplot(2,3,5)
% patch('Faces', jnt(5).F, 'Vertices', eval(jnt(5).V), 'FaceVertexCData', (1:size(jnt(5).F,1))', 'FaceColor', 'flat');
% grid on
% subplot(2,3,6)
% patch('Faces', jnt(6).F, 'Vertices', eval(jnt(6).V), 'FaceVertexCData', (1:size(jnt(6).F,1))', 'FaceColor', 'flat');
% grid on

clear pi xb yb zb ii jnt SwivelBase LowerArm UpperArm ArmRoll WristBend ToolFlange

clc

disp('Done Section 1')
pause(0.5)
set(0,'DefaultFigureWindowStyle','normal')
% Search Algorithms
% Define the Potential Fields in some manner
% Obstacle Potential Fields
set(0,'DefaultFigureWindowStyle','docked')
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
Jntstrt = sym([ j1(ii)   ,  j2(ii)   ,   j3(ii)  , 0 , 0 , 0]);
Jntgoal = sym([j1(end-ii), j2(end-ii), j3(end-ii), 0 , 0 , 0]);
XYZstart = Tf(Jntstrt(1),Jntstrt(2),Jntstrt(3),Jntstrt(4),Jntstrt(5),Jntstrt(6));
XYZgoal = Tf(Jntgoal(1),Jntgoal(2),Jntgoal(3),Jntgoal(4),Jntgoal(5),Jntgoal(6));

C = Tf(j1g,j2g,j3g,zeros(size(j1g)),zeros(size(j1g)),zeros(size(j1g)));

%
clc
X = double(C{1,1}); Y = double(C{2,1}); Z = double(C{3,1});
disp('half way Pt1')
Po = double(Obt(C{1,1}./1000,C{2,1}./1000,C{3,1}./1000));                   % Is the point at an obstacle or not
Pf = double((1/2)*((1./C{3,1})./1000).^2);                                  % Force Pushing away from the floor                       
Pd = double((1/2)*(((XYZgoal(1) - C{1,1})./1000).^2 +...                           
            ((XYZgoal(2) - C{2,1})./1000).^2 + ...                          % Weight of the distance to the goal configuration, further out more potential
            ((XYZgoal(3) - C{3,1})./1000).^2));                                    

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
hold on;
scatter3(C{1,1}(:)./1000,C{2,1}(:)./1000,C{3,1}(:)./1000,2,Pt(:));colorbar;
xlabel('X');ylabel('Y');zlabel('Z');
quiver3(C{1,1}(:)./1000,C{2,1}(:)./1000,C{3,1}(:)./1000,Pt1(:),Pt2(:),Pt3(:))
hold off
figure(6)
scatter3(j1g(:),j2g(:),j3g(:),2,Pt(:));colorbar
hold on
quiver3(j1g(:),j2g(:),j3g(:),Pt1(:),Pt2(:),Pt3(:))
xlabel('Joint 1');ylabel('Joint 2');zlabel('Joint 3');
set(0,'DefaultFigureWindowStyle','normal')
clc
%%
clc
Gama.toll = 1e-3;
Gama.gama = .05;
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

Qtst1 = jseq(:,1);
Qtst2 = jseq(:,2);
Qtst3 = jseq(:,3);
Qtst4 = jseq(:,4);
Qtst5 = jseq(:,5);
Qtst6 = jseq(:,6);
Qtst7 = jseq(:,7);
Qtst8 = jseq(:,8);
Vwmax = 0.5  ;Awmax = 0.5;

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

% vizualisation
r = rateControl(120);
figure(9)
show(GP7,double(jseq(:,1)))

scatter()
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