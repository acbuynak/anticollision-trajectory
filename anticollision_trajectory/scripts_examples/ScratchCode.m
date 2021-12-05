% 2 Links planar manipulator forward kineamtics
clc
clear
set(0,'DefaultFigureWindowStyle','docked')
pi = sym('pi');                                                         % Numeric variables
syms Xc Yc Zc X1 Y1 Z1 X2 Y2 Z2 x y z real                                    % Computational Variables
R_E = sym('RE_', [3,3], 'real');                                        % Desired End Effector Orientation
P_E = sym('PE_', [3,1], 'real');                                        % Desired End Effector Position
EE = sym([R_E,P_E;[0,0,0,1]]);                                          % Desired End Effector Transformation

% Forward Kinematics of Desired Robot - Define diferent characteristics
% of the robot in Product Of Exponential convention w.r.t the baseframe
% of the robot, unless specificed are w.r.t the robot's base, some
% values, like those relevant to the center of mass, or sensors nodes,
% are defined w.r.t. the joint's base frame
% Base Frame X Y Z Directions

xb = ([sym('1.0'),0,0]');    yb = ([0,sym('1.0'),0]');    zb = ([0,0,sym('1.0')]');                 

jntg = struct('s',[],'h',[],'m',[],'I',[],'qb',[],'qc',[],'qf',[],...   % General Initialization of a Joint
             'Rc',[],'Rf',[],'nSens',[],...
'S',[],'Vs',[],'sS',[],'esS',[],'Tp',[],'Tf',[],'Tcm',[],'Vb',[],'Gb',[]);  % These laset ones are meant to be outputs

Arm1 = jntg; Arm2 = jntg; Arm3 = jntg;

%%% Define joint parameters in Null position if a joint is prismatic,
%%% define its pitch as Inf, else, 0 for no linear motion and a real number
%%% for some amount of linear motion
resj = 30;

% Joint 1 - Arm 1
Arm1.s = zb;                                                      % Screw Axis direction in C axis
Arm1.h = 0;                                                       % Pitch of joint
Arm1.m = sym('m1','real');                                        % Total mass of the link
Arm1.I = sym('I1',[3,1],'real');                                  % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
Arm1.qb = sym([ sym('0.0'), 0, 0]');                            % Location the joint wrt. the base frame
Arm1.qf = sym([ sym('2.5'), 0, 0]');                  % Location of the output frame wrt joint base frame
Arm1.Rf = sym(eye(3));                                % Rotation from the joint base frame to output frame
Arm1.qc = sym((Arm1.qf - Arm1.qb)/2);                              % Vector from joint to COM in jnt base frame
Arm1.Rc = sym(eye(3));                              % Rotation from the joint base frame to COM frame
Arm1.lmts = linspace(-360,360,resj)*(pi/180);                     % Joint Limits [min,max]
Arm1.F = 1; Arm1.V = 1; Arm1.C = 1;

% Joint 2 - Arm2
Arm2.s = zb;                                                      % Screw Axis direction in C axis
Arm2.h = 0;                                                       % Pitch of joint
Arm2.m = sym('m1','real');                                        % Total mass of the link
Arm2.I = sym('I1',[3,1],'real');                                  % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
Arm2.qb = sym([ sym('2.5'), 0, 0]');                              % Location the joint wrt. the base frame
Arm2.qf = sym([ sym('5.0'), 0, 0]');                              % Location of the output frame wrt joint base frame
Arm2.Rf = sym(eye(3));                                            % Rotation from the joint base frame to output frame
Arm2.qc = sym((Arm2.qf - Arm2.qb)/2);                             % Vector from joint to COM in jnt base frame
Arm2.Rc = sym(eye(3));                                            % Rotation from the joint base frame to COM frame
Arm2.lmts = linspace(-360,360,resj)*(pi/180);                     % Joint Limits [min,max]
Arm2.F = 1; Arm2.V = 1; Arm2.C = 1;

% M_null frame  
M = sym( [eye(3),[sym('5.0'), 0, 0]';[0,0,0,1]]);

jnt = [Arm1,Arm2];
[robot,q,~,~] = FrwKin(jnt,M);

n = numel(q);

syms q [1,n] real
syms qd [1,n] real
syms qdd [1,n] real

%
clc
clear pi
subsi = {'100.0','-100.0','50.0','-50.0','25.0','-25.0',str2sym('cos(q1)'),str2sym('sin(q1)'),str2sym('cos(q2)'),str2sym('sin(q2)')};
subso = {  100  ,  -100  ,  50  ,  -50  ,  25  ,  -25  ,cos(q1)   ,         sin(q1)  ,         cos(q2)  ,         sin(q2)};

Tbe = eval(simplify(subs(robot.T.be,subsi,subso)));   
Tb1 = eval(simplify(subs(robot.Jnts(1).Tb,subsi,subso)));
Tb2 = eval(simplify(subs(robot.Jnts(2).Tb,subsi,subso)));
Tc1 = eval(simplify(subs(robot.Jnts(1).Tcm,subsi,subso)));
Tc2 = eval(simplify(subs(robot.Jnts(2).Tcm,subsi,subso)));
j1 = simplify(subs(robot.Jnts(1).lmts,'pi',pi));
j2 = simplify(subs(robot.Jnts(2).lmts,'pi',pi));
n1 = numel(j1); 
n2 = numel(j2);
Js = simplify(subs(robot.Js,subsi,subso));
Jb = simplify(subs(robot.Jb,subsi,subso));

XY_E = symfun(Tbe(1:2,4),q);
X_E = eval(Tbe(1,4));
Y_E = eval(Tbe(2,4));
XY_Tc1 = eval(Tc1(1:2,4));
XY_Tc2 = eval(Tc2(1:2,4));
XY_Tb2 = eval(Tb2(1:2,4));


[j1g,j2g] = ndgrid(j1,j2);

nel = numel(j1g);

XY_Strt = (XY_E(0,-pi/4));
XY_goal = (XY_E(3*pi/2,pi/2));

rho = 1; mu = 10; K = 1; K2 = 2; sft = 0.5;                                 % Constants to controll the growth of the  potential field
cPg = ( (rho/2) .* cdist(XY_goal , [Xc,Yc]).^(2*K) );                       % Potential to the goal
obst1 = cdist([5,0] , [Xc,Yc]);
obst2 = cdist([3,0] , [Xc,Yc]);
obst3 = cdist([4,0] , [Xc,Yc]);
obst4 = cdist([5,1] , [Xc,Yc]);
obst5 = cdist([4,1] , [Xc,Yc]);
obst6 = cdist([3,1] , [Xc,Yc]);
obst7 = cdist([5,-1] , [Xc,Yc]);
obst8 = cdist([4,-1] , [Xc,Yc]);
obst9 = cdist([3,-1] , [Xc,Yc]);

r = 2; d = 100;
ob1P = d*exp(-(obst1^2)*r);
ob2P = d*exp(-(obst2^2)*r);
ob3P = d*exp(-(obst3^2)*r);
ob4P = d*exp(-(obst4^2)*r);
ob5P = d*exp(-(obst5^2)*r);
ob6P = d*exp(-(obst6^2)*r);
ob7P = d*exp(-(obst7^2)*r);
ob8P = d*exp(-(obst8^2)*r);
ob9P = d*exp(-(obst9^2)*r);
obTP = ob1P + ob2P + ob3P + ob4P + ob5P + ob6P + ob7P + ob8P + ob9P;

sft = 0.5;
Pc =  ( rho .* (cdist([X1,Y1] , [X2,Y2]) + sft).^(-1)) ;                           
Pcf = symfun(Pc,[X1,Y1,X2,Y2]);

cPg = subs(cPg,'pi','pi');
cPgf = symfun(cPg,[Xc,Yc]);
cobTP = symfun(obTP,[Xc,Yc]);

cPt = obTP + cPg;

figure
fsurf(cPgf,[-10,10,-10,10])
title('Distance to Goal')
xlabel('X');ylabel('Y');zlabel('Potential')
figure
fsurf(obTP,[-10,10,-10,10])
title('Distance to Obstacle')
xlabel('X');ylabel('Y');zlabel('Potential')
figure
fsurf(cPt,[-10,10,-10,10])
title('Total to Cartisean')
xlabel('X');ylabel('Y');zlabel('Potential')

EE_P_gol = eval(cPgf(X_E,Y_E));                                               % EE potential field to the goal
EE_P_obs = eval(cobTP(X_E,Y_E));                                              % EE potential field to the goal

distTc1_Tc2= cdist(XY_Tc1 , XY_Tc2);
distTc2_O= cdist(XY_Tb2 , [0,0]);

Pc_1_2 = eval(d*exp(-(distTc1_Tc2^2)*r));
Pc_O_E = eval(d*exp(-(distTc2_O^2)*r));

jPt = eval(EE_P_gol + EE_P_obs + Pc_1_2 + Pc_O_E);

figure
subplot(1,2,1)
fsurf(cPt,[-10,10,-10,10])
title('Cartesian Potential Field')
xlabel('X');ylabel('Y');zlabel('Potential')
subplot(1,2,2)
fsurf(jPt,[eval(j1(1)),eval(j1(end)),eval(j2(1)),eval(j2(end))])
title('Joint Potential Field')
xlabel('Q1');ylabel('Q2');zlabel('Potential')



