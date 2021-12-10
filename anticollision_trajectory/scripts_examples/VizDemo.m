%% Visualization Tool For GP7  Potential Fields
%   This demonstration is to showcase the visualization of the potential
%   fields present on the  main file. This evolved from an initial solution
%   to the complete prject that used discrete potential fields instead of
%   continous functions.
%% Generate the Robot File With a Desired Joint Limit Resolution
% This should be comented out once a relevant file is generated since its
% not needed.
clear;clc;
resj = 15;                                                             % Resolution of the sampling grids in joint space in degrees
resc = 0.5;                                                            % Resolution of the sampling grids in cartesian space in distance units
rest = 50;                                                             % Resolution of the sampling grids in time space in hrz
inintGP7(resj,resc,rest)
clear; clc;
% load('Initialization.mat');
%% Main File of Demonstration
clc;clear;
% load('Initialization.mat');VizDemoLowRes.mat
load('VizDemoLowRes.mat');
% Define all symbolic joint parameters withing this workspace and other
% suefull recals for later
syms q [1,numel(q)] real; syms qd [1,numel(q)] real; syms qdd [1,numel(q)] real;
Tbe = eval(robot.T.be);
Js = eval(Js); Jb = eval(Jb);

% Just make a quick 'pi' substitution. Adding and 'double' function to 
% j1-j6 also can help with later memory concerns since it stores the
% results here as double precision points
j1 = subs(robot.Jnts(1).lmts,'pi',pi);j2 = subs(robot.Jnts(2).lmts,'pi',pi);
j3 = subs(robot.Jnts(3).lmts,'pi',pi);j4 = subs(robot.Jnts(4).lmts,'pi',pi);
j5 = subs(robot.Jnts(5).lmts,'pi',pi);j6 = subs(robot.Jnts(6).lmts,'pi',pi);

% Make some usefull functions for reusing later
XYZ_E = Tbe(1:3,4)./1000; XYZf_E  = symfun(eval(XYZ_E),q');                 % Position of the End-Effector
Rot_E = Tbe(1:3,1:3); Rotf_E = symfun(eval(Rot_E),q');                      % Rotation of the End-Effector

% Define the Grid to be Used in the Configuration Space - In this
% demosntration we are using only joints 1 through 3. This is also due to
% RAM constraints within MATLAB and computers with lower computational
% resources. Just change the code bellow to add/remove relevant ji and jig
% values
% [j1g,j2g,j3g,j4g,j5g] = ndgrid(j1,j2,j3,j4,j5);
[j1g,j2g,j3g] = ndgrid(j1,j2,j3); 
% A Bunch ofg Zero Arrays to Use in Function Evaluation
j4g = sym('0')*(ones(size(j1g))); 
j5g = sym('0')*(ones(size(j1g))); 
j6g = sym('0')*(ones(size(j1g)));

% Calculate the Forward Kinematics for Every Component of the Hyperprism in
% Configuration Scpace
XYZ = XYZf_E(j1g, j2g, j3g, j4g, j5g, j6g);
XYZc(1,:) = XYZ{1}(:); XYZc(2,:) = XYZ{2}(:); XYZc(3,:) = XYZ{3}(:);        % Reshape the results from cells into simple vectors

% Display the reachable workspace of the GP7 manipulator
figure
show(GP7);
title('GP7 Reachable Workspace')
AXISLimits = eval([min(XYZc(1,:)) max(XYZc(1,:)) min(XYZc(2,:)) max(XYZc(2,:)) min(XYZc(3,:)) max(XYZc(3,:))]).*1.05;
axis(AXISLimits)
hold on
scatter3(XYZ{1}(:),XYZ{2}(:),XYZ{3}(:),'b.')
view(135,10)
pause(1)

% Potential Fields
% Generate the formulas for the generalized potential fields in cartesian
% space. In this example we use a goal point and a single obstacle point.
% %% %% %% %% Later, hopefully I will include a demo using multiple obstacle points
% Define symbolic variables for generalizing the following equations in
% n-diemsnions, we use 3 dimensions. Some of these will not be used for
% when they area actualiy defined but for analysis and coding they are
% great placeholders

syms Xg Yg Zg Xo Yo Zo Xl1 Yl1 Zl1 Xl2 Yl2 Zl2 X Y Z Zf real
XYZg = [Xg, Yg, Zg]; 
XYZ_var = [Xc,Yc,Zc]; 
XYZ_o = [Xo, Yo, Zo];
XYZ_l1 = [Xl1, Yl1, Zl1]; 
XYZ_l2 = [Xl2, Yl2, Zl2];

% Define the Starting position and Ending position of the robot
% manipulator, in this demonstration we use the forward kinematics to
% ensure both points are within the manipulator workspace
ofst = 5;                                                                   % Ofster from the end of the jnt limits given the rest number of steps

Jntstrt = sym([ j1(ofst)   ,  j2(ofst)   ,   j3(ofst)  , 0 , 0 , 0]);       % Start Configuration
Jntgoal = sym([j1(end-ofst), j2(end-ofst), j3(end-ofst), 0 , 0 , 0]);       % End Configuration

% Also Define the height of the wrt the base frame of the manipulator
Zf = [Xc,Yc, sym('0')];

% Also define an obstacle wrt the base frame of the manipulator
XYZ_o = sym([0.5,0,0.35]);

% Convert these markers into cartesian space values
XYZ_start = expand(XYZf_E(Jntstrt(1),Jntstrt(2),Jntstrt(3),...
                          Jntstrt(4),Jntstrt(5),Jntstrt(6)));

XYZg = expand(XYZf_E(Jntgoal(1),Jntgoal(2),Jntgoal(3),...
                         Jntgoal(4),Jntgoal(5),Jntgoal(6)));

% Atraction Potential Field
rho = 10; K1 = 1;                                                           % Growth Rate Parameters
dg = (cdist(XYZg,XYZ_var));
Pg = (rho*(dg^(2*K1)));
Pgf = symfun(Pg,XYZ_var);
gradient_Pgf = gradient(Pgf);

% Obstacle Repuslion Potential Field
mu = 1; kapa = 1; K2 = 1;                                                   % Growth Rate Parameters
do = (cdist(XYZ_o,XYZ_var));
Po = (mu*exp(-kapa*(do^(2*K2))));
Pof = symfun(Po,XYZ_var);
gradient_Pof = gradient(Pof);

% Floor Repulsion Potential Fielz
mu2 = 10; kapa2 = 10; K3 = 1;                                               % Growth Rate Parameters
df = (cdist(Zf,XYZ_var));
Pf = (mu2*exp(-kapa2*(df^(2*K2))));
Pff = symfun(Pf,XYZ_var);
gradient_Pff = gradient(Pff);

% Total Cartesian Potential Field
Ptotal_c = simplify(expand(Pg + Po + Pf));
Ptotalf_c = symfun(Ptotal_c,XYZ_var);
gradient_Ptotalf_c = gradient(Ptotalf_c);

% Repulsion Potential Field Between Manipulator Links Defined to the middle
% of the link bodies, generally this goes to the center of mass, in our
% case those are close enough to the same thing. This can also be extended
% to be calculated between points in the surface of the manipulator's
% links. this field also only makes sense to be ploted in the configuration
% space, even when the metric is defined in cartesian space

mu3 = 1; kapa3 = 1; K4 = 1;                                                 % Growth Rate Parameters
dml = (cdist(XYZ_l1,XYZ_l2));
Pml = (mu*exp(-kapa*(dml^(2*K2))));
Pmlf = symfun(Pml, [XYZ_l1,XYZ_l2]);

% Potential Fields Evaluated for the region os space representing the
% % reachable workspace of the manipulator

% Evaluated Goal Potential in Cartesian Space
XYZ_goalPotential = sym('0')*(ones(size(j1g)));
XYZ_goalPotential(:) = Pgf(XYZ{:});
XYZ_goalPotential = double(XYZ_goalPotential);
XYZ_goalPotential_gradient = sym('0')*(ones(size(j1g)));
XYZ_goalPotential_gradient = gradient_Pgf(XYZ{:});

figure
show(GP7);
title('Goal Potential At Reachable Workspace')
axis(AXISLimits)
hold on
scatter3(XYZ{1}(:),XYZ{2}(:),XYZ{3}(:),2,XYZ_goalPotential(:));colorbar
scatter3(XYZg(1),XYZg(2),XYZg(3),'r*');
quiver3(XYZ{1}(:),XYZ{2}(:),XYZ{3}(:),XYZ_goalPotential_gradient{1}(:),XYZ_goalPotential_gradient{2}(:),XYZ_goalPotential_gradient{3}(:))
view(135,10)
pause(1)

% Evaluated Floor Potential in Cartesian Space
XYZ_FloorPotential = sym('0')*(ones(size(j1g)));
XYZ_FloorPotential = Pff(XYZ{:});
XYZ_FloorPotential = double(XYZ_FloorPotential);
XYZ_FloorPotential_gradient = sym('0')*(ones(size(j1g)));
XYZ_FloorPotential_gradient = gradient_Pff(XYZ{:});

figure
show(GP7);
title('Floor Potential At Reachable Workspace')
axis(AXISLimits)
hold on
scatter3(XYZ{1}(:),XYZ{2}(:),XYZ{3}(:),2,XYZ_FloorPotential(:));colorbar
quiver3(XYZ{1}(:),XYZ{2}(:),XYZ{3}(:),XYZ_FloorPotential_gradient{1}(:),XYZ_FloorPotential_gradient{2}(:),XYZ_FloorPotential_gradient{3}(:))
view(135,10)
pause(1)

% Evaluated Obstacle Potential in Cartesian Space
XYZ_ObstaclePotential = sym('0')*(ones(size(j1g)));
XYZ_ObstaclePotential = Pof(XYZ{:});
XYZ_ObstaclePotential = double(XYZ_ObstaclePotential);
XYZ_ObstaclePotential_gradient = sym('0')*(ones(size(j1g)));
XYZ_ObstaclePotential_gradient = gradient_Pof(XYZ{:});

figure
show(GP7);
title('Obstacle Potential At Reachable Workspace')
axis(AXISLimits)
hold on
scatter3(XYZ{1}(:),XYZ{2}(:),XYZ{3}(:),2,XYZ_ObstaclePotential(:));colorbar
quiver3(XYZ{1}(:),XYZ{2}(:),XYZ{3}(:),XYZ_ObstaclePotential_gradient{1}(:),XYZ_ObstaclePotential_gradient{2}(:),XYZ_ObstaclePotential_gradient{3}(:))
view(135,10)
pause(1)

% Evaluated Total Potential in Cartesian Space
XYZ_TotalPotential = sym('0')*(ones(size(j1g)));
XYZ_TotalPotential = Ptotalf_c(XYZ{:});
XYZ_TotalPotential = double(XYZ_TotalPotential);
XYZ_TotalPotential_gradient = sym('0')*(ones(size(j1g)));
XYZ_TotalPotential_gradient = gradient_Ptotalf_c(XYZ{:});

figure
show(GP7);
title('Total Potential At Reachable Workspace')
axis(AXISLimits)
hold on
scatter3(XYZ{1}(:),XYZ{2}(:),XYZ{3}(:),2,XYZ_TotalPotential(:));colorbar
scatter3(XYZg(1),XYZg(2),XYZg(3),'r*');
quiver3(XYZ{1}(:),XYZ{2}(:),XYZ{3}(:),XYZ_TotalPotential_gradient{1}(:),XYZ_TotalPotential_gradient{2}(:),XYZ_TotalPotential_gradient{3}(:))
view(135,10)
pause(1)

% Calculating the Total Potential Field Gradient for the End Effector on the
% configuration space, only for the EE, other points in the robot must be
% calculated separatly and added up to generate the Total Configuration
% Soace

% Ptotalf_c(Xc, Yc, Zc) = ... substitue the Xc Yc and Zc with the position
% component of the homogeneous trasnformation of the EE XYZ_E = Tbe(1:3,4)

Ptotal_q = Ptotalf_c(XYZ_E(1),XYZ_E(2),XYZ_E(3));
Ptotalf_q = symfun(Ptotal_q,q); % Data wrangling
Ptotalf_q = symfun(Ptotalf_q(q1,q2,q3,0,0,0),[q1,q2,q3]); % Data wrangling
gradient_Ptotalf_q = gradient(Ptotalf_q);                            

QEE_TotalPotential = Ptotalf_q(j1g,j2g,j3g);
QEE_TotalPotential_gradient = gradient_Ptotalf_q(j1g,j2g,j3g);

figure
scatter3(j1g(:),j2g(:),j3g(:),2,QEE_TotalPotential(:));colorbar
title('Total Potential Configuration Space')
axis(double([j1(1), j1(end), j2(1), j2(end), j3(1), j3(end)]).*1.1)
xlabel('q1');ylabel('q2');zlabel('q3');
hold on
quiver3(j1g(:),j2g(:),j3g(:),QEE_TotalPotential_gradient{1}(:),QEE_TotalPotential_gradient{2}(:),QEE_TotalPotential_gradient{3}(:))
view(135,10)
pause(1)
    