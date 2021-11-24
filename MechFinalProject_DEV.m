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
res = 0.5;                                                                  % Resolution of the sampling grids

% Forward Kinematics of Desired Robot
% Initialize
xs = sym([1,0,0]'); ys = sym([0,1,0]'); zs = sym([0,0,1]');                 % Canonical X Y Z Directions

jntg = struct('s',[],'h',[],'m',[],'I',[],'qb',[],'qc',[],'qf',[],...       % General Initialization of a Joint
             'Rc',[],'Rf',[],'nSens',[],...
'S',[],'Vs',[],'sS',[],'esS',[],'Tp',[],'Tf',[],'Tcm',[],'Vb',[],'Gb',[]);  % These laset ones are meant to be outputs

SwivelBase = jntg; LowerArm = jntg; UpperArm = jntg;
ArmRoll = jntg; WristBend = jntg; ToolFlange = jntg;                        % Initialize as many joints as needed

% Define joint parameters in Null position if a joint is prismatic, define
% its pitch as Inf, else, 0 for no linear motion and a real number for some
% amount of linear motion

% Joint 1 - Swivel Base
SwivelBase.s = zs;                                                          % Screw Axis direction in C axis
SwivelBase.h = sym('0');                                                    % Pitch of joint
SwivelBase.m = sym('m1','real');                                            % Total mass of the link
SwivelBase.I = sym('I1',[3,1],'real');                                      % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
SwivelBase.qb = sym([0,0,330]');                                            % Location the joint wrt. the base frame
SwivelBase.qc = sym('rcom1',[3,1],'real');                                  % Vector from joint to COM in jnt base frame
SwivelBase.qf = sym([40,0,330]');                                           % Location of the output frame wrt joint base frame
SwivelBase.Rc = sym('Rcom1',[3,3],'real');                                  % Rotation from the joint base frame to COM frame
SwivelBase.Rf = sym('Rf1',[3,3],'real');                                    % Rotation from the joint base frame to output frame
SwivelBase.lmts = sym([-170:res:170]*(pi/180));                             % Joint Limits [min,max]
SwivelBase.rsns = 2;                                                        % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 2 - Lower Arm
LowerArm.s = ys;                                                            % Screw Axis direction in C axis
LowerArm.h = sym('0');                                                      % Pitch of joint
LowerArm.m = sym('m2','real');                                              % Total mass of the link
LowerArm.I = sym('I2',[3,1],'real');                                        % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
LowerArm.qb = sym([40,0,715]');                                             % Location the joint wrt. the base frame
LowerArm.qc = sym('rcom2',[3,1],'real');                                    % Vector from joint to COM in jnt base frame
LowerArm.qf = sym([40,0,330]');                                             % Location of the output frame wrt joint base frame
LowerArm.Rc = sym('Rcom2',[3,3],'real');                                    % Rotation from the joint base frame to COM frame
LowerArm.Rf = sym('Rf2',[3,3],'real');                                      % Rotation from the joint base frame to output frame
LowerArm.lmts = sym([-65:res:145]*(pi/180));                                % Joint Limits [min,max]
LowerArm.rsns = 2;                                                          % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 3 - Upper Arm
UpperArm.s = ys;                                                            % Screw Axis direction in base frame
UpperArm.h = sym('0');                                                      % Pitch of joint
UpperArm.m = sym('m3','real');                                              % Total mass of the link
UpperArm.I = sym('I3',[3,1],'real');                                        % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
UpperArm.qb = sym([40,0,715]');                                             % Location the joint wrt. the base frame
UpperArm.qc = sym('rcom3',[3,1],'real');                                    % Vector from joint to COM in jnt base frame
UpperArm.qf = sym([40,0,715]');                                             % Location of the output frame wrt joint base frame
UpperArm.Rc = sym('Rcom3',[3,3],'real');                                    % Rotation from the joint base frame to COM frame
UpperArm.Rf = sym('Rf3',[3,3],'real');                                      % Rotation from the joint base frame to output frame
UpperArm.lmts = sym([-70:res:190]*(pi/180));                                % Joint Limits [min,max]
UpperArm.rsns = 2;                                                          % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 4 - Arm Roll
ArmRoll.s = xs;                                                             % Screw Axis direction in base frame
ArmRoll.h = sym('0');                                                       % Pitch of joint
ArmRoll.m = sym('m4','real');                                               % Total mass of the link
ArmRoll.I = sym('I4',[3,1],'real');                                         % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
ArmRoll.qb = sym([40,0,715]');                                              % Location the joint wrt. the base frame
ArmRoll.qc = sym('rcom4',[3,1],'real');                                     % Vector from joint to COM in jnt base frame
ArmRoll.qf = sym([380,0,715]');                                             % Location of the output frame wrt joint base frame
ArmRoll.Rc = sym('Rcom4',[3,3],'real');                                     % Rotation from the joint base frame to COM frame
ArmRoll.Rf = sym('Rf4',[3,3],'real');                                       % Rotation from the joint base frame to output frame
ArmRoll.lmts = sym([-190:res:190]*(pi/180));                                % Joint Limits [min,max]
ArmRoll.rsns = 2;                                                           % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use                        

% Joint 5 - Wrist Bend
WristBend.s = ys;                                                           % Screw Axis direction in base frame
WristBend.h = sym('0');                                                     % Pitch of joint
WristBend.m = sym('m5','real');                                             % Total mass of the link
WristBend.I = sym('I5',[3,1],'real');                                       % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
WristBend.qb = sym([380,0,715]');                                           % Location the joint wrt. the base frame
WristBend.qc = sym('rcom5',[3,1],'real');                                   % Vector from joint to COM in jnt base frame
WristBend.qf = sym([460,0,715]');                                           % Location of the output frame wrt joint base frame
WristBend.Rc = sym('Rcom5',[3,3],'real');                                   % Rotation from the joint base frame to COM frame
WristBend.Rf = sym('Rf5',[3,3],'real');                                     % Rotation from the joint base frame to output frame
WristBend.lmts = sym([-135:res:135]*(pi/180));                              % Joint Limits [min,max]
WristBend.rsns = 2;                                                         % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use

% Joint 6 - Tool Flange
ToolFlange.s = xs;                                                          % Screw Axis direction in base frame
ToolFlange.h = sym('0');                                                    % Pitch of joint
ToolFlange.m = sym('m6','real');                                            % Total mass of the link
ToolFlange.I = sym('I6',[3,1],'real');                                      % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
ToolFlange.qb = sym([460,0,715]');                                          % Location the joint wrt. the base frame
ToolFlange.qc = sym('rcom6',[3,1],'real');                                  % Vector from joint to COM in jnt base frame
ToolFlange.qf = sym([460,0,715]');                                          % Location of the output frame wrt joint base frame
ToolFlange.Rc = sym('Rcom6',[3,3],'real');                                  % Rotation from the joint base frame to COM frame
ToolFlange.Rf = sym('Rf6',[3,3],'real');                                    % Rotation from the joint base frame to output frame
ToolFlange.lmts = sym([-360:res:360]*(pi/180));                             % Joint Limits [min,max]
ToolFlange.rsns = 2;                                                        % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use                          

% EE frame in null configuration
R_M = sym([zs,ys,-xs]);                                                     % Desired End Effector Orientation
P_M = sym([460,0,715]');                                                    % Desired End Effector Position
M_M =sym([R_M,P_M;[0,0,0,1]]);                                              % Composed Null Configuration EE Frame wrt base frame

% Define the order of the joints
jnt = [SwivelBase,LowerArm,UpperArm,ArmRoll,WristBend,ToolFlange];          % Set the order of the joints in the robot

clear SwivelBase LowerArm UpperArm ArmRoll WristBend ToolFlange

% Perform Forward Kinematics
[robot,q,qd,qdd] = FrwKin(jnt,M_M)

% Redefine some things for direct use later
Tbe = simplify(robot.T.be);
Teb = simplify(robot.T.eb);
Js = robot.Js;
Jb = robot.Jb;
%%
A = simplify((Js)*(transpose(Js)));


%% Inverse Kinematics of Desired Robot
% really meant to be defined bellow in the "Included Functions" section but
% called out here due to importance

%% Potential Fields 
% Potential Field of the obstacles - Cartesian Space
% Requires some foretought on how you define the obstacles, but assuming
% them to be boxes at locations, 1 means its inside the box, 0 means
% outside the box

% Function Defined in Included Functions

LIMS = [-3, 3, -3,3,-2,5];

Floor(x,y,z) =  piecewise(z < 0,2,0);

Ob1(x,y,z) = piecewise(-1 < x & x < 1 &...
                       -1 < y & y < 1 &...
                        0 < z & z < 1,2,0);

Ob2(x,y,z) = piecewise(0 < x & x < 2 &...
                       0 < y & y < 2 &...
                       0 < z & z < 4,2,0);
Obt = Floor+Ob1+Ob2;

% View Obstacle potential fileds
figure(1)
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

% Performing forward kineamtics on every joint  grid value and saving the
% sym off all potential fields for any given joint

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
A = forwardKinScara(QtstA(1),QtstA(2),QtstA(3),QtstA(4));
B = forwardKinScara(QtstB(1),QtstB(2),QtstB(3),QtstB(4));
Vwmax = sym('1');Awmax = sym('0.5');

segment1.Start = A;
segment1.End = B;
segment1.Vwmax= Vwmax;
segment1.Awmax= Awmax;

% A path is a sequence of segments

pathobj.Segment = [segment1];

% Create a timescale vector
hrz = 50; 

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
        Tp = simplify(Tp*temp(ii).esS);                                     % Collects the value of the exponential for future use across all joints
        temp(ii).Tp = Tp;                                                   % Stores the previous value for this joint
        temp(ii).Tb = (Tp * [eye(3),[jnt(ii).qb(1);
                                     jnt(ii).qb(2);
                                     jnt(ii).qb(3)];...
                             [0,0,0,1]]);                                   % Transformation from the base frame to the base of the joint     
        temp(ii).Tb
        temp(ii).Tcm = (Tp * [jnt(ii).Rc,[jnt(ii).qc(1);
                                          jnt(ii).qc(2);
                                          jnt(ii).qc(3)];...
                                    [0,0,0,1]]);                            % Transformation from the base to the Center of Mass
        temp(ii).Tf = (Tp * [jnt(ii).Rf,[jnt(ii).qf(1);
                                         jnt(ii).qf(2);
                                         jnt(ii).qf(3)];...
                                  [0,0,0,1]]);                              % Transformation from the base frame to the output of the joint
        
        % Another loop (for/end) needs to be created here to calculate a
        % series of homogeneous transformations from the base to the
        % n-number of sensor points we choose to use
%         [w,l] = size(jnt(ii).rsns);                                         % Get the amount of points in the sensor array
%         
%         for jj = 1:w
%             jnt(ii).rsns(:,jj)
%         end

        if ii == 1
            Js(:,ii) = temp(ii).S;
        else
            Js(:,ii) = Adjoint(temp(ii-1).Tp)*temp(ii).S;
        end

    end
    temp
    pause
    robot.Jnts = temp;
    robot.T.be = simplify(temp(end).Tp*M);                                  % Transformation from the base frame to the EE frame
    robot.T.eb = invHT(robot.T.be);                                         % Transformation from the EE frame to the base frame
    robot.Js = Js;                                                          % Store the Space Jacobian
    robot.Jb = simplify(Adjoint(robot.T.eb)*Js);                            % Calculate and store the Body Jacobian
    % Calcualte EE twist in space and body frame
    robot.Vs = robot.Js*qd; robot.Vb = robot.Jb*qd;
end


function [Q,Qs] = InvKin(HT,FrwKin)
% This needs to be defined given the choosen robot architecture





end


% Inertia code is glitchy and would porbably benefit from a second pair of eyes but i dont expect to need it
function [robot] = inertia(robot,g)
% Given the robot output of the above functions, it will generate the total
% Kinetic Energy, total potential energy, and update the robot to give the
% mass matrix
    n = length(robot.jnts);
    [w,l] = size(robot.Jb);
    for ii = 1:n
             Jp = sym(zeros(w,l));
        Jp(:,1:ii) = sym(robot.Jb(:,1:ii));
        robot.jnts(ii).mI = sym(robot.jnts(ii).m*eye(3));
        Ib = sym(([robot.jnts(ii).I(1),0,0;
                0,robot.jnts(ii).I(2),0;
                0,0,robot.jnts(ii).I(3)]));
        Iq =  Ib + (robot.jnts(ii).m*(...
               ((robot.jnts(ii).rcom')*(robot.jnts(ii).rcom)*eye(3)) - ...
               ((robot.jnts(ii).rcom)*(robot.jnts(ii).rcom'))...
                    ));
        robot.jnts(ii).Iq = sym(Iq);
        Tcm = invHT(robot.jnts(ii).Tcm);
        robot.jnts(ii).M = simplify(((Jp(4:6,:)')*(robot.jnts(ii).mI)*(Jp(4:6,:))) + ...
                           ((Jp(1:3,:)')*(Tcm(1:3,1:3))*(robot.jnts(ii).mI)*(Tcm(1:3,1:3)')*(Jp(1:3,:))));
        
        robot.jnts(ii).Up = (robot.jnts(ii).m)*(g')*(robot.jnts(ii).Tcm(1:3,4));
    end
    robot.M  = sym(zeros(n));
    robot.Up = sym(0);
    for ii = 1:n
        robot.M = robot.M + robot.jnts(ii).M;
        robot.Up = robot.Up + robot.jnts(ii).Up;
    end
    robot.M = collect(robot.M);
    robot.C = sym(zeros(size(robot.M)));
    for kk = 1:length(robot.M)
        for jj = 1:length(robot.M)
            for ii = 1:length(robot.M)

                Crtl(ii,jj,kk) = (1/2)*(...
                    diff( robot.M(kk,jj) , robot.q(ii))  +   ...
                         diff( robot.M(kk,ii) , robot.q(jj))   -   ...
                               diff( robot.M(ii,jj) , robot.q(kk)));
                                
                robot.C(kk,jj) = robot.C(kk,jj) + Crtl(ii,jj,kk)*robot.qd(ii);
                
            end
        end
    end

    robot.C = expand(robot.C);
    robot.G(n,1) = 0;

    for ii = 1:n
        robot.G(ii,1) = diff(robot.Up,robot.q(ii));
    end
    tau(1:n,1) = sym(0);
    for ii = 1:n
        disp(['trq fun.'])
        m = 0; c = 0;
        for jj = 1:n
            disp(['trq fun..'])
            for kk = 1:n
                disp(['trq fun...'])
                m = m + robot.M(ii,jj)*robot.qdd(jj);
                c = c + Crtl(ii,jj,kk)*robot.qdd(jj)*robot.qdd(kk);
                u = diff(robot.Up,robot.q(ii));

                tau(ii,1) = m + c + u;

            end
        end
    end
robot.tau = tau;

M = robot.M ; C = robot.C; G = robot.G;
Torqe = matlabFunction(((M*robot.qdd) + ((robot.qd')*C*robot.qd) + G),'Vars', [robot.q, robot.qd, robot.qdd]);

end



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