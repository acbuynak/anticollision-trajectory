% Generates a file.mat file that can be used to quickly load and unload
% data without having to regenerate the complete robot architecture

%resj = 30;                                                                % Resolution of the sampling grids in joint space in degrees
%resc = 0.5;                                                               % Resolution of the sampling grids in cartesian space in distance units
%rest = 50;                                                                % Resolution of the sampling grids in time space in hrz

function inintGP7(resj,resc,rest)
    set(0,'DefaultFigureWindowStyle','docked')
    pi = sym('pi');                                                         % Numeric variables
    syms Xc Yc Zc real                                                      % Computational Variables
    R_E = sym('RE_', [3,3], 'real');                                        % Desired End Effector Orientation
    P_E = sym('PE_', [3,1], 'real');                                        % Desired End Effector Position
    EE = sym([R_E,P_E;[0,0,0,1]]);                                          % Desired End Effector Transformation
    
    % Robot URDF Import
    GP7 = importrobot('motoman_gp7_support/urdf/gp7.urdf');
    GP7.DataFormat = 'row';
    GP7.Gravity = sym([0,0,-9.81]);
    
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
    
    SwivelBase = jntg; LowerArm = jntg; UpperArm = jntg;
    ArmRoll = jntg; WristBend = jntg; ToolFlange = jntg;                    % Initialize as many joints as needed
    
    %%% Define joint parameters in Null position if a joint is prismatic,
    %%% define its pitch as Inf, else, 0 for no linear motion and a real number
    %%% for some amount of linear motion
    
    % Joint 1 - Swivel Base
    SwivelBase.s = zb;                                                      % Screw Axis direction in C axis
    SwivelBase.h = sym('0');                                                       % Pitch of joint
    SwivelBase.m = sym('m1','real');                                        % Total mass of the link
    SwivelBase.I = sym('I1',[3,1],'real');                                  % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
    SwivelBase.qb = sym([ 0, 0, sym('330.0')]');                            % Location the joint wrt. the base frame
    SwivelBase.qc = sym('rcom1',[3,1],'real');                              % Vector from joint to COM in jnt base frame
    SwivelBase.qf = sym([ sym('40.0'), 0, sym('330.0')]');                  % Location of the output frame wrt joint base frame
    SwivelBase.Rc = sym('Rcom1',[3,3],'real');                              % Rotation from the joint base frame to COM frame
    SwivelBase.Rf = sym('Rf1',[3,3],'real');                                % Rotation from the joint base frame to output frame
    SwivelBase.lmts = linspace(-170,170,resj)*(pi/180);                     % Joint Limits [min,max]
    % Brings in the STL information as arrays
    [F, V, C] = ftread('gp7_s_axis.stl.stl'); 
    SwivelBase.F = F; SwivelBase.V = sym(round(V,4)); SwivelBase.C = C;     % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use
    
    % Joint 2 - Lower Arm
    LowerArm.s = yb;                                                        % Screw Axis direction in C axis
    LowerArm.h = sym('0');                                                         % Pitch of joint
    LowerArm.m = sym('m2','real');                                          % Total mass of the link
    LowerArm.I = sym('I2',[3,1],'real');                                    % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
    LowerArm.qb = ([ sym('40.0'), 0, sym('330.0')]');                       % Location the joint wrt. the base frame
    LowerArm.qc = sym('rcom2',[3,1],'real');                                % Vector from joint to COM in jnt base frame
    LowerArm.qf = ([ sym('40.0'), 0, sym('715.0')]');                       % Location of the output frame wrt joint base frame
    LowerArm.Rc = sym('Rcom2',[3,3],'real');                                % Rotation from the joint base frame to COM frame
    LowerArm.Rf = sym('Rf2',[3,3],'real');                                  % Rotation from the joint base frame to output frame
    LowerArm.lmts = linspace(-65,145,resj)*(pi/180);                        % Joint Limits [min,max]
    % Brings in the STL information as arrays
    [F, V, C] = ftread('gp7_l_axis.stl.stl');    
    LowerArm.F = F; LowerArm.V = sym(round(V,4)); LowerArm.C = C;           % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use
    
    % Joint 3 - Upper Arm
    UpperArm.s = -yb;                                                       % Screw Axis direction in base frame
    UpperArm.h = sym('0');                                                         % Pitch of joint
    UpperArm.m = sym('m3','real');                                          % Total mass of the link
    UpperArm.I = sym('I3',[3,1],'real');                                    % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
    UpperArm.qb = ([ sym('40.0'), 0, sym('715.0')]');                       % Location the joint wrt. the base frame
    UpperArm.qc = sym('rcom3',[3,1],'real');                                % Vector from joint to COM in jnt base frame
    UpperArm.qf = ([ sym('40.0'), 0, sym('715.0')]');                       % Location of the output frame wrt joint base frame
    UpperArm.Rc = sym('Rcom3',[3,3],'real');                                % Rotation from the joint base frame to COM frame
    UpperArm.Rf = sym('Rf3',[3,3],'real');                                  % Rotation from the joint base frame to output frame
    UpperArm.lmts = linspace(-70,190,resj)*(pi/180);                        % Joint Limits [min,max]
    % Brings in the STL information as arrays
    [F, V, C] = ftread('gp7_u_axis.stl.stl');    
    UpperArm.F = F; UpperArm.V = sym(round(V,4)); UpperArm.C = C;           % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use
    
    % Joint 4 - Arm Roll
    ArmRoll.s = -xb;                                                        % Screw Axis direction in base frame
    ArmRoll.h = sym('0');                                                          % Pitch of joint
    ArmRoll.m = sym('m4','real');                                           % Total mass of the link
    ArmRoll.I = sym('I4',[3,1],'real');                                     % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
    ArmRoll.qb = ([ sym('40.0'), 0, sym('715.0')]');                        % Location the joint wrt. the base frame
    ArmRoll.qc = sym('rcom4',[3,1],'real');                                 % Vector from joint to COM in jnt base frame
    ArmRoll.qf = ([sym('380.0'), 0, sym('715.0')]');                        % Location of the output frame wrt joint base frame
    ArmRoll.Rc = sym('Rcom4',[3,3],'real');                                 % Rotation from the joint base frame to COM frame
    ArmRoll.Rf = sym('Rf4',[3,3],'real');                                   % Rotation from the joint base frame to output frame
    ArmRoll.lmts = linspace(-190,190,resj)*(pi/180);                        % Joint Limits [min,max]
    % Brings in the STL information as arrays
    [F, V, C] = ftread('gp7_r_axis.stl.stl');  
    ArmRoll.F = F; ArmRoll.V = sym(round(V,4)); ArmRoll.C = C;              % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use
    
    % Joint 5 - Wrist Bend
    WristBend.s = -yb;                                                      % Screw Axis direction in base frame
    WristBend.h = sym('0');                                                        % Pitch of joint
    WristBend.m = sym('m5','real');                                         % Total mass of the link
    WristBend.I = sym('I5',[3,1],'real');                                   % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
    WristBend.qb = ([sym('380.0'), 0, sym('715.0')]');                      % Location the joint wrt. the base frame
    WristBend.qc = sym('rcom5',[3,1],'real');                               % Vector from joint to COM in jnt base frame
    WristBend.qf = ([sym('460.0'), 0, sym('715.0')]');                      % Location of the output frame wrt joint base frame
    WristBend.Rc = sym('Rcom5',[3,3],'real');                               % Rotation from the joint base frame to COM frame
    WristBend.Rf = sym('Rf5',[3,3],'real');                                 % Rotation from the joint base frame to output frame
    WristBend.lmts = linspace(-135,135,resj)*(pi/180);                      % Joint Limits [min,max]
    % Brings in the STL information as arrays
    [F, V, C] = ftread('gp7_b_axis.stl.stl');                                   
    WristBend.F = F; WristBend.V = sym(round(V,4)); WristBend.C = C;        % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use
    
    % Joint 6 - Tool Flange
    ToolFlange.s = -xb;                                                     % Screw Axis direction in base frame
    ToolFlange.h = sym('0');                                                       % Pitch of joint
    ToolFlange.m = sym('m6','real');                                        % Total mass of the link
    ToolFlange.I = sym('I6',[3,1],'real');                                  % Inertia matrix in Center of mass [Ixx, Iyy, Izz]
    ToolFlange.qb = ([sym('460.0'), 0, sym('715.0')]');                     % Location the joint wrt. the base frame
    ToolFlange.qc = sym('rcom6',[3,1],'real');                              % Vector from joint to COM in jnt base frame
    ToolFlange.qf = ([sym('460.0'), 0, sym('715.0')]');                     % Location of the output frame wrt joint base frame
    ToolFlange.Rc = sym('Rcom6',[3,3],'real');                              % Rotation from the joint base frame to COM frame
    ToolFlange.Rf = sym('Rf6',[3,3],'real');                                % Rotation from the joint base frame to output frame
    ToolFlange.lmts = linspace(-360,360,resj)*(pi/180);                     % Joint Limits [min,max]
    % Brings in the STL information as arrays
    [F, V, C] = ftread('gp7_t_axis.stl.stl');
    ToolFlange.F = F; ToolFlange.V = sym(round(V,4)); ToolFlange.C = C;     % Preferable an array or struct of the vectors from the joint base to the vertex, currently not in use
    
    % EE frame in null configuration
    R_M = sym([zb,yb,-xb]);                                                 % Desired End Effector Orientation
    P_M = ([sym('460.0'), 0, sym('715.0')]');                               % Desired End Effector Position
    M_M =sym([R_M,P_M;[0,0,0,1]]);                                          % Composed Null Configuration EE Frame wrt base frame
    
    % Define the order of the joints
    jnt = [SwivelBase,LowerArm,UpperArm,ArmRoll,WristBend,ToolFlange,[]];   % Set the order of the joints in the robot
    
    % Perform Forward Kinematics
    [robot,q,~,~] = FrwKin(jnt,M_M);

    % Redefine some things for direct use later
    % Define the numbers that need substituting  and their numerical version,
    % idk if vpa is bettee but i like this way
    % inputs
    subsi = {'80.0','-80.0','340.0','-340.0','40.0','-40.0','385.0','-385.0','340.0','-340.0','330.0','-330.0','715.0','-715.0','-1.0','1.0','-0.5','0.5','160.0','-160.0','680.0','-680.0','770.0','-770.0','320.0','-320.0','1360.0','-1360.0','380.0','-380.0','1430.0','-1430.0','640.0','-640.0','760.0','-760.0',...
             str2sym('cos(q1)'),str2sym('sin(q1)'),str2sym('cos(q2)'),str2sym('sin(q2)'),str2sym('cos(q3)'),str2sym('sin(q3)'),str2sym('cos(q4)'),str2sym('sin(q4)'),str2sym('cos(q5)'),str2sym('sin(q5)'),str2sym('cos(q6)'),str2sym('sin(q6)')};
    % Outputs
    subso = { 80,    -80,    340,    -340,    40,    -40,    385,    -385,    340,    -340,    330,    -330,    715,    -715,    -1,    1,    -0.5,  0.5,  160,    -160,    680,    -680,    770,    -770,    320.    -320,    1360,    -1360,    380,    -380,    1430,    -1430,    640,    -640,    760,    -760,...
                      cos(q(1))   ,     sin(q(1)),         cos(q(2)),         sin(q(2)),         cos(q(3)),         sin(q(3))  ,       cos(q(4)),         sin(q(4)),         cos(q(5)),         sin(q(5))  ,       cos(q(6))  ,       sin(q(6))};
    Tbe = simplify(subs(robot.T.be,subsi,subso));Teb = simplify(subs(robot.T.eb,subsi,subso));
    Js = robot.Js;Jb = robot.Jb;
    
    for ii = 1:length(robot.Js)
        Js(:,ii) = simplify(collect(subs(Js(:,ii),subsi,subso),[sin(q)',cos(q)']));
        Jb(:,ii) = simplify(collect(subs(Jb(:,ii),subsi,subso),[sin(q)',cos(q)']));
        robot.Jnts(ii).Tb = simplify(collect(subs(robot.Jnts(ii).Tb,subsi,subso),[sin(q)',cos(q)']));
        robot.Jnts(ii).Tcm = simplify(collect(subs(robot.Jnts(ii).Tcm,subsi,subso),[sin(q)',cos(q)']));
        robot.Jnts(ii).Tf = simplify(collect(subs(robot.Jnts(ii).Tf,subsi,subso),[sin(q)',cos(q)']));
        robot.Jnts(ii).esS = simplify(subs(robot.Jnts(ii).esS,subsi,subso));
    end
    
    Ab = ((Jb)*(transpose(Jb))); Av = (Js(4:6,:))*transpose(Js(4:6,:)); Aw = (Js(1:3,:))*transpose(Js(1:3,:));
    Ab(:) = simplify(Ab(:));
    Av(:) = simplify(Av(:));    
    Aw(:) = simplify(Aw(:));

    SYMBS = syms;
    save('Initialization.mat','robot','Js','Jb','Tbe','Teb','resc',...
                              'resj','rest','Ab','Av','Aw','GP7','q',...
                              'subsi','subso','SYMBS','Xc',...
                              'Yc','Zc')
    disp('Done Section 1')
end
