% Forward Kineamtics
function [robot,q,qd,qdd] = FrwKin(jnt,M)
% Takes the key components of a robot, its joints and null frame, and
% creates said robot while also expanding the joint information to include
% other data
njnt = length(jnt);                                                         % Get the number of joints in the robot
Tp = eye(4);                                                                % Initialize Transformation Buffer

% Initilize Variable Vectors
q = sym('q', [1,njnt],'real');                                              % Position Variables
qd = sym('qd', [1,njnt],'real');                                            % Velocity Variables
qdd = sym('qdd', [1,njnt],'real');                                          % Acceleration Variables

% Initialize some data containers depending on 
robot = struct("Jnts",[],"Js",sym(zeros(6,njnt)),"Jb",sym(zeros(6,njnt)),"T",[]);   
temp = jnt;
Js = sym(zeros(6,njnt));                                                    % Initialize the Jacobian

% Takes a joint series as a structure containing (I Inputs and gives O
% outputs):
% jnt = struct('s',[I],'h',[I],'m',[I],'I',[I],'qb',[I],'qc',[I],'qf',[I],... 
%              'Rc',[I],'Rf',[I],'nSens',[I],...
% 'S',[O],'Vs',[O],'sS',[O],'esS',[O],'Tp',[O],'Tf',[O],'Tcm',[O],'Vb',[O],'Gb',[O]);

    for ii = 1:njnt
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
        temp(ii).Sv = temp(ii).S*q(ii);                                   % Creates the screw axis in space frame with the variable
        temp(ii).Vs = temp(ii).S*qd(ii);                                  % Creates the screw axis in space frame with the variable
        temp(ii).sS = screw2skew(temp(ii).Sv);                              % Represents the screw axis in skew symetric form
        temp(ii).esS = expm(temp(ii).sS);                                   % Takes the matrix exponential of the skew symetric screw axis 
        Tp = collect(Tp*temp(ii).esS,[sin(q)',cos(q)']);                    % Collects the value of the exponential for future use across all joints
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
        %robot.Jnts(ii).Tvec = (robot.Jnts(ii).Tcm)*(robot.Jnts(ii).V);
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
    robot.Vs = robot.Js*qd'; robot.Vb = robot.Jb*qd';
end


