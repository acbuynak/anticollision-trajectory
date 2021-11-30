%% Trajectory Generation - Motion between Waypoints - In jnt coordinates
% A function of some polynomial trajectory scheme and a search algorithm
% that ensures compliance with motion constrains, here defined as joint
% position, velocity and acceleration, but a more appropiate definition is
% torque
% Path definition as a series of waypoints accompanied by constraints on
% their motion. Generate trajectory over segment A,B,C....
% Deine waypoints as [4x4] HTs in cartesian space, define maximum joint
% velocities and acceleration for each joint or for all joints

clc; clear;
load('Tbef.mat');
rest = 50; 

path_gen = struct('Segment',[],'Jnt',[],'tvct',[]); 
sgmnt_gen = struct('Start',[],'End',[],'Vwmax',[],'Awmax',[],'t0',0,...
    'tf',[]);
% Initialize # of paths and segments
path = path_gen;segment1 = sgmnt_gen;segment2 = sgmnt_gen;

% Pick a joint configuration just to test - this ideally comes from the
% waypoints

QtstA = sym([  0 ,  0 ,  0 ,  0 ,  0 ,  0 ]);
QtstB = sym([pi/2,pi/2,pi/2,pi/2,pi/2,pi/2]);
QtstC = sym([-pi/2,pi  ,-pi/2,pi/2,pi  ,-pi/2]);

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

% Included Function
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

        PathObj.Jnt.jnt(njnt,1:hrz*tf) = Inf;
        PathObj.Jnt.jntd(njnt,1:hrz*tf) = Inf;
        PathObj.Jnt.jntdd(njnt,1:hrz*tf) = Inf;
        
        s.s = PathObj.Jnt.jnt;
        s.sd = PathObj.Jnt.jntd;
        s.sdd = PathObj.Jnt.jntdd;

        s.T = [];

        while max(max(abs(s.sd))) > Vmax || max(max(abs(s.sdd))) > Amax
            disp(['searching trj.'])
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

        if ii == 1
            PathObj.Jnt.jnt = s.s;
            PathObj.Jnt.jntd = s.sd;
            PathObj.Jnt.jntdd = s.sdd;
            PathObj.tvct = s.T;
        else
            PathObj.Jnt.jnt = [PathObj.Jnt.jnt,s.s];
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
