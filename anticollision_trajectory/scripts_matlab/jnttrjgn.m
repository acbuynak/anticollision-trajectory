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
        
        PathObj.Jnt.jnt = zeros(njnt,hrz*tf);
        PathObj.Jnt.jntd = (Vmax + 0.01)*ones(njnt,hrz*tf);
        PathObj.Jnt.jntdd = (Amax + 0.01)*ones(njnt,hrz*tf);
        
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
%         plot(PathObj.tvct,PathObj.Jnt.jnt(1,:),'r-',...
%              PathObj.tvct,PathObj.Jnt.jnt(2,:),'b-',...
%              PathObj.tvct,PathObj.Jnt.jnt(3,:),'g-',...
%              PathObj.tvct,PathObj.Jnt.jnt(4,:),'y-',...
%              PathObj.tvct,PathObj.Jnt.jnt(5,:),'p-',...
%              PathObj.tvct,PathObj.Jnt.jnt(6,:),'o-')
%         legend('q1','q2','q3','q4','q5','q6')
%         ylabel('Position Radians')
%         xlabel('Time seconds')
%         grid on
%         subplot(3,1,2)
%         plot( PathObj.tvct,PathObj.Jnt.jntd(1,:),'r-',...
%               PathObj.tvct,PathObj.Jnt.jntd(2,:),'b-',...
%               PathObj.tvct,PathObj.Jnt.jntd(3,:),'g-',...
%               PathObj.tvct,PathObj.Jnt.jntd(4,:),'y-',...
%               PathObj.tvct,PathObj.Jnt.jntd(5,:),'p-',...
%               PathObj.tvct,PathObj.Jnt.jntd(6,:),'o-')
%         legend('qd1','qd2','qd3','qd4','qd5','qd6')
%         ylabel('Velocity')
%         xlabel('Time seconds')
%         grid on
%         subplot(3,1,3)
%         plot( PathObj.tvct,PathObj.Jnt.jntdd(1,:),'r-',...
%               PathObj.tvct,PathObj.Jnt.jntdd(2,:),'b-',...
%               PathObj.tvct,PathObj.Jnt.jntdd(3,:),'g-',...
%               PathObj.tvct,PathObj.Jnt.jntdd(4,:),'y-',...
%               PathObj.tvct,PathObj.Jnt.jntdd(5,:),'p-',...
%               PathObj.tvct,PathObj.Jnt.jntdd(6,:),'o-')
%         legend('qdd1','qdd2','qdd3','qdd4','qdd5','qdd6')
%         ylabel('Acceleration')
%         xlabel('Time seconds')
%         grid on
% 
%     end
% 
% TT = timetable(PathObj.Jnt.jnt',PathObj.Jnt.jntd',PathObj.Jnt.jntdd','SampleRate',hrz);
% TT.Properties.VariableNames = {'Position','Velocity','Acceleration'};
TT = [];

end

