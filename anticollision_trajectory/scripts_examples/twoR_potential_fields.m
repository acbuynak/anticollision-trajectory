%% Potential fields method for obstacle avoidance

clear
close all
% DH parameter values

l_1 = 2;        % m
l_2 = 1;        % m
l_1c = l_1/2;   % m
l_2c = l_2/2;   % m
b = 0.125;      % m
c = 0.125;      % m
rho = 2.70*.001*(100^3);  % 2.70 g/cm^3 converted to kg/m^3, density of Al

% Frame 1
Frame(1).M = [eye(3) [l_1; 0; 0]; zeros(1,3) 1]; 
Frame(1).Mc = [eye(3) [l_1c; 0; 0]; zeros(1,3) 1]; 
Frame(1).omega = [0 0 1];
Frame(1).vel = [0 0 0];

% Frame 2
Frame(2).M = [eye(3) [l_1+l_2; 0; 0]; zeros(1,3) 1]; 
Frame(2).Mc = [eye(3) [l_1+l_2c; 0; 0]; zeros(1,3) 1]; 
Frame(2).omega = [0 0 1];
Frame(2).vel = [0 -l_1 0];

% set symbolic variables
t = sym('t', [1 size(Frame,2)]);
tdot = sym('tdot', [1 size(Frame,2)]);
tddot = sym('tddot', [1 size(Frame,2)]);
          
masses = [l_1*b*c*rho; l_2*b*c*rho];   % masses of each link, kg

Itensor = [(1/12)*masses(1)*[b^2+c^2, l_1^2+c^2, l_1^2+b^2]; (1/12)*masses(2)*[b^2+c^2, l_2^2+c^2, l_2^2+b^2]];
        
       
uv_len = max(max(Frame(size(Frame,2)).M))/7;   % unit vector length to show up on plot, make 1/7 the longest link

%% Obstacle visualization and potential field calculation

p_start = [2; 1];    % m
p_end = [-1; 1];   % m
obstacle_center = [0,1];
obstacle_radius = 0.25;

% plot obstacle and goal
obstacle_bound = obstacle_radius*[cos([0:2*pi/100:2*pi]); sin([0:2*pi/100:2*pi])]+obstacle_center';
figure(100)
plot(obstacle_bound(1,:), obstacle_bound(2,:), 'k-', p_start(1), p_start(2), 'bo', p_end(1), p_end(2), 'ro', 'LineWidth', 1.5)
xlabel ('X_0 [m]')
ylabel ('Y_0 [m]')
xlim ([-2 2])
ylim ([-2 2])

% Tabulate collisions in a gridded joint space
theta_space = 2*pi/50;
theta_1_test = -2*pi:theta_space:2*pi;
theta_2_test = -2*pi:theta_space:2*pi;
link_1_test = 0:l_1/10:l_1;
link_2_test = 0:l_1/10:l_2;
collision = zeros(length(theta_1_test), length(theta_2_test));

% define goal theta values
[thetaset1, thetaset2] = twoR_inv(p_end, l_1, l_2);
% find the negative set of these
thetaset3 = -2*pi+thetaset1;
thetaset4 = 2*pi+thetaset2;

% sanity check to make sure all four of these yield the correct forward
% kinematics.  Not used for anything else.  Comment out when not using
% T1 = exp_coor_hom_trans(Frame(1).omega,Frame(1).vel, thetaset1(1))*exp_coor_hom_trans(Frame(2).omega,Frame(2).vel, thetaset1(2))*Frame(2).M;
% T2 = exp_coor_hom_trans(Frame(1).omega,Frame(1).vel, thetaset2(1))*exp_coor_hom_trans(Frame(2).omega,Frame(2).vel, thetaset2(2))*Frame(2).M;
% T3 = exp_coor_hom_trans(Frame(1).omega,Frame(1).vel, thetaset3(1))*exp_coor_hom_trans(Frame(2).omega,Frame(2).vel, thetaset3(2))*Frame(2).M;
% T4 = exp_coor_hom_trans(Frame(1).omega,Frame(1).vel, thetaset4(1))*exp_coor_hom_trans(Frame(2).omega,Frame(2).vel, thetaset4(2))*Frame(2).M;

% check along the length of the manipulator
for i = 1:length(theta_1_test)
    for j = 1:length(theta_2_test)
        % link 1 collisions
        for k = 1:length(link_1_test)
        
        T_temp = exp_coor_hom_trans(Frame(1).omega,Frame(1).vel, theta_1_test(i))*[eye(3) [link_1_test(k); 0; 0]; 0 0 0 1];
        if (norm([obstacle_center' - T_temp(1:2,4)]) <= obstacle_radius)
            collision(i,j) = 1;
        end
        end
        
        % link 2 collisions
        for k = 1:length(link_2_test)
        
        T_temp = exp_coor_hom_trans(Frame(1).omega,Frame(1).vel, theta_1_test(i))*exp_coor_hom_trans(Frame(2).omega,Frame(2).vel, theta_2_test(j))*[eye(3) [l_1+link_2_test(k); 0; 0]; 0 0 0 1];
        if (norm([obstacle_center' - T_temp(1:2,4)]) <= obstacle_radius)
            collision(i,j) = 1;
        end
        end
        % goal field is the minimum squared distance to the nearest inverse
        % kinematics solution
        goal_field(i,j) = (1/400)*min([norm([theta_1_test(i); theta_2_test(j)]-thetaset1)^2, norm([theta_1_test(i); theta_2_test(j)]-thetaset2)^2, norm([theta_1_test(i); theta_2_test(j)]-thetaset3)^2, norm([theta_1_test(i); theta_2_test(j)]-thetaset4)^2]);
    end
end

% plot obstacle potential
figure(200)
surfc(theta_1_test, theta_2_test, collision)
xlabel ('\theta_2 [rad]')
ylabel ('\theta_1 [rad]')

% plot goal potential field
figure(300)
surfc(theta_1_test, theta_2_test, goal_field)
xlabel ('\theta_2 [rad]')
ylabel ('\theta_1 [rad]')

% plot total potential field
total_pot = collision+goal_field;
figure(400)
surfc(theta_1_test, theta_2_test, total_pot)
xlabel ('\theta_2 [rad]')
ylabel ('\theta_1 [rad]') 

%% Trajectory generation

 
%% Shortest path algorithm.  Built from https://www.mathworks.com/matlabcentral/answers/722518-shortest-path-in-a-2d-matrix
% find starts and stops in the gridded space
[start dummy]= twoR_inv(p_start, l_1, l_2);
[stop(:,1), stop(:,2)] = twoR_inv(p_end, l_1, l_2);
% find the negative set of these.  
stop(:,3) = dsearchn(theta_1_test',-2*pi+stop(:,1));
stop(:,4) = dsearchn(theta_1_test',2*pi+stop(:,2));
stop(:,1) = dsearchn(theta_1_test', stop(:,1));
stop(:,2) = dsearchn(theta_1_test', stop(:,2));
start = dsearchn(theta_1_test', start');

[m, n] = size(total_pot);
[i, j] = ndgrid(1:m,1:n);

s2i = @(i,j) sub2ind(size(total_pot),i,j);
s = s2i(i,j);
b1 = j>1;
s1 = s(b1);
d1 = s2i(i(b1),j(b1)-1);
b2 = j<n;
s2 = s(b2);
d2 = s2i(i(b2),j(b2)+1);
b3 = i>1;
s3 = s(b3);
d3 = s2i(i(b3)-1,j(b3));
b4 = i<m;
s4 = s(b4);
d4 = s2i(i(b4)+1,j(b4));
s = [s1 s2 s3 s4];
d = [d1 d2 d3 d4];
w = total_pot(d);
G = digraph(s, d, w);

i_ = ones(100,4);
j_ = ones(100,4);

% Find the shorttest path of the 4 possible end points
for ii = 1:4
k = G.shortestpath(s2i(start(1),start(2)), s2i(stop(1,ii),stop(2,ii)));
k = k(:);
[i,j] = ind2sub(size(total_pot),k);
cost = total_pot(k);
% Display result
stpath = table(i,j,cost) 
total_cost(ii) = sum(cost)
len_(ii) = length(i);
i_(1:length(i),ii) = i;
j_(1:length(j),ii) = j;
cost_(1:length(cost),ii) = cost;

 figure(400)
 hold on
 plot3(theta_2_test(j), theta_1_test(i), cost', 'r-')
 plot3(theta_2_test(start(2)),theta_1_test(start(1)), total_pot(start(1),start(2))', 'go','LineWidth', 1.5)
 plot3(theta_2_test(stop(2)), theta_1_test(stop(1)), total_pot(stop(1),stop(2))', 'ro','LineWidth', 1.5)
 

end

[cost_min, i_min] = min(total_cost);

 theta_1.pos = theta_1_test(i_(:,i_min));
 theta_2.pos = theta_2_test(j_(:,i_min));
 
% plot theta pos
figure(1)
% subplot(3,1,1)
plot(0:length(theta_1.pos)-1,theta_1.pos, 'b-', 0:length(theta_1.pos)-1, theta_2.pos, 'r-', 'LineWidth', 1.5)
ylabel ('\theta [deg]')
xlabel ('Time [sec]')
legend('\theta_1','\theta_2')
% xlim ([0 time(end)])

ux = [1; 0; 0; 0]; uy = [0; 1; 0; 0]; uz = [0; 0; 1; 0];
Ox = [uv_len; 0; 0; 1]; Oy = [0; uv_len; 0; 1]; Oz = [0; 0; uv_len; 1];
% num_steps = 50;

% initialize video
writerObj = VideoWriter('twoR_pot_field.avi');
writerObj.FrameRate = 8;
open(writerObj);
h = figure('Position',[100 100 850 600]);
axis tight
set(gca,'NextPlot','replacechildren');

%% Symbolic Linear and Angular Velocity Jacobian of center of mass, k and u calc
% Tsym = eye(4);      % frame homogeneous transform
% TCOM = eye(4);      % center of mass homogeneous transform
% J_Omega = zeros(3,size(DH_table,1),size(DH_table,1)); 
M = zeros(size(Frame,2),size(Frame,2)); u = 0;       % kinetic and potential energy initializations
% 
for j = 1:size(Frame,2)
    if j == 1;
        T(:,:,j) = exp_coor_hom_trans(Frame(j).omega,Frame(j).vel, t(j));
        Js(:,j) = sym([Frame(j).omega'; Frame(j).vel']);
    else
        % exp([S_1]theta_1)*exp([S_2]theta_2)*...*exp([S_j])theta_j
        T(:,:,j) = T(:,:,j-1)*exp_coor_hom_trans(Frame(j).omega,Frame(j).vel, t(j));
        Js(:,j) = Adj_operator(T(:,:,j-1))*[Frame(j).omega'; Frame(j).vel'];
    end
 
    u = u - masses(j)*[0 -9.81 0 0]*T(:,:,j)*Frame(j).Mc(1:4,4);
end

% M matrix compile

% Convert to Jb
Jb = Adj_operator(homogen_inv(T(:,:,size(Frame,2))*Frame(size(Frame,2)).M))*Js;

for j = 1:size(Frame,2)
    
    M = M + masses(j)*transpose(Jb(4:6,j))*Jb(4:6,j)+transpose(Jb(1:3,j))*T(1:3,1:3,j)*[Itensor(j,1) 0 0; 0 Itensor(j,2) 0; 0 0 Itensor(j,3)]*transpose(T(1:3,1:3,j))*Jb(1:3,j);
    
end
 
%% Compute Christoffel symbols and gravitational force vector
C = sym(zeros(size(Frame,2),size(Frame,2)));

for k = 1:size(Frame,2)
    for j = 1:size(Frame,2)
        for i = 1:size(Frame,2)
            ctemp = (1/2)*(diff(M(k,j),t(i))+diff(M(k,i),t(j))-diff(M(i,j),t(k)));
            C(k,j) = C(k,j) + ctemp*tdot(i);            
        end
    end
end

g = jacobian(u,t);

%% Create function from the equations of motion.  A function runs much faster than using the 'subs' operator

% compute torques  
 tau = M*tddot' + C*tdot' + g';
 
 taufun = matlabFunction(tau, 'File', 'tau_fun_file2R', 'Vars', [t, tdot, tddot]);

%% Position visualization
% for j = 1:size(DH_table,1)

%trajectory tracking variable
% end_effect = zeros(3,length(theta_1.time));

    for k = 1:len_(i_min)
        % angle set for animation
        angle_values = [theta_1_test(i_(k,i_min)); theta_2_test(j_(k,i_min))];
%         vel_values = [theta_1.vel(k); theta_2.vel(k)]*pi/180;
%         accel_values = [theta_1.accel(k); theta_2.accel(k)]*pi/180;
        frame_org = [0; 0; 0; 1];
        for i = 1:size(Frame,2)
            
            % initial {0} frame
%             subplot(1,2,1)            
            plot3([0 uv_len], [0 0], [0 0],'k--', [0 0], [0 uv_len], [0 0], 'k--', [0 0], [0 0], [0 uv_len], 'k--', 'LineWidth', 1.5)
            xlabel ('X_0')
            ylabel ('Y_0')
            zlabel ('Z_0')
             hold on
             plot(obstacle_bound(1,:), obstacle_bound(2,:), 'k-', p_start(1), p_start(2), 'bo', p_end(1), p_end(2), 'ro', 'LineWidth', 1.5)

             
           % compute homogeneous transform for the ith frame
            T_curr = double(vpa(subs(T(:,:,i)*Frame(i).M, t, angle_values')));  
            end_effect(:,k) = T_curr(1:3,4);
            
            frame_org_old = frame_org;
            frame_org = T_curr*[0; 0; 0; 1];
            
            % plot new frame
            plot3([dot(ux,frame_org) dot(ux,T_curr*Ox)], [dot(uy,frame_org) dot(uy,T_curr*Ox)], [dot(uz,frame_org) dot(uz,T_curr*Ox)] ,'k--', [dot(ux,frame_org) dot(ux,T_curr*Oy)], [dot(uy,frame_org) dot(uy,T_curr*Oy)], [dot(uz,frame_org) dot(uz,T_curr*Oy)], 'k--', [dot(ux,frame_org) dot(ux,T_curr*Oz)], [dot(uy,frame_org) dot(uy,T_curr*Oz)], [dot(uz,frame_org) dot(uz,T_curr*Oz)], 'k--', 'LineWidth', 1.5)
            plot3([dot(ux,frame_org_old) dot(ux,frame_org)], [dot(uy,frame_org_old) dot(uy,frame_org)],[dot(uz,frame_org_old) dot(uz,frame_org)], 'r--', 'LineWidth', 1.5)
            plot3(end_effect(1,1:k), end_effect(2,1:k), end_effect(3,1:k), 'k-', 'LineWidth', 1.5)
            text(frame_org(1), frame_org(2), frame_org(3), ['\{' num2str(i) '\}'])
            
            limit_extents = max(max(Frame(size(Frame,2)).M));
            xlim([-1 1]*limit_extents);    % establish appropriate axes bounds
            ylim([-1 1]*limit_extents);
            zlim([-1 1]*limit_extents);
%             title ([num2str(theta_1.time(k), '%.2f') ' [sec]'])
            view(0,90)
            grid on
            axis square
        end
         
  
         
%          subplot(1,2,1)
         hold off
         F2(i) = getframe(h);
         writeVideo(writerObj, F2(i));
        
    end

% use 1st frame to get dimensions
[h, w, p] = size(F2(1).cdata);
hf = figure; 
% resize figure based on frame's w x h, and place at (150, 150)
set(hf,'Position', [150 150 w h]);
axis off
% Place frames at bottom left
% movie(hf,F2,1,30,[0 0 0 0]);
close(writerObj);
       


function T = exp_coor_hom_trans(omega,vel, theta)
%% Local function used to compute homogeneous transformations

%% Inputs:
% omega: 3x1 vector of \hat\omega
% vel: 3x1 vector of v
% theta: 1x1 scalar of the current joint angle

%% Outputs:
% homogeneous transform

%% Main function

% create [\hat\omega]
skew_omega = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];

% evaluation T
T = [(eye(3)+sin(theta)*skew_omega + (1-cos(theta))*skew_omega^2), (eye(3)*theta +(1-cos(theta))*skew_omega + (theta- sin(theta))*skew_omega^2)*vel'; zeros(1,3) 1];

end

function AdjT = Adj_operator(T)
% Local function used to perform the Adjoint operation, Def'n 3.20 of Lynch

% Inputs:
% T: 4x4 homogeneous transformation

% Outputs:
% AdjT: 6x6 matrix

% Main function

AdjT = [T(1:3,1:3) zeros(3,3); [0 -T(3,4) T(2,4); T(3,4) 0 -T(1,4); -T(2,4) T(1,4) 0]*T(1:3,1:3) T(1:3,1:3)];

end


function Tinv = homogen_inv(T)
% Local function used to perform the inverse of a homogeneous transform

% Inputs: 
% T: 4x4 homogeneous transform

% Outputs: 
% Tinv: 4x4 inverse of homogenous transform

Tinv = [T(1:3,1:3)' -T(1:3,1:3)'*T(1:3,4); 0 0 0 1];

end

function [thetaset1, thetaset2] = twoR_inv(p, l1, l2)

% Local function used to perform inverse kinematics for a 2R mechanism
% Inputs: 
% p: 3x1 position vector
% l1: link length 1
% l2: link length 2

% Outputs: 
% thetaset1: first solution to the inverse kinematics
% thetaset2: second solution to the inverse kinematics

% theta 2
c2 = (p(1)^2+p(2)^2-l1^2-l2^2)/(2*l1*l2);
s2 = sqrt(1-c2^2);
thetaset1(2) = atan2(s2,c2);
thetaset2(2) = atan2(-s2,c2);

% theta 1
k1 = l1+l2*c2;
k2 = l2*s2;

thetaset1(1) = atan2((k2*p(1)-k1*p(2))/(-k1^2-k2^2),(-k2*p(2)-k1*p(1))/(-k1^2-k2^2));
thetaset2(1) = atan2((-k2*p(1)-k1*p(2))/(-k1^2-k2^2),(k2*p(2)-k1*p(1))/(-k1^2-k2^2));

end

function [omega, vel, theta] = matrix_log(T)

% omega, theta info
if T(1:3,1:3) == eye(3)
    omega = zeros(3,1);
    vel = T(1:3,4)/norm(T(1:3,4));
    theta = norm(T(1:3,4));
elseif trace(T(1:3,1:3)) == -1
    theta = pi;
    omega = (1/(sqrt(2*(1+T(3,3)))))*[T(1,3); T(2,3); 1+T(3,3)];
else
    theta = acos((1/2)*(trace(T(1:3,1:3))-1));
    brac_hat_omega = (1/(2*sin(theta)))*(T(1:3,1:3)-T(1:3,1:3)');
    omega = [brac_hat_omega(3,2); brac_hat_omega(1,3); brac_hat_omega(2,1)];
end

skew_omega = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];

% vel info
Ginv = (1/theta)*eye(3)-(1/2)*skew_omega + (1/theta-(1/2)*cot(theta/2))*skew_omega^2;

vel = Ginv*T(1:3,4);
end
    
