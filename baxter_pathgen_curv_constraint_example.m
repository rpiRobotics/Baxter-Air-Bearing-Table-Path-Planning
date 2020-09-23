% Path Generation Baxter

clear variables; close all; clc

animate_on = 0;
q_curve_limit_on = 1;
kr = 25; % rotational feedback on yaw
kp = 20; % position feedback on x,y

% Specify Initial and Desired Pose, Initial Joint Angles, and Other Parameters
[robot_const, robot_structure] = defineBaxter();
qlimit = [robot_const(1).limit.lower_joint_limit,...
    robot_const(1).limit.upper_joint_limit];

% Specify initial joint angles
q0 = [-0.959931088596881;-0.785398163397448;0;1.570796326794897;0;-0.785398163397448;0]; % y-dir motion only

% Compute initial pose
[R0T0, P0T0] = fwdkin(robot_const(1).kin,q0);
z0 = P0T0(3);

% Specify desired final pose
P0T1 = [0.7;0.3591;0.3267]; % case 1 with only 1 waypoint
P0Td = [0.9132;0.5091;0.3267];

R0Td = R0T0;

ep0 = norm(P0Td - P0T0)^2;
er0 = 0.5*norm( (sqrtm(R0T0*R0Td') - eye(3)),'fro')^2;

% Specify Lambda (path variable)
N = 500;
lambda = [0:1/N:1];

% Compute Path in Task Space
ER0 = R0T0*R0Td';
temp = vrrotmat2vec(ER0);
k_hat = [temp(1);temp(2);temp(3)];
theta0 = temp(4);

P0T_lambda = zeros(3,length(lambda));
theta_lambda = zeros(1,length(lambda));
% constant in lambda
der_dlambda = -theta0;

for k = 1:length(lambda)
    P0T_lambda(:,k) = (1 - lambda(k))*P0T0 + lambda(k)*P0Td;
    theta_lambda(k) = (1 - lambda(k))*theta0;
    %der_dlambda(k) = -10*theta0*sin((theta_lambda(k))/2);
end

% Solve QP Problem and Generate Joint Space Path
epsilon_r = 0.1;
epsilon_p = 0.1;
q_prime_min = -5*ones(7,1); q_prime_max = 5*ones(7,1);
q_primeprime_min = -30*ones(7,1); q_primeprime_max = 30*ones(7,1);
q_prime = zeros(7,length(lambda));
q_lambda = zeros(7,length(lambda));
q_lambda(:,1) = q0;
exitflag = zeros(1,length(lambda));
P0T_lambda = zeros(3,length(lambda));
R0T_lambda = zeros(3,3,length(lambda));
rpy_lambda = zeros(3,length(lambda));
P0T_lambda(:,1) = P0T0;
P0Td_lambda = P0T_lambda;
P0Td_lambda(:,1) = P0T0;
R0T_lambda(:,:,1) = R0T0;
rpyd_lambda = zeros(3,length(lambda));
rpytemp = rotm2eul(R0T0,'XYZ');
rpyd_lambda(:,1) = [rpytemp(1);rpytemp(2);rpytemp(3)];
rot_dev = zeros(1,length(lambda));
rot_dev(1) = acos(dot(R0T0(:,3),[0;0;1]));


qprev = q0;
q_prime_prev = zeros(7,1);
Ptemp = P0T0;
Rtemp = R0T0;
options = optimoptions('quadprog','Display','off');

for k = 1:length(lambda)

    if lambda(k) <= 0.5
        dP0T_dlambda = (1/0.5)*(P0T1 - P0T0);
        P0Td_lambda(:,k) = (1 - lambda(k)/0.5)*P0T0 + lambda(k)/0.5*P0T1;
    else
        dP0T_dlambda = (1/0.5)*(P0Td - P0T1);
        P0Td_lambda(:,k) = (1 - (lambda(k)-0.5)/(1 - 0.5))*P0T1 + (lambda(k)-(1 - 0.5))/0.5*P0Td;
    end
    
    
    rpytemp = rotm2eul(expm(theta_lambda(k)*hat(k_hat))*R0Td,'XYZ');
    rpyd_lambda(:,k) = [rpytemp(1);rpytemp(2);rpytemp(3)];
    
    lb = [q_prime_min;0;0]; ub = [q_prime_max;1;1];
    
    J = robotjacobian(robot_const(1).kin, qprev);

    [Aeqrot,beqrot] = equality_constraints_rotation(Rtemp,R0T0',75,J);
    Aeq = [Aeqrot;[J(6,:),0,0]];
    beq = [beqrot;-75*(Ptemp(3)-z0)];
    
    vt = dP0T_dlambda + kp*[P0Td_lambda(1,k)-Ptemp(1);P0Td_lambda(2,k)-Ptemp(2);0];
    thetax = atan2(Rtemp(3,3),Rtemp(3,2)); 
    thetay = atan2(Rtemp(3,3),Rtemp(3,1));
    rpytemp = rotm2eul(Rtemp,'XYZ');
    vr = der_dlambda*k_hat + kr*[0;0;rpyd_lambda(3,k)-rpytemp(3)]; %+ 50*[(thetax - pi/2);-(thetay - pi/2);0];
    
    H = getqp_H(qprev, J, vr, vt, ...
        epsilon_r, epsilon_p);
    H = (H+H')/2;
    f = getqp_f( qprev, epsilon_r, epsilon_p );

    h = zeros(14,1);
    h(1:7) = qprev - qlimit(:,1);
    h(8:14) = qlimit(:,2) - qprev;

    %parameters for inequality constraints
    c = 0.9;%0.5;
    eta = 0.01;%0.1;
    epsilon_in = 0.15;
    e = 0.002;
    
    sigma = inequality_bound(h, c, eta, epsilon_in, e);
    dhdq = [eye(7) zeros(7, 2); -eye(7) zeros(7, 2)];
    
    A = -dhdq;
    b = -sigma;
    
    if (q_curve_limit_on)&&(k>1)
        A = [A;eye(7),zeros(7,2);-eye(7),zeros(7,2)];
        b = [b;(1/N)*q_primeprime_max + q_prime_prev;-((1/N)*q_primeprime_min + q_prime_prev)];
    end
    
    [q_prime_temp,~,exitflag(k)] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[]...
        ,options);
    q_prime_temp = q_prime_temp(1:7);
    % check exit flag - all elements should be 1
    
    q_prime(:,k) = q_prime_temp;
    q_prime_prev = q_prime_temp;
    qprev = qprev + (1/N)*q_prime_temp;
    q_lambda(:,k+1) = qprev;
    
    [Rtemp, Ptemp] = fwdkin(robot_const(1).kin,qprev);
    P0T_lambda(:,k+1) = Ptemp;
    R0T_lambda(:,:,k+1) = Rtemp;
    rpy_temp = rotm2eul(Rtemp,'XYZ');
    rpy_lambda(:,k) = [rpy_temp(1);rpy_temp(2);rpy_temp(3)];
    rot_dev(k+1) = acos(dot(Rtemp(:,3),[0;0;1]));
end

% Chop off excess
q_lambda(:,end) = [];
P0T_lambda(:,end) = [];
R0T_lambda(:,:,end) = [];
rot_dev(end) = [];

q_primeprime = zeros(7,length(lambda)-1);
path_curve = zeros(3,length(lambda)-1);
for k = 2:length(lambda)
   q_primeprime(:,k) = (q_prime(:,k)-q_prime(:,k-1))./(1/N);
   Jprev = robotjacobian(robot_const(1).kin, q_lambda(:,k-1));
   Jnew = robotjacobian(robot_const(1).kin,q_lambda(:,k));
   path_curve_temp = (Jnew*q_lambda(:,k) - Jprev*q_lambda(:,k-1))./(1/N);
   path_curve(:,k) = path_curve_temp(3:5);
end

% Plot Results

% Check End Position
qf = q_lambda(:,end);
[R0Tf, P0Tf] = fwdkin(robot_const(1).kin,qf);
R0Td
R0Tf
P0Td
P0Tf

figure(1)
for k = 1:7
    if k == 7
        subplot(3,3,8)
    else
        subplot(3,3,k)
    end
    plot(lambda,q_lambda(k,:),'b','LineWidth',2)
    hold on
    plot([0,1],qlimit(k,1)*ones(1,2),'k--','LineWidth',2)
    plot([0,1],qlimit(k,2)*ones(1,2),'k--','LineWidth',2)
    xlabel('$\lambda$','Interpreter','Latex')
    if (k == 1)||(k==4)||(k==7)
        ylabel('q (rad)','Interpreter','Latex')
    end
    title(['q',num2str(k)])
end

figure(2)
for k = 1:7
    if k == 7
        subplot(3,3,8)
    else
        subplot(3,3,k)
    end
    plot(lambda,q_prime(k,:),'b','LineWidth',2)
    hold on
    plot([0,1],[q_prime_min(k),q_prime_min(k)],'k--','LineWidth',2)
    plot([0,1],[q_prime_max(k),q_prime_max(k)],'k--','LineWidth',2)
    xlabel('$\lambda$','Interpreter','Latex')
    if (k == 1)||(k==4)||(k==7)
        ylabel('q'' (rad)','Interpreter','Latex')
    end
    title(['q''',num2str(k)])
end

figure(3)
for k = 1:7
    if k == 7
        subplot(3,3,8)
    else
        subplot(3,3,k)
    end
    plot(lambda,q_primeprime(k,:),'b','LineWidth',2)
    hold on
    plot([0,1],[q_primeprime_min(k),q_primeprime_min(k)],'k--','LineWidth',2)
    plot([0,1],[q_primeprime_max(k),q_primeprime_max(k)],'k--','LineWidth',2)
    xlabel('$\lambda$','Interpreter','Latex')
    if (k == 1)||(k==4)||(k==7)
        ylabel('q'''' (rad)','Interpreter','Latex')
    end
    title(['q''''',num2str(k)])
end

figure(4)
axis_lables = ["x (m)","y (m)"];
for k = 1:2
    subplot(1,3,k)
    plot(lambda,P0T_lambda(k,:),'b','LineWidth',2)
    hold on
    plot(lambda,P0Td_lambda(k,:),'r--','LineWidth',2)
    xlabel('$\lambda$','Interpreter','Latex')
    ylabel(axis_lables(k),'Interpreter','Latex')
end
subplot(1,3,3)
plot(lambda,rpy_lambda(3,:),'b','LineWidth',2)
hold on
plot(lambda,rpyd_lambda(3,:),'r--','LineWidth',2)
axis([0 1 -1 -0.9])
xlabel('$\lambda$','Interpreter','Latex')
ylabel('Yaw (rad)','Interpreter','Latex')
legend('Planned','Desired')
    
figure(5)
subplot(1,3,1)
plot(lambda,P0T_lambda(3,:),'b','LineWidth',2)
hold on
plot(lambda,P0Td_lambda(3,:),'r--','LineWidth',2)
xlabel('$\lambda$','Interpreter','Latex')
ylabel('z (m)','Interpreter','Latex')
legend('Planned','Desired')
axis_lables = ["roll (rad)","pitch (rad)"];
for k = 1:2
    subplot(1,3,1+k)
    plot(lambda,rpy_lambda(k,:),'b','LineWidth',2)
    hold on
    plot(lambda,rpyd_lambda(k,:),'r--','LineWidth',2)
    xlabel('$\lambda$','Interpreter','Latex')
    ylabel(axis_lables(k),'Interpreter','Latex')
end

figure(6)
axis_lables = ["x (m)","y (m)","yaw (rad)"];
for k = 1:3
   subplot(1,3,k)
   plot(lambda,path_curve(k,:),'b','LineWidth',2)
   xlabel('$\lambda$','Interpreter','Latex')
   ylabel(axis_lables(k),'Interpreter','Latex')
end

figure(7)
baxter = createCombinedRobot(robot_const, robot_structure);
axis equal;
axis([-2 2 -2 2 -.85 1]);
view([109 27]);
counter = 1;
if animate_on
    q = get_angle_structure(baxter);
    for k = 1:length(lambda)
        q(1).state = q_lambda(:,k)';
        q(2).state = [-45 0 0 90 0 0 0]*pi/180;
        baxter = updateRobot(q, baxter);
        xlabel('x')
        ylabel('y')
        zlabel('z')
        drawnow
        M(counter) = getframe(4);
        counter = counter+1;
    end
    
end

