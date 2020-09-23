% Path Generation Baxter

clear variables; close all; clc

animate_on = 0;

% Specify Initial and Desired Pose, Initial Joint Angles, and Other Parameters
[robot_const, robot_structure] = defineBaxter();
qlimit = [robot_const(1).limit.lower_joint_limit,...
    robot_const(1).limit.upper_joint_limit];

qlimit = qlimit + [0,0;0,0;0,0;0,0;10*pi/180,-10*pi/180;10*pi/180,-10*pi/180;0,0];

% Specify initial joint angles
q0 = [-0.959931088596881;-0.785398163397448;0;1.570796326794897;0;-0.785398163397448;0]; % Long Path (Kims)

% Compute initial pose
[R0T0, P0T0] = fwdkin(robot_const(1).kin,q0);
z0 = P0T0(3);

% Specify desired final pose
P0Td = [0.3451;0.8451;0.3267]; % Long Path
R0Td = rotz(-20*pi/180)*R0T0;

ep0 = norm(P0Td - P0T0)^2;
er0 = 0.5*norm( (sqrtm(R0T0*R0Td') - eye(3)),'fro')^2;

% Specify Lambda (path variable)
N = 1000;
lambda = [0:1/N:1];

% Compute Path in Task Space
ER0 = R0T0*R0Td';
temp = vrrotmat2vec(ER0);
k_hat = [temp(1);temp(2);temp(3)];
theta0 = temp(4);

P0T_lambda = zeros(3,length(lambda));
theta_lambda = zeros(1,length(lambda));
dP0T_dlambda = (P0Td - P0T0); % constant in lambda
der_dlambda = zeros(1,length(lambda));

for k = 1:length(lambda)
    P0T_lambda(:,k) = (1 - lambda(k))*P0T0 + lambda(k)*P0Td;
    theta_lambda(k) = (1 - lambda(k))*theta0;
    %der_dlambda(k) = -10*theta0*sin((theta_lambda(k))/2);
    der_dlambda(k) = -theta0;
end

% Solve QP Problem and Generate Joint Space Path
epsilon_r = 0.1;
epsilon_p = 0.1;
q_prime_min = -5*ones(7,1); q_prime_max = 5*ones(7,1);
q_prime = zeros(7,length(lambda));
q_lambda = zeros(7,length(lambda));
q_lambda(:,1) = q0;
exitflag = zeros(1,length(lambda));
P0T_lambda = zeros(3,length(lambda));
R0T_lambda = zeros(3,3,length(lambda));
thetaz = ones(1,length(lambda));
P0T_lambda(:,1) = P0T0;
R0T_lambda(:,:,1) = R0T0;
rot_dev = zeros(1,length(lambda));
rot_dev(1) = acos(dot(R0T0(:,3),[0;0;1]));


qprev = q0;
Ptemp = P0T0;
Rtemp = R0T0;
options = optimoptions('quadprog','Display','off');

for k = 1:length(lambda)

    [lb,ub] = qprimelimits_full(qlimit,qprev,N,q_prime_max,q_prime_min);
    J = robotjacobian(robot_const(1).kin, qprev);

    [Aeqrot,beqrot] = equality_constraints_rotation(Rtemp,R0T0',75,J);
    Aeq = [Aeqrot;[J(6,:),0,0]];
    beq = [beqrot;-75*(Ptemp(3)-z0)];
    
    vt = dP0T_dlambda;
    thetax = atan2(Rtemp(3,3),Rtemp(3,2)); 
    thetay = atan2(Rtemp(3,3),Rtemp(3,1));
    vr = der_dlambda(k)*k_hat; %+ 50*[(thetax - pi/2);-(thetay - pi/2);0];
    H = getqp_H(qprev, J, vr, vt, ...
        epsilon_r, epsilon_p);
    f = getqp_f( qprev, epsilon_r, epsilon_p );

    [q_prime_temp,~,exitflag(k)] = quadprog(H,f,[],[],Aeq,beq,lb,ub,[]...
        ,options);
    q_prime_temp = q_prime_temp(1:7);
    % check exit flag - all elements should be 1
    
    q_prime(:,k) = q_prime_temp;
    qprev = qprev + (1/N)*q_prime_temp;
    q_lambda(:,k+1) = qprev;
    
    [Rtemp, Ptemp] = fwdkin(robot_const(1).kin,qprev);
    P0T_lambda(:,k+1) = Ptemp;
    R0T_lambda(:,:,k+1) = Rtemp;
    thetaz(k) = atan2(Rtemp(2,1),Rtemp(1,1));
    rot_dev(k+1) = acos(dot(Rtemp(:,3),[0;0;1]));
end

% Chop off excess
q_lambda(:,end) = [];
P0T_lambda(:,end) = [];
R0T_lambda(:,:,end) = [];
rot_dev(end) = [];

joint_plots(1,'$\lambda$','q (rad)',lambda,q_lambda,qlimit)

% Check End Position
qf = q_lambda(:,end);
[R0Tf, P0Tf] = fwdkin(robot_const(1).kin,qf);
R0Td
R0Tf
P0Td
P0Tf

% Plot Trajectory in task space (look for error in z-dir)
figure(2)
subplot(2,2,1)
plot(lambda,P0T_lambda(1,:))
xlabel('lambda')
ylabel('x-dir')
subplot(2,2,2)
plot(lambda,P0T_lambda(2,:))
xlabel('lambda')
ylabel('y-dir')
subplot(2,2,3)
plot(lambda,P0T_lambda(3,:))
xlabel('lambda')
ylabel('z-dir')
axis([0 inf 0.32 0.33])

% Plot deviation from contrained orientation
figure(3)
plot(lambda,rot_dev)

figure(4)
plot(lambda,thetaz*180/pi,'b','LineWidth',2)
xlabel('lambda')
ylabel('theta z (deg)')

figure(5);
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
