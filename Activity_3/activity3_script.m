close all; clc; clear

addpath function_files
addpath mex_files_windows_intel

%% User input for the MPC horizon / future time that the quadrotor brain can see
while 1

    hor_ip = input(['Select the time into the future that the quadrotor brain can see',...
           '\n1. 0.2 sec',...
           '\n2. 0.5 sec',...
           '\n3. 1 sec',...
           '\n4. 1.5 sec',...
           '\n5. 2 sec',...
           '\n6. 3 sec : \n']);
   
    avail_horizons = [0.2 0.5 1 1.5 2 3];
    
    if ismember(hor_ip, 1:6)
        mpc_horizon = avail_horizons(hor_ip);
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        disp('Input Received ! Thank You !')
        disp('Press Enter to Proceed for Simulation')
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        break;
    else
        disp('*************Invalid Selection************')
        disp('    ')
    end

end

pause;


%% Simulation parameters for the brain
ctrlDT = 0.1; % time interval between two divisions of MPC horizon
mpc_div = mpc_horizon/ctrlDT; % # of divisions of the mpc horizon
% time of simulation
tsim = 10;

%% Define initial conditions
% define the initial rotation matrix
R0=rotx(20)*roty(5)*rotz(30);
% R0=eye(3);
w0=zeros(3,1); % define the initial angular velocities
x0=[0;0;0]; % initial position
v0=zeros(3,1); % initial velocity
% initial conditions as a state vector
x0=[x0' v0' R0(:)' w0'];

%% Define Reference

Rd=eye(3); % Ref rotation matrix
xd=[1;1;1]; % ref position
vd=0*[0.1;0.1;0.1]; % ref velocities
wd=0*[0.01;0.01;0.01]; % ref angular velocities
Xd=[xd' vd' Rd(:)' wd']; % as a vector reference



%% MPC Startup
% initial guess of the decision variables
input.x=repmat(x0,mpc_div+1,1);
input.u=zeros(mpc_div,4);

% incorporate reference inputs
input.y=repmat([Xd zeros(1,4)],mpc_div,1);
input.yN=Xd; % terminal reference input

%% Run Simulation

iter = 0;
time=0;

u_mpc = [];
x_now=x0;

mpc_time = [];

while time(end) < tsim
    tic;
    input.x0=x_now(end,:);

    if hor_ip == 1
        output = acado_MPC_solve_hor_p2_sec(input);
    elseif hor_ip == 2
         output = acado_MPC_solve_hor_half_sec(input);
    elseif hor_ip == 3
        output = acado_MPC_solve_hor_1sec(input);
    elseif hor_ip == 4
        output = acado_MPC_solve_hor_1nhalf_sec(input);
    elseif hor_ip == 5
        output = acado_MPC_solve_hor_2sec(input);
    else 
        output = acado_MPC_solve_hor_3sec(input);
    end

    u_mpc = [u_mpc; output.u(1,:)];
    
    % Shifting of mpc outputs
    input.x = [output.x(2:end,:);output.x(end,:)];
    input.u = [output.u(2:end,:);output.u(end,:)];
    time_elapsed = toc;
    mpc_time = [mpc_time; time_elapsed];
    
    % simulate system
    sim_input.x = x_now(end,:)';
    sim_input.u = output.u(1,:)';
    [states out] = integrator_quad(sim_input);
    x_now = [x_now; states.value'];

    iter=iter+1;
    nextTime = iter*ctrlDT;
    time = [time nextTime];

end

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('Completed Simulation ')
disp('Press Enter to Proceed for visualization')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
pause;

[x,y,z,roll,pitch,yaw] = viz_conversion_states(x_now,time);
video=0;
quad_animation(x,y,z,roll,pitch,yaw,xd,video)


return;
%% plotting
% Initialize classical attitude tracking error
V1=zeros(length(time),1);
% Initialize  the velocity error matrix
We = zeros(length(time),3);
xe=zeros(length(time),3);
ve=zeros(length(time),3);

for jj=1:length(time)
    R=x_now(jj,7:15); % vec:R in time tspan(jj)
    R=reshape(R,3,3);
    V1(jj)=trace(eye(3)-Rd'*R); % classical attitude tracking error
    % wd = 0.001*[sin(0.03*tspan(jj));sin(0.01*tspan(jj));sin(0.02*tspan(jj))];
    w = x_now(jj,16:18);
    We(jj,:) = abs(wd-w');
    x=x_now(jj,1:3); v=x_now(jj,4:6);
    xe(jj,:)=abs(xd-x');
    ve(jj,:)=abs(vd-v');
end

f=figure;
subplot(2,2,1)
plot(time,V1,'LineWidth',3)
grid
title('Attitude Tracking Error')
hold on
xlabel('Time (s)','Interpreter','latex')
ylabel('$\mathbf{V_1 = tr(I-R_d^TR)}$','Interpreter','latex')
% ylim([0 4])
set(gca,'FontSize',20)

subplot(2,2,2)
plot(time,We,'LineWidth',3)
title('Angular Velocity Error')
hold on
xlabel('Time (s)','Interpreter','latex')
ylabel('Error','Interpreter','latex')
legend('\omega_1','\omega_2','\omega_3')
set(gca,'FontSize',20)
% ylim([0 0.1])
grid

subplot(2,2,3)
plot(time,xe,'LineWidth',3)
title('Position')
hold on
xlabel('Time (s)','Interpreter','latex')
ylabel('Error','Interpreter','latex')
legend('x_1','x_2','x_3')
set(gca,'FontSize',20)
% ylim([0 0.1])
grid

subplot(2,2,4)
plot(time,ve,'LineWidth',3)
title('Velocity Error')
hold on
xlabel('Time (s)','Interpreter','latex')
ylabel('Error','Interpreter','latex')
legend('v_1','v_2','v_3')
set(gca,'FontSize',20)
% ylim([0 0.1])
grid

f.Position = [100 100 1200 1000];

%%

figure;
subplot(1,2,1)
stairs(time,[u_mpc(:,1);nan],'-.','LineWidth',3)
title('f')
xlim([0 10])
subplot(1,2,2)
stairs(time(1:end-1),u_mpc(:,2:end),'--','LineWidth',3)
title('M')
xlim([0 10])
