close all; clc; clear

addpath function_files
% addpath mex_files_intel_mac
addpath mex_files_windows_intel

disp('%%%%%%%%%%%%%%%%%%%%%%%- Activity 3 - %%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('     Changing the prediction horizon of the quadrotor brain        ')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' The quadrotor is at 0 position and we try to reach (x,y,z) = (1,1,1) ')
disp('Press Enter to Proceed')
disp(' ')
pause;

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


%% Timing parameters for the brain of the quadrotor
ctrlDT = 0.1; % time interval between two divisions of MPC horizon
mpc_div = mpc_horizon/ctrlDT; % # of divisions of the mpc horizon

% time of simulation
tsim = 10;

%% Define initial conditions
x0=[0;0;0];
X0 = gen_init_condition(x0);

%% Define Reference
xd = [1;1;1];
Xd = gen_ref_condition(xd);


%% MPC Startup
% initial guess of the decision variables
input.x=repmat(X0,mpc_div+1,1);
input.u=zeros(mpc_div,4);

% incorporate reference inputs
input.y=repmat([Xd zeros(1,4)],mpc_div,1);
input.yN=Xd; % terminal reference input

%% Run Simulation MPC Simulation

iter = 0;
time=0;

u_mpc = [];
x_now=X0;


while time(end) < tsim
    
    % MPC solve step
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
    % control signal computed
    u_mpc = [u_mpc; output.u(1,:)];
    
    % Shifting of mpc outputs
    input.x = [output.x(2:end,:);output.x(end,:)];
    input.u = [output.u(2:end,:);output.u(end,:)];

    
    % simulate the quadrotor system
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
video_flag=0;
speed_flag=0;
quad_animation_h(x,y,z,roll,pitch,yaw,xd,x0,time,speed_flag,video_flag)

% thrust_mat = individual_rotor_thrust_compute(u_mpc);

%% plotting errors

xe=zeros(length(time),3);
ve=zeros(length(time),3);

for jj=1:length(time)
    x=x_now(jj,1:3); 
    xe(jj,:)=abs(xd-x');
end

f=figure;
plot(time,xe,'LineWidth',3)
title('Position Error with time')
hold on
xlabel('Time (s)','Interpreter','latex')
ylabel('Error','Interpreter','latex')
legend('x_1','x_2','x_3')
set(gca,'FontSize',20)
% ylim([0 0.1])
grid

f.Position = [100 100 1200 1000];
set(gca,'FontSize',20)
% ylim([0 0.1])
grid

f.Position = [100 100 1200 1000];

