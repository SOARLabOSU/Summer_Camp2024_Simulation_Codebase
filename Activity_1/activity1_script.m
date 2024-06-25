close all; clc; clear

addpath function_files
% addpath mex_files_intel_mac
addpath mex_files_windows_intel

disp('%%%%%%%%%%%%%%%%%%%%%%%- Activity 1 - %%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('                           Hover Task                              ')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')
disp('Press Enter to Proceed')
disp(' ')
pause;

%% User input for starting & desired position of the Quadrotor
xd=zeros(3,1);
x0 = zeros(3,1);

% take input for starting point
while 1

    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    start_ip = input(['Enter 1 if starting at 0 position'...
                       '\nEnter 2 to set manual starting point \n: ']);
    if start_ip == 1
        break;
    elseif start_ip == 2
        x0(1) = input('Enter initial x co-ordinate (-4 m to 4 m): ');
        x0(2) = input('Enter initial y co-ordinate (-4 m to 4 m): ');
        x0(3) = input('Enter initial z co-ordinate (-4 m to 4 m): ');

        if max(abs(x0)) <=4.1
            disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
            disp('Input Received ! Thank You !')
            disp('Press Enter to Proceed for Simulation')
            disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
            break;
        else
            disp('    ')
            disp('*************Invalid input************')
            disp('    ')
        end
    else
         disp('    ')
        disp('*************Invalid input************')
        disp('    ') 
    end


end


% take input for the desired point

while 1

    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

    xd(1) = input('Enter desired x co-ordinate (-4 m to 4 m): ');
    xd(2) = input('Enter desired y co-ordinate (-4 m to 4 m): ');
    xd(3) = input('Enter desired z co-ordinate (-4 m to 4 m): ');

    if max(abs(xd)) <=4.1
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        disp('Input Received ! Thank You !')
        disp('Press Enter to Proceed for Simulation')
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        break;
    else
        disp('    ')
        disp('*************Invalid input************')
        disp('    ')
    end

end

pause;


%% Timing parameters for the brain of the quadrotor
mpc_horizon=4; % length of MPC horizon / time in the future that the brain can forsee
ctrlDT = 0.1; % time interval between two divisions of MPC horizon
mpc_div = mpc_horizon/ctrlDT; % # of divisions of the mpc horizon

% time of simulation
tsim = 10;

%% Define initial conditions
X0 = gen_init_condition(x0);

%% Define Reference conditions
Xd = gen_ref_condition(xd);


%% MPC Startup

% initial guess of the decision variables
input.x=repmat(X0,mpc_div+1,1);
input.u=zeros(mpc_div,4);

% incorporate reference inputs
input.y=repmat([Xd zeros(1,4)],mpc_div,1);
input.yN=Xd; % terminal reference input

%% Run MPC Simulation

iter = 0;
time=0;

u_mpc = [];
x_now=X0;


while time(end) < tsim
    
    % MPC solve step
    input.x0=x_now(end,:);
    output = acado_MPC_solve(input);
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

