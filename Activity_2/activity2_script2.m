close all; clc; clear

addpath function_files
% addpath mex_files_intel_mac
addpath mex_files_windows_intel


disp('%%%%%%%%%%%%%%%%%%%%%%%- Activity 2 - %%%%%%%%%%%%%%%%%%%%%%%%')
disp('                     Following a Torus Knot           ')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')
disp('Press Enter to Proceed')
disp(' ')
pause;
%% User input for starting position of the Quadrotor
x0 = zeros(3,1);

% take input for starting point
while 1

    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    start_ip = input(['Enter 1 if starting at 0 position'...
                       '\nEnter 2 to set manual starting point : ']);
    if start_ip == 1
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
        disp('Input Received ! Thank You !')
        disp('Press Enter to Proceed for Simulation')
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
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

pause;

%% Timing parameters for the brain of the quadrotor
mpc_horizon=4; % length of MPC horizon / time in the future that the brain can forsee
ctrlDT = 0.1; % time interval between two divisions of MPC horizon
mpc_div = mpc_horizon/ctrlDT; % # of divisions of the mpc horizon

% time of simulation
tsim = 70;



%% Define initial conditions
X0 = gen_init_condition(x0);


%% MPC Startup
input.x=repmat(X0,mpc_div+1,1);
input.u=zeros(mpc_div,4);


%% Run MPC Sim
iter = 0;
time=0;
u_mpc = [];
x_now=X0;


xds=[];

while time(end) < tsim
    
    
    % Define Reference conditions
    [Xd,xd] = gen_torus_knot(iter,ctrlDT);
    xds = [xds; xd'];

    % incorporate reference inputs
    input.y=repmat([Xd zeros(1,4)],mpc_div,1);
    input.yN=Xd; % terminal reference input

    % MPC solve step
    input.x0=x_now(end,:);
    output = acado_MPC_solve(input);
    
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
video=0;
speed_flag=1;
quad_animation(x,y,z,roll,pitch,yaw,x0,time,speed_flag,video)

hold on
plot3(xds(:,1),xds(:,2),xds(:,3))
