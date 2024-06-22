close all; clc;

%{

This script demonstrates a Torus Knot

%}

addpath function_files
addpath mex_files_intel_mac
%addpath mex_files_windows_intel

%% Parameters for Simulattion
mpc_horizon=4;
ctrlDT = 0.1; % time interval between two divisions of MPC horizon
mpc_div = mpc_horizon/ctrlDT; % # of divisions of the mpc horizon
% time of simulation
tsim = 70;

% define the initial rotation matrix
% R0=rotx(5)*roty(25)*rotz(30);
R0=eye(3);
w0=zeros(3,1); % define the initial angular velocities
x0=[0;-1;0]; % initial position
v0=[0.5;0;-0.3]; % initial velocity

% initial conditions as a state vector
x0=[x0' v0' R0(:)' w0'];


% initial guess of the decision variables
input.x=repmat(x0,mpc_div+1,1);
input.u=zeros(mpc_div,4);

Rd=eye(3); % Ref rotation matrix
wd=0*[0.01;0.01;0.01]; % ref angular velocities
% vd=0*[0.1;0.1;0.1]; % ref velocities



%% Run Sim
iter = 0;
time=0;
u_mpc = [];
x_now=x0;

xds=[]; %store reference trajectory

while time(end) < tsim
    
    % define reference trajectory position & velocities
    xd = [sin(0.1*iter*ctrlDT)+2*sin(2*0.1*iter*ctrlDT);....
          cos(0.1*iter*ctrlDT)-2*cos(2*0.1*iter*ctrlDT);....
          -sin(3*0.1*iter*ctrlDT)];
    xds = [xds; xd'];
    vd = [0.1*cos(0.1*iter*ctrlDT)+0.4*cos(2*0.1*iter*ctrlDT);....
        -0.1*sin(0.1*iter*ctrlDT)+0.4*sin(2*0.1*iter*ctrlDT);...
         -0.3*cos(3*0.1*iter*ctrlDT)];
    Xd=[xd' vd' Rd(:)' wd']; % reference vector current

    % inputs to MPC
    input.x0=x_now(end,:);
    input.y=repmat([Xd zeros(1,4)],mpc_div,1);
    input.yN=Xd; % terminal reference input
    % run MPC
    output = acado_MPC_solve(input);
    u_mpc = [u_mpc; output.u(1,:)];
    
    % Shifting of mpc outputs
    input.x = [output.x(2:end,:);output.x(end,:)];
    input.u = [output.u(2:end,:);output.u(end,:)];

    % simulate system
    sim_input.x = x_now(end,:)';
    sim_input.u = output.u(1,:)';
    [states out] = integrator_quad(sim_input);
    x_now = [x_now; states.value'];

    iter=iter+1;
    nextTime = iter*ctrlDT;
    time = [time nextTime];

end


%% Visualization
[x,y,z,roll,pitch,yaw] = viz_conversion_states(x_now,time);
video=0;
speed_flag=1;
quad_animation(x,y,z,roll,pitch,yaw,xds(1,:),time,speed_flag,video)
hold on
plot3(xds(:,1),xds(:,2),xds(:,3))