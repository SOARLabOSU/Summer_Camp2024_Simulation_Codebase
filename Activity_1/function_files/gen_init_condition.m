function X0 = gen_init_condition(x0)

R0=rotx(20)*roty(5)*rotz(30);
w0=zeros(3,1); % define the initial angular velocities
v0=zeros(3,1); % initial velocity
% initial conditions as a state vector
X0=[x0' v0' R0(:)' w0'];

end


