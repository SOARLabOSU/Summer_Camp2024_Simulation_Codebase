function thrust_mat = individual_rotor_thrust_compute(u_mpc)

d=0.315;
Ctau = 8.004e-4;

conv_matrix = [1 1 1 1; 0 -d 0 d; d 0 -d 0; -Ctau Ctau -Ctau Ctau];

for i=1:length(u_mpc)
    fM = u_mpc(i,:)';
    fs = conv_matrix\fM;
    thrust_mat(i,:) = fs';
end
end


