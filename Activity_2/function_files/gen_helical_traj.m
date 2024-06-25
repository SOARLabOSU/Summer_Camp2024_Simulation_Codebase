function [Xd,xd] = gen_helical_traj(iter,ctrlDT)

xd = [iter*ctrlDT/40; sin(iter*ctrlDT/5); cos(iter*ctrlDT/5)];
vd = [1; cos(iter*ctrlDT/5)/5; -sin(iter*ctrlDT/5)/5];
Rd=eye(3); % Ref rotation matrix
wd=0*[0.01;0.01;0.01]; % ref angular velocities
Xd=[xd' vd' Rd(:)' wd'];

end


