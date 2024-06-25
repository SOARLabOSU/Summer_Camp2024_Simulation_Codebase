function Xd = gen_ref_condition(xd)

Rd=eye(3); % Ref rotation matrix
% xd=[1;1;1]; % ref position
vd=0*[0.1;0.1;0.1]; % ref velocities
wd=0*[0.01;0.01;0.01]; % ref angular velocities
Xd=[xd' vd' Rd(:)' wd']; % as a vector reference

end


