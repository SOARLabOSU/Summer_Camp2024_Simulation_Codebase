function [Xd,xd] = gen_torus_knot(iter,ctrlDT)

xd = [sin(0.1*iter*ctrlDT)+2*sin(2*0.1*iter*ctrlDT);....
          cos(0.1*iter*ctrlDT)-2*cos(2*0.1*iter*ctrlDT);....
          -sin(3*0.1*iter*ctrlDT)];
vd = [0.1*cos(0.1*iter*ctrlDT)+0.4*cos(2*0.1*iter*ctrlDT);....
        -0.1*sin(0.1*iter*ctrlDT)+0.4*sin(2*0.1*iter*ctrlDT);...
         -0.3*cos(3*0.1*iter*ctrlDT)];
Rd=eye(3); % Ref rotation matrix
wd=0*[0.01;0.01;0.01]; % ref angular velocities
Xd=[xd' vd' Rd(:)' wd'];

end


