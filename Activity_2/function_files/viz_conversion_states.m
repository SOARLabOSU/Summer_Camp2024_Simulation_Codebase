function [x,y,z,x_rot,y_rot,z_rot] = viz_conversion_states(x_now,time)

x = x_now(:,1);
y = x_now(:,2);
z = x_now(:,3);

for i=1:length(time)
    R = reshape(x_now(i,7:15),3,3);
    eulang = rotm2eul(R);
    x_rot(i,1) = eulang(3);
    y_rot(i,1) = eulang(2);
    z_rot(i,1) = eulang(1);
end

end


