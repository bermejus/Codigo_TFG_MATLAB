function res = euler_angles(q)
    if size(q,1) == 1 || size(q,2) == 1
        roll = atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)^2 + q(4)^2));
        pitch = asin(2*(q(1)*q(3) - q(4)*q(2)));
        yaw = atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(2)^2 + q(3)^2));
    else
        roll = atan2(2*(q(:,1).*q(:,4) + q(:,2).*q(:,3)), 1 - 2*(q(:,3).^2 + q(:,4).^2));
        pitch = asin(2*(q(:,1).*q(:,3) - q(:,4).*q(:,2)));
        yaw = atan2(2*(q(:,1).*q(:,2) + q(:,3).*q(:,4)), 1 - 2*(q(:,2).^2 + q(:,3).^2));
    end
    
    res = [roll, pitch, yaw];
end