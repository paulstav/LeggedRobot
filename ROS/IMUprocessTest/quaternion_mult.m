function prod = quaternion_mult(q,p)
%returns the quaternion product , quaternions scalar part in the end

pr1 = q(4)*p(1) + q(3)*p(2) - q(2)*p(3) + q(1)*p(4);

pr2 = -q(3)*p(1) + q(4)*p(2) + q(1)*p(3) + q(2)*p(4);

pr3 = q(2)*p(1) - q(1)*p(2) + q(4)*p(3) + q(3)*p(4);

pr4 = -q(1)*p(1) - q(2)*p(2) - q(3)*p(3) + q(4)*p(4);

prod = [ pr1 pr2 pr3 pr4]';

end