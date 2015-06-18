function M = quaternion2Matrix(quat)
%returns the rotation matrix based on a quaternion, quaternion's scalar
%part must be last

q13 = [quat(1) quat(2) quat(3)]';

M = (2*(quat(4)*quat(4)) - 1)*eye(3) + 2*(q13*q13') - 2*quat(4)*skew(q13);


end