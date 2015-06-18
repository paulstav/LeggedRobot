function q = rot_vector2quaternion(vector)
%returns the quaternion based on the rotation vector scalar part is at the
%end

norm = sqrt(vector(1)*vector(1)+.....
    vector(2)*vector(2) +....
    vector(3)*vector(3));

if norm > 0.1

q1 = vector(1)*sin(norm/2)/norm;
q2 = vector(2)*sin(norm/2)/norm;
q3 = vector(3)*sin(norm/2)/norm;
q4 = cos(norm/2);
else
    q1 = vector(1)/2;
    q2 = vector(2)/2;
    q3 = vector(3)/2;
    q4 = 1;
end

q = [q1 q2 q3 q4]';
end