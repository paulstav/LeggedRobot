function H = measur(qk, pk1 , pk2, rk)
%measurement matrix based on the error of the leg kinematics minus the
%estimated position of the footpoins calculated by the difference C*(p-r)

%Cq = quat2dcm([qk(4) qk(1) qk(2) qk(3)]);
Cq = quaternion2Matrix(qk);


%Cq = 2*(scalar*scalar - 1)*eye(3) + 2*q13*q13' - 2*scalar*skew(q13);

H = [-Cq zeros(3) skew((Cq*(pk1-rk))) Cq zeros(3) zeros(3);
     -Cq zeros(3) skew(Cq*(pk2-rk)) zeros(3) Cq zeros(3)];
     %zeros(3) zeros(3) skew(Cq*(pk2-pk1)) -Cq Cq zeros(3)];
       
    
end