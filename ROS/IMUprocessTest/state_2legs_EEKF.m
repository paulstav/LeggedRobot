function [r ,u ,q, p1, p2, b_w] = state_2legs_EEKF(z_leg1, z_leg2, .......
    omega, f, dt,g,touched, foot1pos, foot2pos, touch_first)
%all the required vectors are column vectors


global wf ww  wbw wp %sigma values of noise 
global P Kal_gain first C R dx H S
global r_es u_es q_es p1_es bias_w p2_es  call r_es_p wp1 wp2

measures = 2;

%check if is the first call to function and initialize

if isempty(first)
    
%initialize sigmas of noise    
sigmaf = 0.01;
sigmaw =0.044*pi/180;%0.44%pi/180
sigmabw =  0.007*pi/180;
sigmawp1 = 0.004;
sigmawp2 = 0.004;    


wf = sigmaf*sigmaf;
ww = sigmaw*sigmaw;
wbw = sigmabw*sigmabw;
wp1 = sigmawp1*sigmawp1;
wp2 = sigmawp2*sigmawp2;

%initial conditions
r_es = [0 0.3 0]';
u_es = [-0.1101 0 0]';
q_es = [0 0 0 1];
p1_es = [0.27 0 0.195]';
p2_es = [0.27 0 -0.195]';
bias_w = 0*[1 1 1]';

P = 0.0001*eye(18);
%P = [0.0002*eye(6) 0.000001*eye(6) zeros(6);
 %    0.0001*eye(6) 0.001*eye(6) 0.001*eye(6);
  %   0.00001*eye(6) 0.0001*eye(6) 0.0001*eye(6)];
 
first = 1;

if (measures ==3) 
    R = 0.001*eye(9);
else
    R = 0.01*eye(6);
end

dx = zeros(18,1);
call = 1;
end


G2 = GAMMA((omega-bias_w), dt, 2);
%C = quat2dcm([q_es(4) q_es(1) q_es(2) q_es(3)]);

%JPL notation is left handed, so inverse quaternion and go on

qc = [-q_es(1) -q_es(2) -q_es(3) q_es(4)];
C = quaternion2Matrix(qc);

%C = quaternion2Matrix(q_es);

G1 = GAMMA((omega-bias_w),dt,1);
qp = rot_vector2quaternion(dt*(omega-bias_w));



%every state is 3 dimensional vector

r_es = r_es + dt*u_es + G2'*C'*(f) + 1/2*dt*dt*g;
%r =r + u*dt + 0.5*dt*dt*C*f + 0.5*dt*dt*g;

u_es = u_es + G1'*C'*(f)+ dt*g;
%u = u + dt*C*f + dt*g;

q_es = quaternion_mult(qp,q_es);


%bias_f = bias_f;
bias_w = [0 0 0]';



%
if (touched == 1)%check from loop is foot is on the ground
    
    if (touch_first ==1)%check if 1st touch 
    p1_es = [r_es(1) + z_leg1(1) 0 0.195]'; 
    p2_es = [r_es(1) + z_leg2(1) 0 -0.195]';
    %p1_es = foot1pos;
    %p2_es = foot2pos;  
    
    else   %if not 1st touch then foots velocity zero so...
        
        p1_es = p1_es;
        p2_es = p2_es;
    end
    
    
qc = [-q_es(1) -q_es(2) -q_es(3) q_es(4)];
C = quaternion2Matrix(qc);



%main EKF equations
[Fd,Qd] = Disc_Dynamics(f, omega-bias_w, qc, wf, ww,wbw, wp1, wp2, dt);

Qd

P = Fd*P*Fd' + Qd;

%
H = measur(qc, p1_es, p2_es, r_es);
%
y1 = z_leg1 - C*(p1_es-r_es);
y2 = z_leg2 - C*(p2_es - r_es);


%3rd measurement not affecting the convergence so no need at the moment
if (measures == 3)
    
y3 = [0 0 -0.39]' - C*(p2_es - p1_es);
y = [y1;y2;y3];

else
%
y = [y1;y2];
end

%

S = H*P*H' + R;

%
Kal_gain = P*H'/S;  %the s is inversed but matlab suggests use this notation than *inv(S)
%

dx = Kal_gain * (y + H*dx); % iterative KF 
%

P = (eye(18) - Kal_gain*H)*P;  % use this at the moment since the other notation doesnt affect convergence

%P = (eye(18) - Kal_gain*H)*P*((eye(18) - Kal_gain*H))' + Kal_gain*R*Kal_gain';

% another method of calculating the quaternion's error but doenst seem to
% help on convergence


%dqe = [dx(7)/2 dx(8)/2 dx(9)/2]';


%dq4 = sqrt(1-dqe'*dqe);


%dqpe = [dqe;dq4];


% correct the states with the calculated errors
r_es = r_es + dx(1:3);
u_es = u_es + dx(4:6);

dqe = rot_vector2quaternion(-dx(7:9));

q_es = quaternion_mult(dqe, q_es);
%q_es = quaternion_mult(dqpe,q_es);

%no need to use the other corrections when the pitch angle is <5 deg since
%even very small bias is considered as a small rotation


p1_es = p1_es + dx(10:12);
p2_es = p2_es + dx(13:15);
bias_w = bias_w + dx(16:18);




end
%}
%


r = r_es;
u = u_es;
q = q_es;
p1 = p1_es ;
p2 = p2_es ;
b_w = bias_w;
r_es_p = r_es;

end