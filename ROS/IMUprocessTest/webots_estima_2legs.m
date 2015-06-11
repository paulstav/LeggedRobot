clear all
clc

load gyrox
%load mastrosim_2legs
load mastrosim_5

%acccelerations
ax = WCoM_Acc_p(:,1);
%+ 1/100*gyrox(1:length(CoM_gyro(:,1)));
ay = WCoM_Acc_p(:,2);
%+ 1/100*gyrox(1:length(CoM_gyro(:,1)));
az = WCoM_Acc_p(:,3);
%+ 1/100*gyrox(1:length(CoM_gyro(:,1)));

 %anglular rates
wx = zeros(length(Wthdp),1) + pi/180*gyrox(length(ax),1);
wy = zeros(length(Wthdp),1) + pi/180*gyrox(length(ax),1);
wz = Wthdp' + pi/180*gyrox(length(ax),1);

%initialize variables
yaw = zeros(length(wx),1);
pitch = yaw;
roll = pitch;
s_es4f = [s_es4(:,1) s_es4(:,2) -s_es4(:,3)];

rg = [0 0.3 0];
ug = [ 0 0 0];
p_f = [0 0 0];



for i = 1:length(wx)
    
    if ((Wspring1_pos_p(i) > 0.004)&&(Wspring4_pos_p(i) > 0.004))
        touch = 1;
        
        touching(i) = touch;
        
        if (Wspring1_pos_p(i-1) <= 0.006)
            first_touch = 1;
        else
            first_touch = 0;
        end
    else 
        first_touch = 0;
        touch = 0;
        touching(i) = touch;
    end
    
    
    [rp, up, qg, p_foot1,p_foot2, bwp ] = ..............
        state_2legs_EEKF(s_es1(i,:)',s_es4f(i,:)' , ..........
        [wx(i) wy(i) wz(i)]',[ax(i) ay(i) az(i)]',.......
        0.001, [0 -9.81 0]', touch,WFoot1_Gps(i,:)', WFoot4_Gps(i,:)', ...............
        first_touch);
    
    rg = [rg; rp'];
    ug = [ug; up'];
    p_f = [p_f; p_foot1']; 
    
   p = [qg(4) qg(1) qg(2) qg(3)];
    
    [yaw(i), pitch(i), roll(i)] = quat2angle(p);
end
    yaw = yaw';
    pitch = pitch';
    roll = roll';