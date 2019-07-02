clear all
%%%%clearing old data and loading rally stage data
load('data_3.mat');
l = length(t);
vel = V.*(1000/3600);
fcl = 10;                        %%%%%%initializing variables for bw2 lowpass filter cutoff freq
fsl = 100;                          %%% samp freq
gl = tan(pi*fcl/fsl);           %%%gamma
dl = gl*gl+gl*sqrt(2)+1;        %%%denominator
b0l = (gl*gl)/dl;               %%%%alpha and beta variables
b1l = (2*gl*gl)/dl;
b2l = (gl*gl)/dl;
a1l = (2*(gl*gl-1))/dl;
a2l = (gl*gl-gl*sqrt(2)+1)/dl;
% f_cut = .76;                    %%%%%%initializing variables for bw2 bandpass filter
% f_samp = 100;
% b_width =10.1;
% gb = tan(pi*f_cut/f_samp);
% db = (1+gb*gb)*f_cut + gb*b_width;
% b0b = b_width*gb/db;
% b1b = 0;
% b2b = -b0b;
% a1b = 2*f_cut*((gb*gb)-1);
% a2b = (1+gb*gb)*f_cut - gb*b_width;
psi_rad = psidot.*(pi/180);     %%%%converting psidot to radians
ax_actual = ax.*9.8;               %%%converting accel from g's to m/s
ay_actual = ay.*9.8;

for i = 3500:4500
    if i >= 3500 && i < 3502
        ax_filt_actual(i) = b0l*ax_actual(i) + b1l*ax_actual(i-1) + b2l*ax_actual(i-2);     %%%filtering accel data
        ay_filt_actual(i) = b0l*ay_actual(i) + b1l*ay_actual(i-1) + b2l*ay_actual(i-2);
        %x_accel_filt(i) = b0b*ax_actual(i) + b1b*ax_actual(i-1) + b2b*ax_actual(i-2);
        %y_accel_filt(i) = b0b*ay_actual(i) + b1b*ay_actual(i-1) + b2b*ay_actual(i-2);
    else
        ax_filt_actual(i) = b0l*ax_actual(i) + b1l*ax_actual(i-1) + b2l*ax_actual(i-2) - a1l*ax_filt_actual(i-1) - a2l*ax_filt_actual(i-2);
        ay_filt_actual(i) = b0l*ay_actual(i) + b1l*ay_actual(i-1) + b2l*ay_actual(i-2) - a1l*ay_filt_actual(i-1) - a2l*ay_filt_actual(i-2);
        %x_accel_filt(i) = b0b*ax_actual(i) + b1b*ax_actual(i-1) + b2b*ax_actual(i-2) - a1b*x_accel_filt(i-1) - a2b*x_accel_filt(i-2);
        %y_accel_filt(i) = b0b*ay_actual(i) + b1b*ay_actual(i-1) + b2b*ay_actual(i-2) - a1b*y_accel_filt(i-1) - a2b*y_accel_filt(i-2);  
    end
end
for i = 3495:3500
    slope(i-3494) = (y(i)-y(i-1))/(x(i)-x(i-1));    %%%claculating initial yaw angle
end
yaw_zero = atan(mean(slope))

for i = 3500:4500
    if i == 3500
        yaw(i) = yaw_zero + ((psi_rad(i) + psi_rad(i-1))/2)*.01;    %%%integrating psidot to get yaw angle
        time(i) = t(i);
    else
        yaw(i) = yaw(i-1) + ((psi_rad(i) + psi_rad(i-1))/2)*.01;
        time(i) = t(i);
    end
end

ax_global_low_filt = ax_filt_actual.*cos(yaw) - ay_filt_actual.*sin(yaw);   %%%converting boday accel into global accel
ay_global_low_filt = ax_filt_actual.*sin(yaw) + ay_filt_actual.*cos(yaw);
%ax_global_band_filt = x_accel_filt.*cos(yaw) - y_accel_filt.*sin(yaw);
%ay_global_band_filt = x_accel_filt.*sin(yaw) + y_accel_filt.*cos(yaw);

for i = 3500:4500
    if i == 3500
        vx_global_low_filt(i) = vel(i)*cos(yaw_zero);   %%%%integrating global accel to global vel
        vy_global_low_filt(i) = vel(i)*sin(yaw_zero);
        %vx_global_band_filt(i) = vel(i)*cos(yaw_zero);
        %vy_global_band_filt(i) = vel(i)*sin(yaw_zero);
    else
        vx_global_low_filt(i) = vx_global_low_filt(i-1) + ((ax_global_low_filt(i) + ax_global_low_filt(i-1))/2)*.01;
        vy_global_low_filt(i) = vy_global_low_filt(i-1) + ((ay_global_low_filt(i) + ay_global_low_filt(i-1))/2)*.01;
        %vx_global_band_filt(i) = vx_global_band_filt(i-1) + ((ax_global_band_filt(i) + ax_global_band_filt(i-1))/2)*.01;
        %vx_global_band_filt(i) = vx_global_band_filt(i-1) + ((ax_global_band_filt(i) + ax_global_band_filt(i-1))/2)*.01;
    end
end
% vel2 = 
% figure
% plot(t,vel,


fc = 10;                        %%%%%%initializing variables for bw2 lowpass filter
fs = 100;
gl2 = tan(pi*fc/fs);
dl2 = gl2*gl2+gl2*sqrt(2)+1;
b0l2 = (gl2*gl2)/dl2;
b1l2 = (2*gl2*gl2)/dl2;
b2l2 = (gl2*gl2)/dl2;
a1l2 = (2*(gl2*gl2-1))/dl2;
a2l2 = (gl2*gl2-gl2*sqrt(2)+1)/dl2;
% f_cut2 = .75;                    %%%%%%initializing variables for bw2 bandpass filter
% f_samp2 = 100;
% b_width2 =10.2;
% gb2 = tan(pi*f_cut2/f_samp2);
% db2 = (1+gb2*gb2)*f_cut2 + gb2*b_width2;
% b0b2 = b_width2*gb2/db2;
% b1b2 = 0;
% b2b2 = -b0b2;
% a1b2 = 2*f_cut2*((gb2*gb2)-1);
% a2b2 = (1+gb2*gb2)*f_cut2 - gb2*b_width2;
for i = 3500:4500
    if i >= 3500 && i < 3502
        vx_filt(i) = b0l2*vx_global_low_filt(i) + b1l2*vx_global_low_filt(i-1) + b2l2*vx_global_low_filt(i-2);     %%%filtering accel data
        vy_filt(i) = b0l2*vy_global_low_filt(i) + b1l2*vy_global_low_filt(i-1) + b2l2*vy_global_low_filt(i-2);
        %vx_filt_band(i) = b0b2*vx_global_band_filt(i) + b1b2*vx_global_band_filt(i-1) + b2b2*vx_global_band_filt(i-2);
        %vy_filt_band(i) = b0b2*vy_global_band_filt(i) + b1b2*vy_global_band_filt(i-1) + b2b2*vy_global_band_filt(i-2);
    
    %%% filtering global vel 
    else
        vx_filt(i) = b0l2*vx_global_low_filt(i) + b1l2*vx_global_low_filt(i-1) + b2l2*vx_global_low_filt(i-2) - a1l2*vx_filt(i-1) - a2l2*vx_filt(i-2);
        vy_filt(i) = b0l2*vy_global_low_filt(i) + b1l2*vy_global_low_filt(i-1) + b2l2*vy_global_low_filt(i-2) - a1l2*vy_filt(i-1) - a2l2*vy_filt(i-2);
        %vx_filt_band(i) = b0b2*vx_global_band_filt(i) + b1b2*vx_global_band_filt(i-1) + b2b2*vx_global_band_filt(i-2) - a1b2*vx_filt_band(i-1) - a2b2*vx_filt_band(i-2);
        %vy_filt_band(i) = b0b2*vy_global_band_filt(i) + b1b2*vy_global_band_filt(i-1) + b2b2*vy_global_band_filt(i-2) - a1b2*vy_filt_band(i-1) - a2b2*vy_filt_band(i-2);  
    end
end
 x_lf = 0; 
 y_lf = 0; 
 x_bf = 0; 
 y_bf = 0;
for i = 3500:4500
    if i == 3500
        x_lf(i) = x(i) + ((vx_filt(i) + vx_filt(i-1))/2)*.01; %%%integrating global vel to get global pos
        y_lf(i) = y(i) + ((vy_filt(i) + vy_filt(i-1))/2)*.01;
        %x_bf(i) = x(i) + ((vx_filt_band(i) + vx_filt_band(i-1))/2)*.01;
        %y_bf(i) = y(i) + ((vy_filt_band(i) + vy_filt_band(i-1))/2)*.01;
        ts = t(i);
    else
        x_lf(i) = x_lf(i-1) + ((vx_filt(i) + vx_filt(i-1))/2)*.01;
        y_lf(i) = y_lf(i-1) + ((vy_filt(i) + vy_filt(i-1))/2)*.01;
        %x_bf(i) = x_bf(i-1) + ((vx_filt_band(i) + vx_filt_band(i-1))/2)*.01;
        %y_bf(i) = y_bf(i-1) + ((vy_filt_band(i) + vy_filt_band(i-1))/2)*.01;
        ts = t(i);
    end
end
yaw_deg = yaw.*(180/pi); %%% calculating beta from global velocity and yaw
beta_est = atand(vy_filt./vx_filt);
for i = 3500:4500
    beta_est2(i) = beta_est(i) - yaw_deg(i);
end

figure
plot(t,beta,time,beta_est2,'r')
title('GPS beta vs estimated beta')
xlabel('time(s)')
ylabel('beta(deg)')
legend('GPS beta','estimated beta')
figure
plot(x,y,x_lf,y_lf,'xk')
title('GPS position vs estimated position')
xlabel('x position meters')
ylabel('y position meters')
legend('GPS position','estimated position')
