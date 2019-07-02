%%%individual Dan McCullough
clear all
load('CF_Variable.mat')
load('data_3.mat');
load('Turn_Correction_Factors.mat')
load('SWA_Brake_Baseline.mat')

l = length(t);
vel = V.*(1000/3600);
distance = 0;
brake_front_corrected = 0;
brake_rear_corrected = 0;
e_brake = 0;
yaw = 0;
yaw_filt = 0;
time_filt = 0;
dist = 0;
fcl = 2;
fsl = 100;
gl = tan(pi*fcl/fsl);
dl = gl*gl+gl*sqrt(2)+1;
b0l = (gl*gl)/dl;
b1l = (2*gl*gl)/dl;
b2l = (gl*gl)/dl;
a1l = (2*(gl*gl-1))/dl;
a2l = (gl*gl-gl*sqrt(2)+1)/dl;
ax_filt_actual = 0;
ay_filt_actual = 0;
time_accel = 0;
ax_actual = ax.*9.8;
ay_actual = ay.*9.8;

for i = 9600:10500
    omegaFLz(i) = omegaFL(i);
    omegaFRz(i) = omegaFR(i);
    omegaRLz(i) = omegaRL(i);
    omegaRRz(i) = omegaRR(i);
    velz(i) = vel(i);
    throttlez(i) = throttle(i);
    swa_corrected(i) = SWA(i) - swa_baseline;
    brake_front_corrected(i) = TBF(i) - brake_baseline;
    brake_rear_corrected(i) = TBR(i) - brake_baseline;
            
        if (brake_rear_corrected(i)/brake_front_corrected(i)) > 3 && brake_rear_corrected(i) > .025
            e_brake(i) = 1;
            rear(i) = brake_rear_corrected(i);
        else
            e_brake(i) = 0;
            rear(i) = 0;
        end
            
        if (e_brake(i) == 0) && brake_front_corrected(i) > .025
            all_brake(i) = 1;
            front(i) = brake_front_corrected(i);
        else
            all_brake(i) = 0;
            front(i) = 0;
        end
    if i < 9602
    ax_filt_actual(i) = b0l*ax_actual(i) + b1l*ax_actual(i-1) + b2l*ax_actual(i-2);
    ay_filt_actual(i) = b0l*ay_actual(i) + b1l*ay_actual(i-1) + b2l*ay_actual(i-2);
    beta_filt(i) = b0l*beta(i) + b1l*beta(i-1) + b2l*beta(i-2);
    b_rad_filt(i) = (beta_filt(i)+1.2026)*(pi/180);
    a_cent_filt(i) = ax_filt_actual(i)*sin(b_rad_filt(i)) + ay_filt_actual(i)*cos(b_rad_filt(i));
    end        
    if swa_corrected(i) >= 0
        turn_deg(i) = swa_corrected(i)*left_turn_CF*1;
    else
        turn_deg(i) = swa_corrected(i)*right_turn_CF*1;
    end
    if i == 9600
        distance(i) = 0;
        yaw(i) = 0;
    else
        distance(i) = distance(i-1) + ((vel(i) + vel(i-1))/2)*.01; 
        yaw(i) = yaw(i-1) + ((psidot(i) + psidot(i-1))/2)*.01; 
    end
        
        if i >= 9602
            ax_filt_actual(i) = b0l*ax_actual(i) + b1l*ax_actual(i-1) + b2l*ax_actual(i-2) - a1l*ax_filt_actual(i-1) - a2l*ax_filt_actual(i-2);
            ay_filt_actual(i) = b0l*ay_actual(i) + b1l*ay_actual(i-1) + b2l*ay_actual(i-2) - a1l*ay_filt_actual(i-1) - a2l*ay_filt_actual(i-2);
            beta_filt(i) = b0l*beta(i) + b1l*beta(i-1) + b2l*beta(i-2) - a1l*beta_filt(i-1) - a2l*beta_filt(i-2);
            b_rad_filt(i) = (beta_filt(i)+1.2026)*(pi/180);
            a_cent_filt(i) = ax_filt_actual(i)*sin(b_rad_filt(i)) + ay_filt_actual(i)*cos(b_rad_filt(i));
        end

  
end
length(distance)
length(front)
figure
subplot(4,1,1)
plot(distance,velz,distance,ay_filt_actual,distance,ax_filt_actual)
legend('velocity','y accel','x accel')
xlabel('distance (m)')
ylabel('speed, acceleration (m/s, m/s^2)')
title('velocity and acceleration vs distance')
subplot(4,1,2)
plot(distance,throttlez)
legend('throttle')
xlabel('distance (m)')
ylabel('percent throttle')
title('throttle vs distance')
subplot(4,1,3)
plot(distance,front,distance,rear)
legend('front brake','rear brake')
xlabel('distance (m)')
ylabel('brake pressure V')
title('brake pressure vs distance')
subplot(4,1,4)
plot(distance,omegaFLz,distance,omegaFRz,distance,omegaRLz,distance,omegaRRz)
legend('front left','front right','rear left','rear right')
xlabel('distance (m)')
ylabel('wheel speed (rad/s)')
title('wheel speed vs distance')

figure
subplot(3,1,1)
plot(distance,yaw)
legend('yaw')
xlabel('distance(m)')
ylabel('yaw(deg)')
title('yaw angle vs distance')
subplot(3,1,2)
plot(distance,b_rad_filt)
legend('beta')
xlabel('distance')
ylabel('beta(rad)')
title('beta vs distance')
subplot(3,1,3)
plot(distance,turn_deg)
legend('steering wheel angle')
xlabel('distance')
ylabel('steering wheel angle(deg)')
title('steering wheel angle vs distance')


