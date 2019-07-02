clear all
%[left_turn_CF, right_turn_CF] = SWA_CF('data_2.mat',650,3760,4200,6800);
%[swa_baseline, brake_baseline] = swa_baseline('data_1.mat');
%save CF_variable.mat
load('data_1.mat');
load('Turn_Correction_Factors.mat')
swa_baseline = 2.84 %%mean(SWA);
brake_baseline_front = mean(TBF);
save('SWA_Brake_Baseline.mat','swa_baseline','brake_baseline_front')
l = length(t);
vel = V.*(1000/3600);
distance = 0;
r = 0;
ax_actual = ax.*9.81; %%%%initializing variables and correcting for units
ay_actual = ay.*9.81;
a_cent = 0;
beta_radians = 0;
dist = 0;
fcl = 1;                %%%%intializing filtering variables for 2nd order low pass butterworth
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
b_rad_filt = 0;
beta_filt = 0;
acent_filt = 0;
rad = 0;
beta_rad = 0; 


    for i = 1:l
        swa_corrected(i) = SWA(i) - 2.84;       %%%%shifting the steering down to zero
        b_rad(i) = (beta(i)+1.2139)*(pi/180);   %%%converting to radians
        beta_radians(i) = beta(i)*(pi/180);
        a_cent(i) = ax_actual(i)*sin(b_rad(i)) + ay_actual(i)*cos(b_rad(i));
        r(i) = (vel(i)*vel(i))/a_cent(i);
        
            if swa_corrected(i) >= 0                            %%%%%%%applying steering correction factor to right and left turns
                turn_deg(i) = swa_corrected(i)*left_turn_CF*1;
            else
                turn_deg(i) = swa_corrected(i)*right_turn_CF*1;
            end
            
            if i == 1
                distance(i) = 0;                            %%%%%calculating distance traveled
            else
                distance(i) = distance(i-1) + ((vel(i) + vel(i-1))/2)*.01; 
            end
            
        if i < 5 && i >= 3
            ax_filt_actual(i) = b0l*ax_actual(i) + b1l*ax_actual(i-1) + b2l*ax_actual(i-2);     %%%%%this loop was used for examining baseline behavior of sensors
            ay_filt_actual(i) = b0l*ay_actual(i) + b1l*ay_actual(i-1) + b2l*ay_actual(i-2);
            beta_filt(i) = b0l*beta(i) + b1l*beta(i-1) + b2l*beta(i-2);
            b_rad_filt(i) = (beta_filt(i)+1.2026)*(pi/180);
            ay_filt_actual_offset(i) = ay_filt_actual(i) - .2114;
            a_cent_filt(i) = ax_filt_actual(i)*sin(b_rad_filt(i)) + ay_filt_actual(i)*cos(b_rad_filt(i));
            a_cent_filt_offset(i) = ax_filt_actual(i)*sin(b_rad_filt(i)) + ay_filt_actual_offset(i)*cos(b_rad_filt(i));
            rad(i) = (vel(i)*vel(i))/a_cent_filt(i);
            rad_offset(i) = (vel(i)*vel(i))/a_cent_filt_offset(i);
            time_accel(i) = t(i);
                if i == 3
                    dist(i) = 0;
                else
                    dist(i) = dist(i-1) + ((vel(i) + vel(i-1))/2)*.01; 
                end
                
        elseif i >= 5                   %%%%%this loop was used for examining baseline behavior of sensors not used in first part
            ax_filt_actual(i) = b0l*ax_actual(i) + b1l*ax_actual(i-1) + b2l*ax_actual(i-2) - a1l*ax_filt_actual(i-1) - a2l*ax_filt_actual(i-2);
            ay_filt_actual(i) = b0l*ay_actual(i) + b1l*ay_actual(i-1) + b2l*ay_actual(i-2) - a1l*ay_filt_actual(i-1) - a2l*ay_filt_actual(i-2);
            beta_filt(i) = b0l*beta(i) + b1l*beta(i-1) + b2l*beta(i-2) - a1l*beta_filt(i-1) - a2l*beta_filt(i-2);
            b_rad_filt(i) = (beta_filt(i)+1.2026)*(pi/180);
            ay_filt_actual_offset(i) = ay_filt_actual(i) - .2114;
            a_cent_filt(i) = ax_filt_actual(i)*sin(b_rad_filt(i)) + ay_filt_actual(i)*cos(b_rad_filt(i));
            a_cent_filt_offset(i) = ax_filt_actual(i)*sin(b_rad_filt(i)) + ay_filt_actual_offset(i)*cos(b_rad_filt(i));
            rad(i) = (vel(i)*vel(i))/a_cent_filt(i);
            rad_offset(i) = (vel(i)*vel(i))/a_cent_filt_offset(i);
            time_accel(i) = t(i);
            dist(i) = dist(i-1) + ((vel(i) + vel(i-1))/2)*.01; 
            
        end
    end
    


% figure
% subplot(4,1,1)
% plot(t,a_cent)
% title('Centripedal Acceleration Without Filtering')
% subplot(4,1,2)
% plot(t,r)
% title('Radius of Curvature Without Filtering')
% subplot(4,1,3)
% plot(t,beta)
% title('Beta Angle in Degrees No Offset No Filtering')
% subplot(4,1,4)
% plot(t,b_rad)
% title('Beta Angle in Radians offset to Zero No filtering')

%  figure
%  plot(time_accel,ax_filt_actual,t,ax)
%  legend('x filt','x')
%  figure
%  plot(time_accel,ay_filt_actual,t,ay)
%  legend('y filt','y')
%  figure
%  plot(time_accel,ay_filt_actual)
%  legend('offset y')
%  figure
%  plot(time_accel,ax_filt_actual,t,ax_actual)
%  legend('filt','unfilt')
% 
% beta_offset_filt = mean(beta_filt)
% 
% figure
% plot(time_accel,correct_x,time_accel,correct_y)
% title('correct x correct y')
% figure
% plot(time_accel,b_rad_filt,t,beta_radians)
% title('Filter Beta vs Unfiltered Beta')
% figure
% plot(time_accel,a_cent_filt)
% title('filt a cent')
% figure
% plot(time_accel,rad)
% title('filt')
% figure
% plot(time_accel,a_cent_correct, time_accel,correct_y,time_accel,b_rad_filt)
% legend('cent','y accel','b filt')
% title('a cent correct')
% figure
% plot(dist,rad_correct)
% title('rad correct')
% 
figure
plot(t,turn_deg)
title('steering wheel angle vs time-Straight')
xlabel('time(s)')
ylabel('steering wheel angle(deg)')
% map_turn = turn_deg.*(-1);
% 
% figure
% plot(map_turn,distance)


clear all
load('data_2.mat');             %%%%%clearing data and calling data from dataset 2
load('Turn_Correction_Factors.mat')
load('SWA_Brake_Baseline.mat')
l = length(t);                  %%%%intializing variables
vel = V.*(1000/3600);
distance = 0;

    for i = 1:l
        swa_corrected(i) = SWA(i) - swa_baseline;       %%%calculating steering wheel angle
        
            if swa_corrected(i) >= 0
                turn_deg(i) = swa_corrected(i)*left_turn_CF*1;
            else
                turn_deg(i) = swa_corrected(i)*right_turn_CF*1;
            end
            
            if i == 1
                distance(i) = 0;            %%%%calculating distance
            else
                distance(i) = distance(i-1) + ((vel(i) + vel(i-1))/2)*.01; 
            end
        
    end
figure
plot(t,turn_deg)                    %%%%%%%%plotting steering wheel angle
title('steering wheel angle vs time-Full lock')
xlabel('time(s)')
ylabel('steering wheel angle(deg)')

% map_turn = turn_deg.*(-1);

% figure
% plot(map_turn,distance)
% figure
% plot(t,vel)
% title('vel')
% plot(t,ay)
% title('y accel')
% figure
% plot(t,beta)
% title('beta')


clear all
load('CF_Variable.mat')             %%%%clearing old data and loading rally stage data
load('data_3.mat');
load('Turn_Correction_Factors.mat')
load('SWA_Brake_Baseline.mat')
l = length(t);
vel = V.*(1000/3600);
distance = 0;
brake_front_corrected = 0;          %%%%%initializing variables and correcting for units
brake_rear_corrected = 0;
e_brake = 0;
yaw = 0;
yaw_filt = 0;
time_filt = 0;
dist = 0;
fcl = 1;                        %%%%%%initializing variables for bw2 lowpass filter
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
fch = .1;                   %%%%%%initializing variables for bw2 highpass filter
fsh = 100;
gh = tan(pi*fch/fsh);
dh = gh*gh+gh*sqrt(2)+1;
b0h = 1/dh;
b1h = -2/dh;
b2h = 1/dh;
a1h = (2*(gh*gh-1))/dh;
a2h = (gh*gh-gh*sqrt(2)+1)/dh;

ax_actual = ax.*9.8;
ay_actual = ay.*9.8;
%  for i = 1:l
%      ay_actual(i) = ay_actual(i) + .6114;
%  end
a_cent = 0;
b_rad = beta.*(pi/180);
r = 0;
b_rad_filt = 0;
beta_filt = 0;
acent_filt = 0;
rad = 0;
front = 0;
rear = 0;

f_cut = .76;                    %%%%%%initializing variables for bw2 bandpass filter
f_samp = 100;
b_width =10.1;
gb = tan(pi*f_cut/f_samp);
db = (1+gb*gb)*f_cut + gb*b_width;
b0b = b_width*gb/db;
b1b = 0;
b2b = -b0b;
a1b = 2*f_cut*((gb*gb)-1);
a2b = (1+gb*gb)*f_cut - gb*b_width;




    for i = 1:l
        swa_corrected(i) = SWA(i) - swa_baseline;           %%%%%%%calculating steering wheel angle
        a_cent(i) = ax_actual(i)*sin(b_rad(i)) + ay_actual(i)*cos(b_rad(i)); %%%%calc unfiltered centripedal accel 
            
%             if a_cent(i) >= .1 || a_cent(i) <= -.1
%                 r(i) = (vel(i)*vel(i))/a_cent(i);
%                 k(i) = a_cent(i)/(vel(i)*vel(i));
%             else
%                 r(i) = 6;
%                 k(i) = a_cent(i)/(vel(i)*vel(i));
%             end
        
            if swa_corrected(i) >= 0
                turn_deg(i) = swa_corrected(i)*left_turn_CF*1;      %%%%%%%calculating steering wheel angle
            else
                turn_deg(i) = swa_corrected(i)*right_turn_CF*1;
            end
            
            if i == 1
                distance(i) = 0;                                    %%%%%%%calculating distance
            else
                distance(i) = distance(i-1) + ((vel(i) + vel(i-1))/2)*.01; 
            end
            
            brake_front_corrected(i) = TBF(i) - brake_baseline;         %%%%shifting the brake signal to zero
            brake_rear_corrected(i) = TBR(i) - brake_baseline;
            
            if (brake_rear_corrected(i)/brake_front_corrected(i)) > 3 && brake_rear_corrected(i) > .025
                e_brake(i) = 1;
                rear(i) = TBR(i);           %%%%if the rear brake is on and the front brake is off then its the e brake
            else
                e_brake(i) = 0;
                rear(i) = 0;
            end
            
            if (e_brake(i) == 0) && brake_front_corrected(i) > .025
                all_brake(i) = 1;
                front(i) = TBF(i);          %%%%%%%if both brakes are on then is the brake pedal
            else
                all_brake(i) = 0;
                front(i) = 0;
            end
            
            if i >=2
                yaw(i) = yaw(i-1) + ((psidot(i) + psidot(i-1))/2)*.01;  %%%%integrating psidot to get yaw
                dist(i) = dist(i-1) + ((vel(i) + vel(i-1))/2)*.01;         %%%%%calc distance
                time(i) = t(i);      
            end 
            
            if i < 5 && i >= 3
                yaw_filt(i) = b0h*yaw(i) + b1h*yaw(i-1) + b2h*yaw(i-2);     %%%filtering yaw data
                
                ax_filt_actual(i) = b0l*ax_actual(i) + b1l*ax_actual(i-1) + b2l*ax_actual(i-2);     %%%filtering accel data
                ay_filt_actual(i) = b0l*ay_actual(i) + b1l*ay_actual(i-1) + b2l*ay_actual(i-2);
                x_accel_filt(i) = b0b*ax_actual(i) + b1b*ax_actual(i-1) + b2b*ax_actual(i-2);
                y_accel_filt(i) = b0b*ay_actual(i) + b1b*ay_actual(i-1) + b2b*ay_actual(i-2);
                ax_filt_high_pass(i) = b0h*ax_actual(i) + b1h*ax_actual(i-1) + b2h*ax_actual(i-2);
                time_filt(i) = t(i);
                beta_filt(i) = b0l*beta(i) + b1l*beta(i-1) + b2l*beta(i-2);             %%%filtering beta data
                b_rad_filt(i) = (beta_filt(i)+1.2026)*(pi/180);
                ay_filt_actual_offset(i) = ay_filt_actual(i) - .2114;
                a_cent_filt(i) = ax_filt_actual(i)*sin(b_rad_filt(i)) + ay_filt_actual(i)*cos(b_rad_filt(i));       %%%calc centripedal accel
                a_cent_filt_offset(i) = (ax_filt_actual(i)*sin(b_rad_filt(i))) + (ay_filt_actual_offset(i)*cos(b_rad_filt(i)));
                rad(i) = (vel(i)*vel(i))/a_cent_filt(i);                    %%%calc radius of curvature
                rad_offset(i) = (vel(i)*vel(i))/a_cent_filt_offset(i);
                time_accel(i) = t(i);
                    if rad(i) >= 250
                        rad(i) = 250;
                    elseif rad(i) <= -250
                        rad(i) = -250;
                    end
                    if i == 3
                        dist(i) = 0;
                    else
                        dist(i) = dist(i-1) + ((vel(i) + vel(i-1))/2)*.01; 
                    end
                    
            elseif i >= 5          %%%%%%same as loop before just need delay to properly integrate for filtering
                yaw_filt(i) = b0h*yaw(i) + b1h*yaw(i-1) + b2h*yaw(i-2) - a1h*yaw_filt(i-1) - a2h*yaw_filt(i-2);
                ax_filt_actual(i) = b0l*ax_actual(i) + b1l*ax_actual(i-1) + b2l*ax_actual(i-2) - a1l*ax_filt_actual(i-1) - a2l*ax_filt_actual(i-2);
                ay_filt_actual(i) = b0l*ay_actual(i) + b1l*ay_actual(i-1) + b2l*ay_actual(i-2) - a1l*ay_filt_actual(i-1) - a2l*ay_filt_actual(i-2);
                x_accel_filt(i) = b0b*ax_actual(i) + b1b*ax_actual(i-1) + b2b*ax_actual(i-2) - a1b*x_accel_filt(i-1) - a2b*x_accel_filt(i-2);
                y_accel_filt(i) = b0b*ay_actual(i) + b1b*ay_actual(i-1) + b2b*ay_actual(i-2) - a1b*y_accel_filt(i-1) - a2b*y_accel_filt(i-2);
                ax_filt_high_pass(i) = b0h*ax_actual(i) + b1h*ax_actual(i-1) + b2h*ax_actual(i-2) - a1h*ax_filt_high_pass(i-1) - a2h*ax_filt_high_pass(i-2);
                time_filt(i) = t(i);
                beta_filt(i) = b0l*beta(i) + b1l*beta(i-1) + b2l*beta(i-2) - a1l*beta_filt(i-1) - a2l*beta_filt(i-2);
                b_rad_filt(i) = (beta_filt(i)+1.2026)*(pi/180);
                ay_filt_actual_offset(i) = ay_filt_actual(i) - .2114;
                a_cent_filt(i) = (ax_filt_actual(i)*sin(b_rad_filt(i))) - (ay_filt_actual(i)*cos(b_rad_filt(i)));
                a_cent_filt_offset(i) = ax_filt_actual(i)*sin(b_rad_filt(i)) + ay_filt_actual_offset(i)*cos(b_rad_filt(i));
                rad(i) = (vel(i)*vel(i))/a_cent_filt(i);
                rad_offset(i) = (vel(i)*vel(i))/a_cent_filt_offset(i);
                time_accel(i) = t(i);
                dist(i) = dist(i-1) + ((vel(i) + vel(i-1))/2)*.01; 
                    if rad(i) >= 250
                        rad(i) = 250;
                    elseif rad(i) <= -250
                        rad(i) = 250;
                    end
            end
    end
    

    
    for i = 2:l                                 %%%%%limiting yaw to only 360 degrees
        if yaw(i) <= -360 && yaw(i) > -720
            yaw(i) = yaw(i) + 360;
        elseif yaw(i) >= 360 && yaw(i) < 720
            yaw(i) = yaw(i) - 360;
        elseif yaw(i) <= -720
            yaw(i) = yaw(i) + 720;
        elseif yaw(i) >= 720
            yaw(i) = yaw(i) - 720;
        end
    end
    yawd = cumtrapz(t,psidot);          %%%%%quickly integrating psidot without filtering
    yawr = yawd.*(pi/180)               
    yaw_rad = yaw.*(pi/180);
    accel_x_actual = ax_actual.*cos(yawr) - ay_actual.*sin(yawr);       %%%%shifting the accel data ffrom bodyframe to gloabl frame
    accel_y_actual = ax_actual.*sin(yawr) - ay_actual.*cos(yawr);
%     for i = 3500:4500
%         if i == 3500
%             vx_global(i) = 12 + (accel_x_actual(i) + accel_x_actual(i-1))/2*.01;
%             vy_global(i) = (accel_y_actual(i) + accel_y_actual(i-1))/2*.01;
%             beta_s(i) = beta(i);
%             yaw_s(i) = yaw_rad(i);
%             t_s(i) = t(i);
%         end
%             vx_global(i) = vx_global(i-1) + (accel_x_actual(i) + accel_x_actual(i-1))/2*.01;
%             vy_global(i) = vy_global(i-1) + (accel_y_actual(i) + accel_y_actual(i-1))/2*.01;
%             beta_s(i) = beta(i);
%             yaw_s(i) = yaw_rad(i);
%             t_s(i) = t(i);
%     end
    
    vx_global = cumtrapz(time_accel,accel_x_actual);        %%%%%quickly inegrating global accel data
    vy_global = cumtrapz(time_accel,accel_y_actual);
    length(vx_global)
    length(yaw_rad)
    vx_body = vx_global.*cos(yawr) + vy_global.*sin(yawr);      %%%calc global vel data
    vy_body = -vx_global.*sin(yawr) + vy_global.*cos(yawr);
    
    beta_est = atan2(vy_body,vx_body);                          %%%%%calc beta estimate
    vel_est = sqrt(vx_body.*vx_body + vy_body.*vy_body);
    %vel_est = vx_body.*cos(beta_est) + vy_body.*sin(beta_est);

%     figure
%     plot(t,vel,t(3500:4500),vel_est(3500:4500));
%     title('vel v est')
%     figure
%     plot(x,y,'*',xx2(3500:4500),yy2(3500:4500),'r')
%     title('pos')
 %max_cent = abs(min(a_cent_filt));
%[max_cent,loc] = max(a_cent_filt)
%vel(loc)
for i = 6166:11210
    cent_accel(i-6165) = a_cent_filt(i);            %%%%just getting cent data from a single lap
    cent_accel_reverse(i-6165) = a_cent_filt(i)*-1; %%%% inverting data so findpeaks function works for the negative values
    time_cent(i-6165) = t(i) - 61.65;
    vel_cent(i-6165) = vel(i);
end
max_cent = max(cent_accel);
k = 1./rad;
%[pks,loc] = findpeaks(cent_accel,'MinPeakDistance',100,'MinPeakProminence',1) 
[pks,loc] = findpeaks(cent_accel,'MinPeakDistance',250,'MinPeakHeight',5.01) %%%%finding peak accel for each turn and location in data
time_loc = loc./100;        %%%covereting data to seconds
[pek,lox] = findpeaks(cent_accel_reverse,'MinPeakDistance',200,'MinPeakHeight',2)
time_lox = lox./100;
loc_l = length(loc);
lox_l = length(lox);
% max_vel = zeros(size(time_accel));
% max_vel2 = zeros(size(time_accel));
% diff_vel = zeros(size(time_accel));
% diff_vel2 = zeros(size(time_accel));

max_vel2 = 0;
vel_check2 = 0;
for i = 1:loc_l
    radius(i) = vel_cent(loc(i))*vel_cent(loc(i))/pks(i);   %%%estimating radius of each corner
    max_vel(i) = sqrt(radius(i)*max_cent);                  %%%%finding max vel of each corner
    diff_vel(i) = max_vel(i) - vel_cent(loc(i));            %%%comparing max velocity with actual velocity
    vel_check(i) = vel_cent(loc(i));                        
end

for i = 1:lox_l
    radius2(i) = vel_cent(lox(i))*vel_cent(lox(i))/pek(i);  %%%same as other loop
    max_vel2(i) = sqrt(radius2(i)*max_cent);
    diff_vel2(i) = max_vel2(i) - vel_cent(lox(i));
    vel_check2(i) = vel_cent(lox(i));
end

loc = loc./100;
lox = lox./100;
% figure
% plot(loc,max_vel,'rx',lox,max_vel2,'rx',time_cent,vel_cent,loc,vel_check,'xk',lox,vel_check2,'ok')
% legend('max vel')
% figure
% plot(loc,diff_vel,'kx',lox,diff_vel2,'rx')
% legend('left turn','right turn')
% title('difference in max velocity during corner and actual velocity')
% xlabel('time(s)')
% ylabel('difference in velocity(m/s)')
% figure
% plot(time_accel,vel)
% legend('velocity')


for i = 2:4500
    if i <3500
        if i == 2
            y_velocity(i) = ((ay_filt_actual(i) + ay_filt_actual(i-1))/2)*.01;%%%%integrating to get y velocity from accel
            ty(i) = t(i);
        else
            y_velocity(i) = y_velocity(i-1) + ((ay_filt_actual(i) + ay_filt_actual(i-1))/2)*.01;
            ty(i) = t(i);
        end
    
    elseif i <= 3502 && i >= 3500
        velocity(i) = vel(i-3) + ((ax_filt_actual(i-3) + 3*ax_filt_actual(i-2) + 3*ax_filt_actual(i-1) + ax_filt_actual(i))/8)*.03;%%%%%simpsons 3/8 rule to integrate x accel into vel
        %y_velocity(i) = y_velocity(i-3) + ((ay_filt_actual(i-3) + 3*ay_filt_actual(i-2) + 3*ay_filt_actual(i-1) + ay_filt_actual(i))/8)*.03;
        y_velocity(i) = y_velocity(i-1) + ((ay_filt_actual(i) + ay_filt_actual(i-1))/2)*.01;    %%%%integrating to get  velocity
        t_loss(i) = t(i);   %%%%getting time array correct
        vel_band(i) = vel(i-3) + ((x_accel_filt(i-3) + 3*(x_accel_filt(i-2) + x_accel_filt(i-1)) + x_accel_filt(i))/8)*.03;%%%integrating bandpass filtered accel data
        y_vel_band(i) = y_velocity(i-3) + ((y_accel_filt(i-3) + 3*(y_accel_filt(i-2) + y_accel_filt(i-1)) + y_accel_filt(i))/8)*.03;
        vel_high(i) = vel(i-3) + ((ax_filt_high_pass(i-3) + 3*(ax_filt_high_pass(i-2) + ax_filt_high_pass(i-1)) + ax_filt_high_pass(i))/8)*.03;%%%x acel high pass filter to vel
        vel_ave(i) = (velocity(i) + vel_high(i))/2; %%%averaging high and loss pass filters
        band_ave(i) = (velocity(i) + vel_band(i))/2;    %%%%averagin low and band pass filtering
        y_band_ave(i) = (y_velocity(i) + y_vel_band(i))/2;
        ty(i) = t(i);
            if i == 3500
                x_v(i) = vel(i-1) + (ax_actual(i) + ax_actual(i-1))/2*.01;      %%%%%another method to inegrate to get velocity and then data
                y_v(i) = y_velocity(i-1) + (ay_actual(i) + ay_actual(i-1))/2*.01;%%%%%ultimately this didnt work
            else
                x_v(i) = x_v(i-1) + (ax_actual(i) + ax_actual(i-1))/2*.01;
                y_v(i) = y_v(i-1) + (ay_actual(i) + ay_actual(i-1))/2*.01;
            end
            if i == 3501
                x_pos(i) = x(i-1) + (band_ave(i) + band_ave(i-1))/2*.01;
                y_pos(i) = y(i-1) + (y_band_ave(i) + y_band_ave(i-1))/2*.01;
                x_p(i) = x(i-1) + (x_v(i) + x_v(i-1))/2*.01;
                y_p(i) = y(i-1) + (y_v(i) + y_v(i-1))/2*.01;
            elseif i > 3501
                x_pos(i) = x_pos(i-1) + (band_ave(i) + band_ave(i-1))/2*.01;
                y_pos(i) = y_pos(i-1) + (y_band_ave(i) + y_band_ave(i-1))/2*.01;
                x_p(i) = x_p(i-1) + (x_v(i) + x_v(i-1))/2*.01;
                y_p(i) = y_p(i-1) + (y_v(i) + y_v(i-1))/2*.01;
            end
    %%%this is the same as the other loop just need a delay for simpsons rule and filtering to work properly    
    else
        velocity(i) = velocity(i-3) + ((ax_filt_actual(i-3) + 3*ax_filt_actual(i-2) + 3*ax_filt_actual(i-1) + ax_actual(i))/8)*.03;
        %y_velocity(i) = y_velocity(i-3) + ((ay_filt_actual(i-3) + 3*ay_filt_actual(i-2) + 3*ay_filt_actual(i-1) + ay_filt_actual(i))/8)*.03;
        y_velocity(i) = y_velocity(i-1) + ((ay_filt_actual(i) + ay_filt_actual(i-1))/2)*.01;
        t_loss(i) = t(i);
        vel_band(i) = vel_band(i-3) + ((x_accel_filt(i-3) + 3*(x_accel_filt(i-2) + x_accel_filt(i-1)) + x_accel_filt(i))/8)*.03;
        y_vel_band(i) = y_velocity(i-3) + ((y_accel_filt(i-3) + 3*(y_accel_filt(i-2) + y_accel_filt(i-1)) + y_accel_filt(i))/8)*.03;
        vel_high(i) = vel_high(i-3) + ((ax_filt_high_pass(i-3) + 3*(ax_filt_high_pass(i-2) + ax_filt_high_pass(i-1)) + ax_filt_high_pass(i))/8)*.03;
        vel_ave(i) = (velocity(i) + vel_high(i))/2;
        band_ave(i) = (velocity(i) + vel_band(i))/2;
        y_band_ave(i) = (y_velocity(i) + y_vel_band(i))/2;
        x_pos(i) = x_pos(i-1) + (band_ave(i) + band_ave(i-1))/2*.01;
        y_pos(i) = y_pos(i-1) + (y_band_ave(i) + y_band_ave(i-1))/2*.01;
        x_v(i) = x_v(i-1) + (ax_actual(i) + ax_actual(i-1))/2*.01;
        y_v(i) = y_v(i-1) + (ay_actual(i) + ay_actual(i-1))/2*.01;
        x_p(i) = x_p(i-1) + (x_v(i) + x_v(i-1))/2*.01;
        y_p(i) = y_p(i-1) + (y_v(i) + y_v(i-1))/2*.01;
        ty(i) = t(i);
%             if i <= 3505
%                 x_pos(i) = x(i-3) + ((band_ave(i-3) + 3*(band_ave(i-2) + band_ave(i-1)) + band_ave(i))/8)*.03;
%                 y_pos(i) = y(i-3) + ((y_band_ave(i-3) + 3*(y_band_ave(i-2) + y_band_ave(i-1)) + y_band_ave(i))/8)*.03;
%             else
%                 x_pos(i) = x_pos(i-3) + ((band_ave(i-3) + 3*(band_ave(i-2) + band_ave(i-1)) + band_ave(i))/8)*.03;
%                 y_pos(i) = y_pos(i-3) + ((y_band_ave(i-3) + 3*(y_band_ave(i-2) + y_band_ave(i-1)) + y_band_ave(i))/8)*.03;
%             end
    end
end
% figure
% plot(t,ay)
% title('y accel')
% figure
% plot(ty,y_velocity)
% title('y velo')
fcl_pos = .01;      %%%%position filter variables
fsl_pos = 100;
gl_pos = tan(pi*fcl_pos/fsl_pos);
dl_pos = gl_pos*gl_pos+gl_pos*sqrt(2)+1;
b0l_pos = 1/dl_pos;
b1l_pos = -2/dl_pos;
b2l_pos = 1/dl_pos;
a1l_pos = (2*(gl_pos*gl_pos-1))/dl_pos;
a2l_pos = (gl_pos*gl_pos-gl_pos*sqrt(2)+1)/dl_pos;
% fch = .1;
% fsh = 100;
% gh = tan(pi*fch/fsh);
% dh = gh*gh+gh*sqrt(2)+1;
% b0h = 1/dh;  (gl_pos*gl_pos)
% b1h = -2/dh;  (2*gl_pos*gl_pos)
% b2h = 1/dh;   (gl_pos*gl_pos)/dl_pos;
% a1h = (2*(gh*gh-1))/dh;
% a2h = (gh*gh-gh*sqrt(2)+1)/dh;
for i = 3503:4500
    if i <= 5305        %%%%trying to filter postion data to correctly match gps
        x_vel_filt(i) = b0l_pos*band_ave(i) + b1l_pos*band_ave(i-1) + b2l_pos*band_ave(i-2);
        y_vel_filt(i) = b0l_pos*y_band_ave(i) + b1l_pos*y_band_ave(i-1) + b2l_pos*y_band_ave(i-2);
    else
        x_vel_filt(i) = b0l_pos*band_ave(i) + b1l_pos*band_ave(i-1) + b2l_pos*band_ave(i-2) - a1l_pos*x_vel_filt(i-1) - a2l_pos*x_vel_filt(i-2);
        y_vel_filt(i) = b0l_pos*y_band_ave(i) + b1l_pos*y_band_ave(i-1) + b2l_pos*y_band_ave(i-2) - a1l_pos*y_vel_filt(i-1) - a2l_pos*y_vel_filt(i-2); 
    end
end

for i = 3503:4500
    if i <= 3503    %%more filtering
        x_pos_filt(i) = x(i-1) + (x_vel_filt(i) + x_vel_filt(i-1))/2*.01;
        y_pos_filt(i) = y(i-1) + (y_vel_filt(i) + y_vel_filt(i-1))/2*.01;
        %x_pos_filt(i) = x(i-3) + ((x_vel_filt(i-3) + 3*(x_vel_filt(i-2) + x_vel_filt(i-1)) + x_vel_filt(i))/8)*.03;
        %y_pos_filt(i) = y(i-3) + ((y_vel_filt(i-3) + 3*(y_vel_filt(i-2) + y_vel_filt(i-1)) + y_vel_filt(i))/8)*.03;
    else
        x_pos_filt(i) = x_pos_filt(i-1) + (x_vel_filt(i) + x_vel_filt(i-1))/2*.01;
        y_pos_filt(i) = y_pos_filt(i-1) + (y_vel_filt(i) + y_vel_filt(i-1))/2*.01;
        %x_pos_filt(i) = x_pos_filt(i-3) + ((x_vel_filt(i-3) + 3*(x_vel_filt(i-2) + x_vel_filt(i-1)) + x_vel_filt(i))/8)*.03;
        %y_pos_filt(i) = y_pos_filt(i-3) + ((y_vel_filt(i-3) + 3*(y_vel_filt(i-2) + y_vel_filt(i-1)) + y_vel_filt(i))/8)*.03;
    end
        
end
%%%
% figure
% plot(t,vel,t_loss,velocity,'x',t_loss,vel_band,'ok',t_loss,band_ave,'r')
% title('Calculated velocity vs GPS velocity')
% xlabel('time(s)')
% ylabel('velocity(m/s)')
% legend('GPS velocity','fused filter velocity')
% figure
% plot(t,vel,t_loss,band_ave,'r')
% title('Calculated velocity vs GPS velocity')
% xlabel('time(s)')
% ylabel('velocity(m/s)')
% legend('GPS velocity','fused filter velocity')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% this is all just plotting%%%%%%%%%%%%
figure
plot(time_accel,b_rad_filt,t(3500:4500),beta_est(3500:4500))
title('Calculated beta vs GPS beta')
xlabel('time(s)')
ylabel('beta(rad)')
legend('GPS beta','calculated beta')

figure
plot(x,y,x_pos,y_pos,'xk',x_pos_filt,y_pos_filt,'or',x_p,y_p,'*c')
title('GPS position vs calculated position')
xlabel('x position')
ylabel('y position')
legend('GPS position','calculated position','filtered calculated position','Esitmated GPS position')


figure
plot(time_cent,cent_accel,time_loc,pks,'x')
legend('one lap accel')
figure
plot(time_cent,cent_accel_reverse,time_lox,pek,'x')
figure
plot(t,turn_deg)
title('steering wheel angle vs time')
xlabel('time')
ylabel('steering wheel angle(deg)')

figure
plot(t,front,t,rear)
legend('front','rear')
title('front and hand brake during rally stage')
xlabel('time(s)')
ylabel('brake pressure(V)')
% figure
% plot(t,brake_front_corrected)
% figure
% plot(t,brake_rear_corrected)
% figure
% plot(t,e_brake)
brake_baseline
brake_rear_percentage = mean(e_brake)
all_brake_percentage = mean(all_brake)
total_brake_percentage = brake_rear_percentage + all_brake_percentage

figure 
subplot(2,1,1)
plot(time,yaw,'.')
title('yaw vs time')
xlabel('time(s)')
ylabel('yaw angle(deg)')
subplot(2,1,2)
plot(time,dist)
title('distance vs time')
xlabel('time(s)')
ylabel('distance(m)')
figure
plot(t,ax*9.81,'c',time_filt,ax_filt_actual,'k')
title('X acceleration filtered vs unfiltered')
xlabel('time(s)')
ylabel('x accerleration(m/s^2)')
legend('filtered','unfiltered')
figure
plot(t,ay*9.81,'c',time_filt,ay_filt_actual,'k')
title('y acceleration filtered vs unfiltered')
xlabel('time(s)')
ylabel('y acceleration m/s^2)')
legend('filtered','unfiltered')

beta_offset_filt = mean(beta_filt)
figure
plot(dist,k)
title('path curvature vs distance')
xlabel('distance(m)')
ylabel('path curvature')

figure
plot(loc,max_vel,'rx',lox,max_vel2,'rx',time_cent,vel_cent,loc,vel_check,'xk',lox,vel_check2,'ok')
legend('max vel')
figure
plot(loc,diff_vel,'kx',lox,diff_vel2,'rx')
legend('left turn','right turn')
title('difference in max velocity during corner and actual velocity')
xlabel('time(s)')
ylabel('difference in velocity(m/s)')






