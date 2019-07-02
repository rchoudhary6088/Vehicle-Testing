function[left_turn_CF,right_turn_CF] = SWA_CF(filename, left_turn_LL, left_turn_UL, right_turn_LL, right_turn_UL)

load(filename);


vel = V.*(1000/3600);
ang = psidot.*(pi/180);
l = length(t);
swa_corrected = 0;
rad_total_left = 0;
swa_corrected_left = 0;
swa_corrected_right = 0;
rad_total_right = 0;

    for i = 1:l
        swa_corrected(i) = SWA(i) - swa_baseline('data_1.mat');
        rad(i) = vel(i)/ang(i);
    end
    
    for i = left_turn_LL:left_turn_UL
        rad_total_left = rad_total_left + rad(i);
        swa_corrected_left = swa_corrected_left + swa_corrected(i);
    end


    for i = right_turn_LL:right_turn_UL
         rad_total_right = rad_total_right + rad(i);
         swa_corrected_right = swa_corrected_right + swa_corrected(i);
    end
   
swa_corrected_left_average = swa_corrected_left/(left_turn_UL - left_turn_LL);
swa_corrected_right_average = swa_corrected_right/(right_turn_UL - right_turn_LL);


left_turn_deg = atan(2.486/(rad_total_left/(left_turn_UL-left_turn_LL)))*(180/pi)
right_turn_deg = atan(2.486/(rad_total_right/(right_turn_UL-right_turn_LL)))*(180/pi)
    
left_turn_CF = left_turn_deg / swa_corrected_left_average
right_turn_CF = right_turn_deg / swa_corrected_right_average
save('Turn_Correction_Factors.mat','left_turn_CF','right_turn_CF')
end