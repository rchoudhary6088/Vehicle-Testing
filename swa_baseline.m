function[swa_baseline, brake_baseline_front] = swa_baseline(filename)
load(filename);
swa_baseline = mean(SWA);
brake_baseline_front = mean(TBF);
end
