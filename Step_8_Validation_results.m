%Created By Abdelrahman Elzemrany
%Updated for 6 DoF System
Beta2=theta;
clc
k=48;        % Number of base parameters (update this if your 6 DoF regressor has more parameters)
time = 10;   % Experiment time
T = 0.01;   % Sampling period
l = time/T;  % Number of samples (10000)

% --- Data Extraction for 6 Joints ---
q11=pos1.signals(1).values(1:l);  % Joint positions
q22=pos1.signals(2).values(1:l);   
q33=pos1.signals(3).values(1:l);  
q44=pos1.signals(4).values(1:l);  
q55=pos1.signals(5).values(1:l);  
q66=pos1.signals(6).values(1:l);  

qp11=velocity1.signals(1).values(1:l);  % Joint velocities
qp22=velocity1.signals(2).values(1:l);  
qp33=velocity1.signals(3).values(1:l);  
qp44=velocity1.signals(4).values(1:l);  
qp55=velocity1.signals(5).values(1:l);  
qp66=velocity1.signals(6).values(1:l);  

qpp11=accelration1.signals(1).values(1:l);   % Joint accelerations
qpp22=accelration1.signals(2).values(1:l);  
qpp33=accelration1.signals(3).values(1:l); 
qpp44=accelration1.signals(4).values(1:l); 
qpp55=accelration1.signals(5).values(1:l); 
qpp66=accelration1.signals(6).values(1:l); 

u11=torque1.signals(1).values(1:l);   % Torques/Voltages
u22=torque1.signals(2).values(1:l); 
u33=torque1.signals(3).values(1:l); 
u44=torque1.signals(4).values(1:l); 
u55=torque1.signals(5).values(1:l); 
u66=torque1.signals(6).values(1:l); 

tau11 = u11; 
tau22 = u22; 
tau33 = u33*1;
tau44 = u44;
tau55 = u55;
tau66 = u66;

t=pos1.time(1:l,1);

% --- Regressor Matrix Generation (6 DoF) ---
% Note: Ensure your 'Y_b_handle' function is updated to accept 6 DoF inputs
for i=1:l
   Yb2(:,:,i)=Y_b_handle(0,0,-9.8, ...
       q11(i),q22(i),q33(i),q44(i),q55(i),q66(i), ...
       qp11(i),qp22(i),qp33(i),qp44(i),qp55(i),qp66(i), ...
       qpp11(i),qpp22(i),qpp33(i),qpp44(i),qpp55(i),qpp66(i));
end

sum11 = zeros(l,k); sum22 = zeros(l,k); sum33 = zeros(l,k);
sum44 = zeros(l,k); sum55 = zeros(l,k); sum66 = zeros(l,k);

for ii=1:k
    for ik=1:l
        sum11(ik,ii)=Yb2(1,ii,ik);
        sum22(ik,ii)=Yb2(2,ii,ik);
        sum33(ik,ii)=Yb2(3,ii,ik);
        sum44(ik,ii)=Yb2(4,ii,ik);
        sum55(ik,ii)=Yb2(5,ii,ik);
        sum66(ik,ii)=Yb2(6,ii,ik);
    end
end

Yc2=[sum11; sum22; sum33; sum44; sum55; sum66];
tv_1=Yc2*theta;

% --- Splitting Estimated Torques ---
tv11=tv_1(1:l,1);
tv22=tv_1(l+1:2*l,1);
tv33=tv_1(2*l+1:3*l,1);
tv44=tv_1(3*l+1:4*l,1);
tv55=tv_1(4*l+1:5*l,1);
tv66=tv_1(5*l+1:6*l,1);

% --- Timeseries Conversion ---
tv11t=timeseries(tv11); tv22t=timeseries(tv22); tv33t=timeseries(tv33);
tv44t=timeseries(tv44); tv55t=timeseries(tv55); tv66t=timeseries(tv66);
tau11t=timeseries(tau11); tau22t=timeseries(tau22); tau33t=timeseries(tau33);
tau44t=timeseries(tau44); tau55t=timeseries(tau55); tau66t=timeseries(tau66);

%% ==================== METRIC CALCULATION FOR EXP 2 ====================
% Combine all 6 validation measurements and estimations
tt2 = [tau11; tau22; tau33; tau44; tau55; tau66];
tv2_all = [tv11; tv22; tv33; tv44; tv55; tv66];

% Cell arrays for automated loop calculations of metrics
actual_torques = {tau11, tau22, tau33, tau44, tau55, tau66};
estimated_torques = {tv11, tv22, tv33, tv44, tv55, tv66};
fit_scores = zeros(1,6);

for j = 1:6
    rmse = sqrt(mean((actual_torques{j} - estimated_torques{j}).^2));
    denom = max(actual_torques{j}) - min(actual_torques{j});
    if denom == 0, denom = 1; end 
    fit_scores(j) = (1 - (rmse / denom)) * 100;
end

% Overall Cross-Validation System Accuracy
rmse_total = sqrt(mean((tt2 - tv2_all).^2));
denom_total = max(tt2) - min(tt2);
if denom_total == 0, denom_total = 1; end
fit_overall = (1 - (rmse_total / denom_total)) * 100;

% Statistical R-squared (R2) Score
SS_res = sum((tt2 - tv2_all).^2);
SS_tot = sum((tt2 - mean(tt2)).^2);
R_squared = (1 - (SS_res / SS_tot)) * 100;

%% ==================== PRINT VALIDATION RESULTS ====================
fprintf('\n================== VALIDATION EXPERIMENT 2 (6 DoF) ==================\n');
joint_names = {'Joint 1 (Base Yaw)', 'Joint 2 (Shoulder)', 'Joint 3 (Elbow)   ', ...
               'Joint 4 (Wrist 1) ', 'Joint 5 (Wrist 2) ', 'Joint 6 (Wrist 3) '};
for j = 1:6
    fprintf('%s   Fit Percentage: %6.2f %%\n', joint_names{j}, fit_scores(j));
end
fprintf('-------------------------------------------------------------\n');
fprintf('OVERALL CROSS-VALIDATION ACCURACY:  %6.2f %%\n', fit_overall);
fprintf('Statistical Validation R2 Score:    %6.4f %%\n', R_squared);
fprintf('=============================================================\n');

%% ==================== MATCHED VISUALIZATION SECTION ====================
if exist('t', 'var') && length(t) == l
    time_axis = t;
else
    time_axis = (0:l-1) * T; 
end

figure('Name', 'The Cross-Trajectory Parameter Estimation Validation Results', 'Color', 'w');
% Adjusted layout to 3x2 grid to comfortably fit all 6 DoF subplots on modern screens
t_layout = tiledlayout(3, 2, 'TileSpacing', 'normal', 'Padding', 'compact'); 

joint_titles = {'Joint 1 Dynamic Alignment', 'Joint 2 Dynamic Alignment', 'Joint 3 Dynamic Alignment', ...
                'Joint 4 Dynamic Alignment', 'Joint 5 Dynamic Alignment', 'Joint 6 Dynamic Alignment'};

for j = 1:6
    nexttile(j);
    
    plot(time_axis, actual_torques{j}, 'r-', 'LineWidth', 1.2); hold on;
    plot(time_axis, estimated_torques{j}, 'b--', 'LineWidth', 1.0);
    
    grid on;
    xlim([0, time]);
    ylabel('Torque (Nm)', 'FontSize', 10);
    title(joint_titles{j}, 'FontSize', 11, 'FontWeight', 'normal');
    
    % Add X-labels to the bottom two plots in the 3x2 grid layout (plots 5 and 6)
    if j == 5 || j == 6
        xlabel('Time (s)', 'FontSize', 10);
    end
    
    if j == 1
        legend('Ideal Real Torque', 'Optimized Model', 'Location', 'northeast', 'FontSize', 9);
    end
end
