% Before you run This code check the dimension Of Beta (the symbolic vector that contains 
% the parameter to be estimated) in the workspace 
%% and Run the simulink File named dataextractForSecondExpValidation.slx
% Abdelrahman ELzemrany (6-DOF Direct Kinematics - Parameter Estimation)

k = 48;      % Dimension of Beta
time = 10;   % Experiment time
T = 0.1;   % Sampling period
l = round(time/T);  % Number of samples (10000)

%% 1. Joint Data Extraction (Direct Sensor Readings)
q1 = pos.signals(1).values(1:l); q1 = q1(:);
q2 = pos.signals(2).values(1:l); q2 = q2(:);
q3 = pos.signals(3).values(1:l); q3 = q3(:);
q4 = pos.signals(4).values(1:l); q4 = q4(:);
q5 = pos.signals(5).values(1:l); q5 = q5(:);
q6 = pos.signals(6).values(1:l); q6 = q6(:);

qp1 = velocity.signals(1).values(1:l); qp1 = qp1(:);
qp2 = velocity.signals(2).values(1:l); qp2 = qp2(:);
qp3 = velocity.signals(3).values(1:l); qp3 = qp3(:);
qp4 = velocity.signals(4).values(1:l); qp4 = qp4(:);
qp5 = velocity.signals(5).values(1:l); qp5 = qp5(:);
qp6 = velocity.signals(6).values(1:l); qp6 = qp6(:);

qpp1 = accelration.signals(1).values(1:l); qpp1 = qpp1(:);
qpp2 = accelration.signals(2).values(1:l); qpp2 = qpp2(:);
qpp3 = accelration.signals(3).values(1:l); qpp3 = qpp3(:);
qpp4 = accelration.signals(4).values(1:l); qpp4 = qpp4(:);
qpp5 = accelration.signals(5).values(1:l); qpp5 = qpp5(:);
qpp6 = accelration.signals(6).values(1:l); qpp6 = qpp6(:);

tau1 = torque.signals(1).values(1:l); tau1 = tau1(:);  
tau2 = torque.signals(2).values(1:l); tau2 = tau2(:);
tau3 = torque.signals(3).values(1:l); tau3 = tau3(:);
tau4 = torque.signals(4).values(1:l); tau4 = tau4(:);  
tau5 = torque.signals(5).values(1:l); tau5 = tau5(:);
tau6 = torque.signals(6).values(1:l); tau6 = tau6(:);

%% 2. Data Cropping Configuration (Maintained for transient stabilization)
crop_idx = 5;
valid_range = (crop_idx + 1) : (l - crop_idx);

q1 = q1(valid_range); q2 = q2(valid_range); q3 = q3(valid_range);
q4 = q4(valid_range); q5 = q5(valid_range); q6 = q6(valid_range);

qp1 = qp1(valid_range); qp2 = qp2(valid_range); qp3 = qp3(valid_range);
qp4 = qp4(valid_range); qp5 = qp5(valid_range); qp6 = qp6(valid_range);

qpp1 = qpp1(valid_range); qpp2 = qpp2(valid_range); qpp3 = qpp3(valid_range);
qpp4 = qpp4(valid_range); qpp5 = qpp5(valid_range); qpp6 = qpp6(valid_range);

tau1 = tau1(valid_range); tau2 = tau2(valid_range); tau3 = tau3(valid_range);
tau4 = tau4(valid_range); tau5 = tau5(valid_range); tau6 = tau6(valid_range);

tt = [tau1; tau2; tau3; tau4; tau5; tau6];
l = length(valid_range);

%% 3. Rearranging The Observation Regressor Matrix 
fprintf('Assembling Regressor Matrix...');
Yb = zeros(6, k, l);
for i = 1:l
   Yb(:,:,i) = Y_b_handle(0,0,-9.81, q1(i),q2(i),q3(i),q4(i),q5(i),q6(i), ...
                                   qp1(i),qp2(i),qp3(i),qp4(i),qp5(i),qp6(i), ...
                                   qpp1(i),qpp2(i),qpp3(i),qpp4(i),qpp5(i),qpp6(i));
end

% SPEEDUP: Vectorized Matrix Rearrangement for 6 channels
sum1 = zeros(l, k); sum2 = zeros(l, k); sum3 = zeros(l, k);
sum4 = zeros(l, k); sum5 = zeros(l, k); sum6 = zeros(l, k);

% Preserving your exact structural decomposition to prevent dimension mixing
for ii = 1:k
    for ik = 1:l
        sum1(ik, ii) = Yb(1, ii, ik);
        sum2(ik, ii) = Yb(2, ii, ik);
        sum3(ik, ii) = Yb(3, ii, ik);
        sum4(ik, ii) = Yb(4, ii, ik);
        sum5(ik, ii) = Yb(5, ii, ik);
        sum6(ik, ii) = Yb(6, ii, ik);
    end
end
Yc = [sum1; sum2; sum3; sum4; sum5; sum6]; 

fprintf(' Done.\n');

%% 4. The Convex Constrained Optimization 
%% 4.1 Unconstrained Baseline
theta_init = pinv(Yc'*Yc)*(Yc'*tt);

%% 4.2 Constrained Optimization Setup
fprintf('Pre-calculating Mass Matrix Grid Regressors for 6-DOF Optimization...\n');

q_lims = [-pi,   pi;      % Joint 1 Min/Max
          -pi, pi;    % Joint 2 Min/Max
          -pi, pi;    % Joint 3 Min/Max
          -pi,   pi;      % Joint 4 Min/Max
          -pi, pi;    % Joint 5 Min/Max
          -pi,   pi];     % Joint 6 Min/Max

% SPEEDUP: Monte Carlo pre-calculation grid for 6-DOF avoids nested loop overhead
rng(42); 
num_samples = 150; 
q_rand_grid = q_lims(:,1) + (q_lims(:,2) - q_lims(:,1)) .* rand(6, num_samples);

% Store regressors for sampled positions, 6 columns each
Y_grid_blocks = cell(num_samples, 6); 

for idx = 1:num_samples
    q_curr = q_rand_grid(:, idx);
    for col = 1:6
        qpp_pulse = zeros(6,1);
        qpp_pulse(col) = 1;
        % Pulse handle with zero gravity/velocity and unit acceleration over 6 dimensions
        Y_grid_blocks{idx, col} = Y_b_handle(0,0,0, q_curr(1),q_curr(2),q_curr(3),q_curr(4),q_curr(5),q_curr(6), ...
                                             0,0,0,0,0,0, ...
                                             qpp_pulse(1),qpp_pulse(2),qpp_pulse(3),qpp_pulse(4),qpp_pulse(5),qpp_pulse(6));
    end
end

fprintf('Running Convex Constrained Optimization via fmincon...\n');

lambda_reg = 1e-6; 
obj_fun = @(theta) sum((Yc * theta - tt).^2) + lambda_reg * sum(theta.^2);

options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'ScaleProblem', 'obj-and-constr', ... 
    'Display', 'iter-detailed', ...        
    'OptimalityTolerance', 1e-12, ...       
    'StepTolerance', 1e-12, ...
    'MaxFunctionEvaluations', 100000, ...
    'MaxIterations', 5000);

% Pass pre-calculated blocks directly into the optimized constraint function
[theta, fval, exitflag] = fmincon(obj_fun, theta_init, [], [], [], [], [], [], ...
    @(theta) mass_constraints_fast_6dof(theta, Y_grid_blocks, num_samples), options);


%% --- OPTIMIZED 6-DOF HELPER CONSTRAINT FUNCTION ---
function [c, ceq] = mass_constraints_fast_6dof(theta, Y_grid_blocks, num_samples)
    ceq = []; 
    epsilon = 0.0004; % Minimum acceptable eigenvalue boundary for M(q)
    
    % Pre-allocate constraint vector for speed
    c = zeros(num_samples, 1);
    M_curr = zeros(6,6);
    
    for idx = 1:num_samples
        % Fast linear multiplication directly from pre-calculated grid blocks
        M_curr(:, 1) = Y_grid_blocks{idx,1} * theta;
        M_curr(:, 2) = Y_grid_blocks{idx,2} * theta;
        M_curr(:, 3) = Y_grid_blocks{idx,3} * theta;
        M_curr(:, 4) = Y_grid_blocks{idx,4} * theta;
        M_curr(:, 5) = Y_grid_blocks{idx,5} * theta;
        M_curr(:, 6) = Y_grid_blocks{idx,6} * theta;
        
        eg = eig(M_curr);
        c(idx) = epsilon - min(eg);
    end
end
