% Abdelrahman ELzemrany - Universal Excitation Trajectory Optimization
clc; close all; 

fprintf('=================================================================\n');
fprintf('  STARTING UNIVERSAL ROBOT TRAJECTORY OPTIMIZATION PIPELINE      \n');
fprintf('=================================================================\n\n');

%% 1. GLOBAL CONFIGURATION (Change these two numbers for ANY robot arm)
setup.num_joints = 6;          % Set to 2, 3, 4, 6, 7, etc. 
setup.N_harmonics = 5;         % Number of unique frequencies per joint

setup.omega_f = 2 * pi / 10;   % Base trajectory frequency (10-second period)
setup.vars_per_joint = 2 * (setup.N_harmonics - 1); 
setup.total_vars = setup.num_joints * setup.vars_per_joint;
setup.t_grid = 0:0.1:10;       % Optimization Grid

%% 2. ENFORCE PHYSICAL SYSTEM BOUNDARY CONSTRAINTS (DYNAMICAL SCALE)
% Automatically populates limits uniformly for all N active joints
setup.pos_limits = repmat([-3.1, 3.1], setup.num_joints, 1); 
setup.vel_limits = repmat([-5.0, 5.0], setup.num_joints, 1); 
setup.acc_limits = repmat([-10.0, 10.0], setup.num_joints, 1);

% Optional: Override unique hardware properties for specific links safely

%% 3. OPTIMIZER INITIALIZATION
rng(42); 
x0 = (rand(setup.total_vars, 1) - 0.5) * 0.5; 

if ~exist('Y_b_handle', 'var')
    Y_b_handle = @Y_b_handle; 
end

options = optimoptions('fmincon', ...
    'Display', 'iter-detailed', ...
    'Algorithm', 'sqp', ...
    'MaxFunctionEvaluations', 30000, ...
    'MaxIterations', 500);

%% 4. EXECUTE THE OPTIMIZATION LOOP
obj_fun = @(x) objective_function_inline(x, setup, Y_b_handle);
constr_fun = @(x) constraints_function_inline(x, setup);

[x_opt, fval] = fmincon(obj_fun, x0, [], [], [], [], [], [], constr_fun, options);

%% 5. SAVE MATRIX PARAMETERS TO DISK
save('optimal_fourier_coefficients.mat', 'x_opt', 'setup');
fprintf('\n>>> Optimization Successful! Parameters saved to disk.\n\n');

%% 6. HIGH-RESOLUTION SIMULINK SIGNAL GENERATION (1000 Hz)
t_sim = (0:0.001:10)'; 
[q, qp, qpp] = reconstruct_fourier_trajectory_inline(x_opt, t_sim', setup.omega_f, setup.N_harmonics, setup.num_joints);

% Dynamically pushes q_ref_joint1 through q_ref_jointN to workspace
for j = 1:setup.num_joints
    assignin('base', sprintf('q_ref_joint%d', j), [t_sim, q(j,:)']);
end

%% 7. PLOT THE TRAJECTORY PROFILES (FULLY GENERALIZED)
figure('Name', 'Optimized Trajectory - Fully Generalized', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.6, 0.7]);
lbls = arrayfun(@(x) sprintf('Joint %d', x), 1:setup.num_joints, 'UniformOutput', false);

subplot(3,1,1); plot(t_sim, q', 'LineWidth', 2); grid on;
title('Zero-Bounded Joint Positions (q) [q(0) = 0]');
ylabel('Position [rad]'); legend(lbls, 'Location', 'best');
ylim([min(setup.pos_limits(:,1))*1.2, max(setup.pos_limits(:,2))*1.2]); 

subplot(3,1,2); plot(t_sim, qp', 'LineWidth', 2); grid on;
title('Zero-Bounded Joint Velocities (q-dot) [q-dot(0) = 0]');
ylabel('Velocity [rad/s]'); legend(lbls, 'Location', 'best');
ylim([min(setup.vel_limits(:,1))*1.2, max(setup.vel_limits(:,2))*1.2]); 

subplot(3,1,3); plot(t_sim, qpp', 'LineWidth', 2); grid on;
title('Zero-Bounded Joint Accelerations (q-double-dot) [q-double-dot(0) = 0]');
ylabel('Acceleration [rad/s²]'); xlabel('Time [seconds]'); legend(lbls, 'Location', 'best');
ylim([min(setup.acc_limits(:,1))*1.2, max(setup.acc_limits(:,2))*1.2]); 


%% =========================================================================
%% INLINE MATHEMATICAL UTILITIES (CONSTRAINED TO ZERO BOUNDARIES)
%% =========================================================================

function [q_out, qp_out, qpp_out] = reconstruct_fourier_trajectory_inline(x, t_vec, omega, N_harm, num_joints)
    v_per_j = 2 * (N_harm - 1); 
    n_samples = length(t_vec);
    
    q_out = zeros(num_joints, n_samples);
    qp_out = zeros(num_joints, n_samples);
    qpp_out = zeros(num_joints, n_samples);
    
    for j_idx = 1:num_joints
        s_idx = (j_idx-1)*v_per_j + 1;
        j_vars = x(s_idx : s_idx + v_per_j - 1);
        
        a_high = j_vars(1 : N_harm-1);      
        b_high = j_vars(N_harm : end);       
        
        a = zeros(N_harm, 1);
        b = zeros(N_harm, 1);
        a(2:end) = a_high;
        b(2:end) = b_high;
        
        a(1) = -sum(a_high);
        k_vec = (2:N_harm)';
        acc_sum = sum(b(k_vec) .* k_vec * omega);
        b(1) = -acc_sum / omega;
        
        k_all = (1:N_harm)';
        q0_val = sum(b(k_all) ./ (k_all * omega));
        
        q_out(j_idx,:) = q0_val;
        for harmonic = 1:N_harm
            kf_val = omega * harmonic;
            q_out(j_idx,:)   = q_out(j_idx,:)   + (a(harmonic)/kf_val) .* sin(kf_val.*t_vec) - (b(harmonic)/kf_val) .* cos(kf_val.*t_vec);
            qp_out(j_idx,:)  = qp_out(j_idx,:)  + a(harmonic) .* cos(kf_val.*t_vec) + b(harmonic) .* sin(kf_val.*t_vec);
            qpp_out(j_idx,:) = qpp_out(j_idx,:) - a(harmonic)*kf_val .* sin(kf_val.*t_vec) + b(harmonic)*kf_val .* cos(kf_val.*t_vec);
        end
    end
end

function cost_val = objective_function_inline(x_vars, setup_struct, handle_Y)
    nj = setup_struct.num_joints;
    [q_eval, qp_eval, qpp_eval] = reconstruct_fourier_trajectory_inline(...
        x_vars, setup_struct.t_grid, setup_struct.omega_f, setup_struct.N_harmonics, nj);
    n_smpl = length(setup_struct.t_grid);
    
    % Dynamically discover parameter dimensions without editing raw files
    dummy_args = num2cell(zeros(1, 3 * nj));
    sample_Y = handle_Y(0, 0, -9.81, dummy_args{:});
    n_base_parameters = size(sample_Y, 2);
    
    W_g = zeros(nj * n_smpl, n_base_parameters);
    
    for step = 1:n_smpl
        r_idx = (nj * step - (nj - 1)):(nj * step);
        
        % DYNAMICALLY UNPACK VECTOR ROW TO INDIVIDUAL COMMA SEPARATED SCALARS
        q_scalars   = num2cell(q_eval(:, step));
        qp_scalars  = num2cell(qp_eval(:, step));
        qpp_scalars = num2cell(qpp_eval(:, step));
        
        W_g(r_idx, :) = handle_Y(0, 0, -9.81, ...
                                 q_scalars{:}, ...
                                 qp_scalars{:}, ...
                                 qpp_scalars{:});
    end
    
    Info_Mat = W_g' * W_g;
    dt_val = det(Info_Mat);
    
    if dt_val > 1e-5
        cost_val = -log(dt_val);
    else
        cost_val = 50.0 - dt_val * 1000; 
    end
end

function [c_val, ceq_val] = constraints_function_inline(x_vars, setup_struct)
    [q_eval, qp_eval, qpp_eval] = reconstruct_fourier_trajectory_inline(...
        x_vars, setup_struct.t_grid, setup_struct.omega_f, setup_struct.N_harmonics, setup_struct.num_joints);
    ceq_val = [];  
    
    c_val = zeros(6 * setup_struct.num_joints, 1);
    c_idx = 1;
    
    for joint = 1:setup_struct.num_joints
        c_val(c_idx)   = max(q_eval(joint,:)) - setup_struct.pos_limits(joint,2);
        c_val(c_idx+1) = setup_struct.pos_limits(joint,1) - min(q_eval(joint,:));
        
        c_val(c_idx+2) = max(qp_eval(joint,:)) - setup_struct.vel_limits(joint,2);
        c_val(c_idx+3) = setup_struct.vel_limits(joint,1) - min(qp_eval(joint,:));
        
        c_val(c_idx+4) = max(qpp_eval(joint,:)) - setup_struct.acc_limits(joint,2);
        c_val(c_idx+5) = setup_struct.acc_limits(joint,1) - min(qpp_eval(joint,:));
        
        c_idx = c_idx + 6;
    end
end
