clear all
close all
clc


%----------------------------------------------
% compose the optimization problem
%----------------------------------------------
% CasADi v3.4.5
%addpath('C:\Users\agha\Dropbox\Teaching\Classical Autonomous Systems\Lecture 5\Casadi\casadi-windows-matlabR2016a-v3.5.5')
addpath('/home/runge/Subjects/AutonomeSystems/casadiCode/')
import casadi.*
 
x = SX.sym('w'); % Decision variables (controls)
obj = exp(0.2*x)*sin(x) ; % calculate obj
 
g = [];  % Optimization constraints – empty (unconstrained)
P = [];  % Optimization problem parameters – empty (no parameters used here)
 
OPT_variables = x;  %single decision variable
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);


opts = struct;
opts.ipopt.max_iter = 1000;
opts.ipopt.print_level = 0; %0,3
opts.print_time = 0; %0,1
opts.ipopt.acceptable_tol =1e-8; % optimality convergence tolerance
opts.ipopt.acceptable_obj_change_tol = 1e-6; 

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;
args.lbx = 0;  % unconstrained optimization 
args.ubx = 4*pi;   % unconstrained optimization
args.lbg = -inf;  % unconstrained optimization
args.ubg = inf;   % unconstrained optimization

args.p   =  [];  % There are no parameters in this optimization problem
args.x0  = 10; % initialization of the optimization problem

sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
x_sol = full(sol.x)            % Get the solution
min_value = full(sol.f)   % Get the value function


%fplot(exp(0.2*x)*sin(x), [0,20])