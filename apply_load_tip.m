%%% Cantilever Beam statics %%%
clear all
close all
clc

%% Load and Startup SoRoSim
% Clean StartUp
diff_sorosim_path = fullfile("SoRoSim", "Differentiable_SoRoSim");
cd(diff_sorosim_path)
startup

% Switch again to the current directory
[current_path, ~, ~] = fileparts(matlab.desktop.editor.getActiveFilename);
cd(current_path)

%% Load Robot
robot_name =  "cantilever_beam";
load(fullfile("robots", robot_name, "robot_linkage.mat"));

addpath("functions");

% Turn on External Forces
T1.CEF = true;

%% Solve Statics
global load_tip;
load_tip = [5; 0.04; 0; 0; 0; 2e-3];

% Solve Statics through trust-region
q0 = 1e+0*randn(T1.ndof, 1);    % Initial guess
[equilibrium, ~, ~] = T1.statics(q0, "plot", true, "save", false);

disp("Equilibrium:")
disp(equilibrium)