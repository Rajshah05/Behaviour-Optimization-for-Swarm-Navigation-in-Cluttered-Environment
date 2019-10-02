function [X] = RK4(dynamics, dt, X, varargin)
%% Function Description:
% This is a simple implementation of an RK4 numerical integrator.  This
% uses the classical Runge-Kutta method to numerically integrate dynamics
% forward in time.clc
%   - Chris Gnam, 2018
%
% ===============
% INPUT VARIABLES
% ===============
%   dynamics - Function handle for state space dynamic model
%   dt       - Time Step Size
%   X        - Input State Vector
%   varargin - Additional values that need to be set for dynamics
%
% ================
% OUTPUT VARIABLES
% ================
%   X        - Updated State Vector
%

%% Runge-Kutta 4th Order Solver:

% Solve for Coefficients:
k1 = dt*dynamics(0,      X,          varargin{:});
k2 = dt*dynamics(0.5*dt, X + 0.5*k1, varargin{:});
k3 = dt*dynamics(0.5*dt, X + 0.5*k2, varargin{:});
k4 = dt*dynamics(dt,     X + k3,     varargin{:});

% Calculate next state of angular velocities:
X = X + (1/6)*(k1 + 2*k2 + 2*k3 + k4);

end