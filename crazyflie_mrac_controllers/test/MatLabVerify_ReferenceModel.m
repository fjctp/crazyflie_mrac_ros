close all; clear; clc

Aref = [
   0.680375  0.566198
  -0.211234  0.59688];

Bref = [
   0.823295
  -0.604897];

% initial condition
x = [
-0.329554
 0.536459];

% Simulation
dt = 1/1000;
for i=1:10
  u = 0;
  if i == 3
    u = -0.444451;
  end
  if i == 8
    u = 0.10794;
  end
  
  xdot = Aref*x + Bref*u;
  x = x + xdot*dt;
  
  disp(i)
  disp(x)
  disp("")
end