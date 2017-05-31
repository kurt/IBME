%%%Dynamic leg model going through a minimum jerk trajectory
%Written by: Sam Campbell
%Date: 2017-05-25
%%%

clear;
clc;

%% Minimum Jerk Trajectory
%Solving for Coefficients
syms x y z
eqn = [8*x + 16*y + 32*z - 1, 12*x + 32*y + 80*z, 12*x + 48*y + 160*z];
[a1, a2, a3] = solve(eqn, x, y , z);

%Setting Variables
position = [];
velocity = [];
acceleration = [];
i=1;
time = 0:0.1:2;

%Creating position/velocity/acceleration vectors for minimum jerk profile
while i<22; %time to take a step
    position(i) = (a1*time(i)^3 + a2*time(i)^4 + a3*time(i)^5);
    velocity(i) = (3*a1*time(i)^2 + 4*a2*time(i)^3 + 5*a3*time(i)^4);
    acceleration(i) = (6*a1*time(i) + 12*a2*time(i)^2 + 20*a3*time(i)^3);
    i = i+1;
end

%%Plotting to confirm minimum jerk profile
plot(time, position)
hold on
plot(time, velocity, 'r')
plot(time, acceleration, 'k')
hold off

%% Creating polynomials for Hip/Knee/Ankle motion
hip = [23 21.5 20 15 10 8 5 2 0 -5 -8 -10 0 10 15 20 22 23 23 23 23];
knee = [5 10 15 18 16 14 12 10 9 12 15 20 30 50 65 70 65 50 30 10 5];
ankle = [0 -3 -5 -3 0 3 5 8 10 12 15 10 0 -5 -15 -5 0 5 0 0 0];
hip1 = [-10 0 10 15 20 22 23 23 23 23 23 21.5 20 15 10 8 5 2 0 -5 -8];
knee1 = [20 30 50 65 70 65 50 30 10 5 5 10 15 18 16 14 12 10 9 12 15];
ankle1 = [10 0 -5 -15 -5 0 5 0 0 0 0 -3 -5 -3 0 3 5 8 10 12 15];
percentage = [0 0.05 0.1 0.15 0.2 0.25 0.3 0.35 0.4 0.45 0.5 0.55 0.6 0.65 0.7 0.75 0.8 0.85 0.9 0.95 1];
hip_coef = polyfit(percentage,hip,9);
knee_coef = polyfit(percentage,knee,9);
ankle_coef = polyfit(percentage,ankle,9);
p = 0:0.01:1;
hip_curve = polyval(hip_coef,p);
knee_curve = polyval(knee_coef,p);
ankle_curve = polyval(ankle_coef,p);

%plotting the flexion angles for all three joints
figure
plot(percentage, knee , 'k')
hold on
plot(percentage, hip, 'r')
plot(p,hip_curve, 'r')
hold on
plot(p,knee_curve, 'k')
plot(p,ankle_curve)
plot(percentage, ankle)
hold off

i=1;

%% Lopping through to create the leg motion
figure
while i<22;
%%Plotting the vectors to create the leg
Thigh1 = [0, 530 + 285];
Thigh2 = [530*sind(hip(i)), 530+285-530*cosd(hip(i))];
Thigh = [Thigh1; Thigh2];
Shank1 = [530*sind(hip(i)), 530+285-530*cosd(hip(i))];
Shank2 = [Thigh2(1,1)+285*sind(hip(i)-knee(i)), Thigh2(1,2)-285*cosd(hip(i)-knee(i))];
Shank = [Shank1; Shank2];
Foot1 = [Thigh2(1,1)+285*sind(hip(i)-knee(i)), Thigh2(1,2)-285*cosd(hip(i)-knee(i))];
Foot2 = [Shank2(1,1)+100*cosd(ankle(i)+hip(i)-knee(i)), Shank2(1,2)+100*sind(ankle(i)+hip(i)-knee(i))];
Foot = [Foot1; Foot2];
%Plotting other leg
Thigh1o = [0, 530 + 285];
Thigh2o = [530*sind(hip1(i)), 530+285-530*cosd(hip1(i))];
Thigho = [Thigh1o; Thigh2o];
Shank1o = [530*sind(hip1(i)), 530+285-530*cosd(hip1(i))];
Shank2o = [Thigh2o(1,1)+285*sind(hip1(i)-knee1(i)), Thigh2o(1,2)-285*cosd(hip1(i)-knee1(i))];
Shanko = [Shank1o; Shank2o];
Foot1o = [Thigh2o(1,1)+285*sind(hip1(i)-knee1(i)), Thigh2o(1,2)-285*cosd(hip1(i)-knee1(i))];
Foot2o = [Shank2o(1,1)+100*cosd(ankle1(i)+hip1(i)-knee1(i)), Shank2o(1,2)+100*sind(ankle1(i)+hip1(i)-knee1(i))];
Footo = [Foot1o; Foot2o];

%Plotting reference leg 
plot(Thigh(:,1),Thigh(:,2));
hold on
plot(Shank(:,1),Shank(:,2));
plot(Foot(:,1),Foot(:,2));
plot(Thigho(:,1),Thigho(:,2));
plot(Shanko(:,1),Shanko(:,2));
plot(Footo(:,1),Footo(:,2));
ylim([-100 1000]);
xlim([-500 500]);
hold off
pause(0.1);
i = i+1;
if i==22;
    i=1;
end
end
