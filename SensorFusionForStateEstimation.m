clc;
clear all;
close all;

%% Setting up the known parameters

Ts = 1;                                           % Sampling time 
t_final = 5;                                      % Simulation end time
t = 1:Ts:t_final;                                 % Time axis
numIters = length(t);                             % Number of iterations 

x = [100 ; 10 ; 2 ; 5];                           % Initial State Vector
x_hat = x;                                        % Initial State Vector Estimate
P = [1 0 0 0 ; 0 1 0 0 ; 0 0 1 0 ; 0 0 0 1];      % Initial Covariance Matrix
x_hat_previous = x;                               % Previous State Vector
P_previous = P;                                   % Previous Covariance Matrix

A = [1 0 0 -Ts ; 0 1 0 Ts ; 0 0 1 Ts ; 0 0 0 1];  % State Transition Matrix or Prediction Matrix
B = [-Ts^2/2 ; Ts^2/2 ; Ts^2/2 ; Ts];             % Input Control Matrix
C = [1 0 0 0 ; 0 1 0 0 ; 0 0 1 0 ; 0 0 0 1];      % Measurement Matrix

u = zeros(1,length(t));                           % Input signal or Control vector

x_actual = x;

for i=1:numIters  
    x_actual = A*x_actual + B*u(i);
    y(:,i) = C*x_actual;                          % Ideal Measurement output
end

sigma_p = 0.2;
amp_p = 0.05;
noise_p = amp_p*(2*sigma_p.*rand(1,length(t))-sigma_p);                    % Process Noise
Q = sigma_p^2;                                                             % Process Noise Covariance Matrix

% Data from Sensor 1

sigma1 = 0.9;
amp1 = 0.05;
noise1 = amp1*(2*sigma1.*rand(1,length(t))-sigma1);                        % Measurement Noise 1
R1 = [sigma1^2 0 0 0 ; 0 sigma1^2 0 0 ; 0 0 sigma1^2 0 ; 0 0 0 sigma1^2];  % Measurement Noise Covariance Matrix 1

Z1 = y + y.*noise1;                                                        % Measurement output 1 with measurement noise

% Data from Sensor 2

sigma2 = 1.2;
amp2 = 0.05;
noise2 = amp2*(2*sigma2.*rand(1,length(t))-sigma2);                        % Measurement Noise 2
R2 = [sigma2^2 0 0 0 ; 0 sigma2^2 0 0 ; 0 0 sigma2^2 0 ; 0 0 0 sigma2^2];  % Measurement Noise Covariance Matrix 2

Z2 = y + y.*noise2;                                                        % Measurement output 2 with measurement noise

% Data from Sensor 3

sigma3 = 1.1;
amp3 = 0.05;
noise3 = amp3*(2*sigma3.*rand(1,length(t))-sigma3);                        % Measurement Noise 3
R3 = [sigma3^2 0 0 0 ; 0 sigma3^2 0 0 ; 0 0 sigma3^2 0 ; 0 0 0 sigma3^2];  % Measurement Noise Covariance Matrix 3

Z3 = y + y.*noise3;                                                        % Measurement output 3 with measurement noise

% Data from Sensor 4

sigma4 = 0.8;
amp4 = 0.05;
noise4 = amp4*(2*sigma4.*rand(1,length(t))-sigma4);                        % Measurement Noise 4
R4 = [sigma4^2 0 0 0 ; 0 sigma4^2 0 0 ; 0 0 sigma4^2 0 ; 0 0 0 sigma4^2];  % Measurement Noise Covariance Matrix 4

Z4 = y + y.*noise4;                                                        % Measurement output 4 with measurement noise

%% Predict-Update Process

for i=1:numIters
    % Predict
    [x_hat1, P1] = predict(x_hat_previous,P_previous,A,B,u(i),Q);
    [x_hat2, P2] = predict(x_hat_previous,P_previous,A,B,u(i),Q);
    [x_hat3, P3] = predict(x_hat_previous,P_previous,A,B,u(i),Q);
    [x_hat4, P4] = predict(x_hat_previous,P_previous,A,B,u(i),Q);
    
    % Update
    [x_hat_updated1, P_updated1] = update(x_hat1,P1,C,Z1(:,i),R1);
    y_hat1(:,i) = C*x_hat_updated1;
    [x_hat_updated2, P_updated2] = update(x_hat2,P2,C,Z2(:,i),R2);
    y_hat2(:,i) = C*x_hat_updated2;
    [x_hat_updated3, P_updated3] = update(x_hat3,P3,C,Z3(:,i),R3);
    y_hat3(:,i) = C*x_hat_updated3;
    [x_hat_updated4, P_updated4] = update(x_hat4,P4,C,Z4(:,i),R4);
    y_hat4(:,i) = C*x_hat_updated4;
    
    % Sensor Fusion
    
    [Za,Ra] = fusion(x_hat_updated1,P_updated1,x_hat_updated2,P_updated2);
    [Zb,Rb] = fusion(x_hat_updated3,P_updated3,x_hat_updated4,P_updated4);
    [x_hat_fused,P_fused] = fusion(Za,Ra,Zb,Rb);
    
    x_hat_previous = x_hat_fused;
    P_previous = P_fused;
    y_fused(:,i) = C*x_hat_fused;
end

% Distance of Object from Vehicle

figure("Name","Distance of Object from Vehicle")

subplot(2,2,1)
plot(t,y(1,:),'-b.',t,Z1(1,:),'--.',t,y_hat1(1,:),'-g.',t,y_fused(1,:),'-r.')
legend("Actual Data","Measured Output","Estimated Output","Fused Output")
grid on;

subplot(2,2,2)
plot(t,y(1,:),'-b.',t,Z2(1,:),'--.',t,y_hat2(1,:),'-g.',t,y_fused(1,:),'-r.')
legend("Actual Data","Measured Output","Estimated Output","Fused Output")
grid on;

subplot(2,2,3)
plot(t,y(1,:),'-b.',t,Z3(1,:),'--.',t,y_hat3(1,:),'-g.',t,y_fused(1,:),'-r.')
legend("Actual Data","Measured Output","Estimated Output","Fused Output")
grid on;

subplot(2,2,4)
plot(t,y(1,:),'-b.',t,Z4(1,:),'--.',t,y_hat4(1,:),'-g.',t,y_fused(1,:),'-r.')
legend("Actual Data","Measured Output","Estimated Output","Fused Output")
grid on;

% Width of Object

figure("Name","Width of Object")

subplot(2,2,1)
plot(t,y(2,:),'-b.',t,Z1(2,:),'--.',t,y_hat1(2,:),'-g.',t,y_fused(2,:),'-r.')
legend("Actual Data","Measured Output","Estimated Output","Fused Output")
grid on;

subplot(2,2,2)
plot(t,y(2,:),'-b.',t,Z2(2,:),'--.',t,y_hat2(2,:),'-g.',t,y_fused(2,:),'-r.')
legend("Actual Data","Measured Output","Estimated Output","Fused Output")
grid on;

subplot(2,2,3)
plot(t,y(2,:),'-b.',t,Z3(2,:),'--.',t,y_hat3(2,:),'-g.',t,y_fused(2,:),'-r.')
legend("Actual Data","Measured Output","Estimated Output","Fused Output")
grid on;

subplot(2,2,4)
plot(t,y(2,:),'-b.',t,Z4(2,:),'--.',t,y_hat4(2,:),'-g.',t,y_fused(2,:),'-r.')
legend("Actual Data","Measured Output","Estimated Output","Fused Output")
grid on;

% Height of Object

figure("Name","Height of Object")

subplot(2,2,1)
plot(t,y(3,:),'-b.',t,Z1(3,:),'--.',t,y_hat1(3,:),'-g.',t,y_fused(3,:),'-r.')
legend("Actual Data","Measured Output","Estimated Output","Fused Output")
grid on;

subplot(2,2,2)
plot(t,y(3,:),'-b.',t,Z2(3,:),'--.',t,y_hat2(3,:),'-g.',t,y_fused(3,:),'-r.')
legend("Actual Data","Measured Output","Estimated Output","Fused Output")
grid on;

subplot(2,2,3)
plot(t,y(3,:),'-b.',t,Z3(3,:),'--.',t,y_hat3(3,:),'-g.',t,y_fused(3,:),'-r.')
legend("Actual Data","Measured Output","Estimated Output","Fused Output")
grid on;

subplot(2,2,4)
plot(t,y(3,:),'-b.',t,Z4(3,:),'--.',t,y_hat4(3,:),'-g.',t,y_fused(3,:),'-r.')
legend("Actual Data","Measured Output","Estimated Output","Fused Output")
grid on;

%% Predict Function

function [newStateVector, newCovariance] = predict(previousStateVector,previousCovariance,stateTransitionMatrix,controlMatrix,controlVector,processNoiseCovariance)
    x_hat_previous = previousStateVector;
    P_previous = previousCovariance;
    A = stateTransitionMatrix;
    B = controlMatrix;
    u = controlVector;
    Q = processNoiseCovariance;
    
    x_hat = A*x_hat_previous + B*u;
    P = A*P_previous*A' + Q;
    
    newStateVector = x_hat;
    newCovariance = P;
end

%% Update Function

function [updatedStateVector, updatedCovariance] = update(StateVector,Covariance,measurementMatrix,measurementOutput,measurementNoiseCovariance)
    x_hat = StateVector;
    P = Covariance;
    C = measurementMatrix;
    Z = measurementOutput;
    R = measurementNoiseCovariance;

    K = P*C'/((C*P*C') + R);  % Kalman Gain
    x_hat_updated = x_hat + K * (Z - (C*x_hat));
    P_updated = (eye(size(C)) - (K*C)) * P;
    
    updatedStateVector = x_hat_updated;
    updatedCovariance = P_updated;
end

%% Fusion Function

function [fusedMeasurementOutput, fusedCovariance] = fusion(measurementOutput1,measurementNoiseCovariance1,measurementOutput2,measurementNoiseCovariance2)

    Z1 = measurementOutput1;
    R1 = measurementNoiseCovariance1;
    Z2 = measurementOutput2;
    R2 = measurementNoiseCovariance2;

    F = R1/(R1 + R2);  % Fused Gain
    Z = Z1 + F * (Z2 - Z1);
    R = R1 - F * R1;
    
    fusedMeasurementOutput = Z;
    fusedCovariance = R;
end