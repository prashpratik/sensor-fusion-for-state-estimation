clc;
clear all;
close all;

actualData = [100 10 2 ; 99 13 5 ; 95 17 6 ; 93 19 8 ; 88 23 11];

time = 1:length(actualData);

for i=1:length(time)
    
    X = actualData(i,:)';
    
    % Data from Sensor 1
    
    sigma1 = 0.9;
    noise1 = 2*sigma1.*rand(1)-sigma1;                  % Measurement Noise 1
    R1 = [sigma1^2 0 0; 0 sigma1^2 0; 0 0 sigma1^2];    % Measurement Noise Covariance Matrix 1
    
    Z1 = X + noise1;                                    % Measurement output 1 with measurement noise

    % Data from Sensor 2
    
    sigma2 = 1.2;
    noise2 = 2*sigma2.*rand(1)-sigma2;                  % Measurement Noise 2
    R2 = [sigma2^2 0 0; 0 sigma2^2 0; 0 0 sigma2^2];    % Measurement Noise Covariance Matrix 2
    
    Z2 = X + noise2;                                    % Measurement output 2 with measurement noise
    
    % Data from Sensor 3
    
    sigma3 = 1.1;
    noise3 = 2*sigma3.*rand(1)-sigma3;                  % Measurement Noise 3
    R3 = [sigma3^2 0 0; 0 sigma3^2 0; 0 0 sigma3^2];    % Measurement Noise Covariance Matrix 3
    
    Z3 = X + noise3;                                    % Measurement output 3 with measurement noise
    
    % Data from Sensor 4
    
    sigma4 = 0.8;
    noise4 = 2*sigma4.*rand(1)-sigma4;                  % Measurement Noise 4
    R4 = [sigma4^2 0 0; 0 sigma4^2 0; 0 0 sigma4^2];    % Measurement Noise Covariance Matrix 4
    
    Z4 = X + noise4;                                    % Measurement output 4 with measurement noise
    
    % Sensor Fusion
    
    [Za,Ra] = fusion(Z1,R1,Z2,R2);
    [Zb,Rb] = fusion(Z3,R3,Z4,R4);
    [Z,R] = fusion(Za,Ra,Zb,Rb);
    
    % Plots
    
    if i==1
    Xplot = X;
    Z1plot = Z1;
    Z2plot = Z2;
    Z3plot = Z3;
    Z4plot = Z4;
    Zplot = Z;
    else
    Xplot = [Xplot  X];
    Z1plot = [Z1plot  Z1];
    Z2plot = [Z2plot  Z2];
    Z3plot = [Z3plot  Z3];
    Z4plot = [Z4plot  Z4];
    Zplot = [Zplot  Z];
    end
end

figure("Name","Distance from Object")
plot(time,Xplot(1,:),'-b.',time,Z1plot(1,:),'--k.',time,Z2plot(1,:),'--m.',time,Z3plot(1,:),'--g.',time,Z4plot(1,:),'--y.',time,Zplot(1,:),'-r.')
legend("Actual Value","Sensor 1 Measurement Value","Sensor 2 Measurement Value","Sensor 3 Measurement Value","Sensor 4 Measurement Value","Fused Measurement Value")

figure("Name","Width of the Object")
plot(time,Xplot(2,:),'-b.',time,Z1plot(2,:),'--k.',time,Z2plot(2,:),'--m.',time,Z3plot(2,:),'--g.',time,Z4plot(2,:),'--y.',time,Zplot(2,:),'-r.')
legend("Actual Value","Sensor 1 Measurement Value","Sensor 2 Measurement Value","Sensor 3 Measurement Value","Sensor 4 Measurement Value","Fused Measurement Value")

figure("Name","Height of the Object")
plot(time,Xplot(3,:),'-b.',time,Z1plot(3,:),'--k.',time,Z2plot(3,:),'--m.',time,Z3plot(3,:),'--g.',time,Z4plot(3,:),'--y.',time,Zplot(3,:),'-r.')
legend("Actual Value","Sensor 1 Measurement Value","Sensor 2 Measurement Value","Sensor 3 Measurement Value","Sensor 4 Measurement Value","Fused Measurement Value")

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