clc;
clear all;
close all;

x = [100 99 95 93 88];

t = 1:length(x);

sigma1 = 1;
mu1 = 0;
amp1 = 1;
noise1 = amp1*sigma1.*randn(1,length(t)) + mu1;  % Measurement Noise 1
R1 = sigma1^2;                                   % Measurement Noise Covariance Matrix 1

Z1 = x + noise1;                                 % Measurement output 1 with measurement noise

sigma2 = 0.7;
mu2 = 0;
amp2 = 1;
noise2 = amp2*sigma2.*randn(1,length(t)) + mu2;  % Measurement Noise 2
R2 = sigma2^2;                                   % Measurement Noise Covariance Matrix 2

Z2 = x + noise2;                                 % Measurement output 2 with measurement noise

sigma3 = 0.4;
mu3 = 0;
amp3 = 1;
noise3 = amp3*sigma3.*randn(1,length(t)) + mu3;  % Measurement Noise 3
R3 = sigma3^2;                                   % Measurement Noise Covariance Matrix 3

Z3 = x + noise3;                                 % Measurement output 3 with measurement noise

sigma4 = 1.2;
mu4 = 0;
amp4 = 1;
noise4 = amp4*sigma4.*randn(1,length(t)) + mu4;  % Measurement Noise 4
R4 = sigma4^2;                                   % Measurement Noise Covariance Matrix 4

Z4 = x + noise4;                                 % Measurement output 4 with measurement noise

[Za,Ra] = fusion(Z1,R1,Z2,R2);

[Zb,Rb] = fusion(Z3,R3,Z4,R4);

[Z,R] = fusion(Za,Ra,Zb,Rb);

plot(t,x,'-b.',t,Z1,'--k.',t,Z2,'--g.',t,Z3,'--m.',t,Z4,'--y.',t,Z,'-r.')

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