%% copyright © November 2021, R. Olusegun Alli-Oke.

clc, clear all; format; 


%% Saturation Constraints
umax=1; dr = 25; st = 10; ST = 35; N = 1000; maxDelay = 10; % maximum delay parameter (maxDelay) of delay block. 


%% Real Network-Parameters
wmax=131; qmax=800; q0=175; n=60; c=3750; Tp=0.2; tau0=(q0/c)+Tp; w0=(c*tau0)/n; u0=(2/(w0^2)); display([q0,Tp,tau0,w0,u0])     %(Ex1 Hollot et al 2002)
% wmax=66; qmax=800; q0=100; n=100; c=800; Tp=0.2; tau0=(q0/c)+Tp; w0=(c*tau0)/n; u0=(2/(w0^2)); display([q0,Tp,tau0,w0,u0])      %(Ex2 Azadegan and Beheshti 2014)


%% Nominal Network-Parameters  +  Linearized (PID) Controllers
   % Nominal in the sense that that these are the values used for controller design. In these examples, nominal values are same as real values.
wmaxn=131; qmaxn=800; q0=175; nn=60; cn=3750; Tpn=0.2; tau0n=(q0/cn)+Tpn; w0n=(cn*tau0n)/n; u0n=(2/(w0n^2)); display([q0,Tpn,tau0n,w0n,u0n])        %(Ex1 Hollot et al 2002)
% wmaxn=66; qmaxn=800; q0=100; nn=100; cn=800; Tpn=0.2; tau0n=(q0/cn)+Tpn; w0n=(cn*tau0n)/n; u0n=(2/(w0n^2)); display([q0,Tpn,tau0n,w0n,u0n])         %(Ex2 Azadegan and Beheshti 2014)    


%% Simulation Files
% PID_JNLM_x1 : compares between Jacobi Model and Misra Model
% PID_NLM_x2a : compares between different controllers using Misra Model
% PID_NLM_x2b : same as PID_NLM_x2a except for the Kahe Controller. 
% PID_NLM_x2c : same as PID_NLM_x2a except that the PID controller blocks uses du/dt block.


        %% Ex 1: Hollot et al 2002 : PID_NLM_x2a, PID_JNLM_x1
        kp1 = -18.189e-6;          ki1 = -9.640e-6;       kd1 = 0;                       %Proportional-Integral Control
        %% Ex 1: Ustebay and Ozbay 2007 : PID_NLM_x2a
%         kp2 = -35.252e-6;          ki2 = -8.956e-6;       kd2 = 0;                        %Proportional-Integral Control
        %% Ex 1: Hammouri et al 2006 : PID_NLM_x2a
%         kp1 = -100e-6;             ki1 = -60e-6;          kd1 = 0;                        %Proportional-Integral Control
        %% Ex 1: Gu et al 2008 : PID_NLM_x2a
%         kp2 = -84.962e-6;          ki2 = -36.832e-6;      kd2 = 0;                        %Proportional-Integral Control
        %% Ex 1: Ge et al 2010 : PID_NLM_x2b
%         kp1 = -100e-6;             ki1 = -200e-6;         kd1 = -50e-6;                   %Proportional-Integral-Derivative Control
        %% Ex 1: Kahe et al 2014 : PID_NLM_x2b
%         kp2 = -4.450;              ki2 =  -1.970;         kd2 = -1;                       %Proportional-Integral-Derivative Control 
%%   
        %% Ex 2: Azadegan and Beheshti 2014 : PID_NLM_x2c
%         kp1 = -6e-4;               ki1 = -1e-4;           kd1 = -12.0e-4;   ST = 50;      %Proportional-Integral-Derivative Control
        %% Ex 2: P-Pina and M-Aguilar 2016 : PID_NLM_x2c
%         kp2 = -12.617e-4;          ki2 = 0;               kd2 = -9.500e-4;                %Proportional-Derivative Control


%% Transfer Function Model (Hollot-2002)
alpha=w0/2;
Pw=tf([-2*alpha^3],[alpha*tau0 1]); Gq=tf([n],[tau0 1]);
[numPw,denPw] = tfdata(Pw,'v'); [numGq,denGq] = tfdata(Gq,'v');
G1=Pw*Gq; [numG1,denG1] = tfdata(G1,'v');


%% Simulate Simulink Model n save data for plotting
% sim('PID_JNLM_x1')
%     save Ex1HollotHollot25
%     save Ex1HollotHollot300
%     save Ex1HollotHollot500
% sim('PID_NLM_x2a')
%     save Ex1HollotUstebay
%     save Ex1HammouriGu
% sim('PID_NLM_x2b')
%     save Ex1GeKahe
% sim('PID_NLM_x2c')
%     save Ex2AzadeganPpina


%%


