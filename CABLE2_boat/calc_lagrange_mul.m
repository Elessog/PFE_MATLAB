function [ lambda ] = calc_lagrange_mul( tau1,tau2,Ndotdot,Cdot,C,Kdc,Kpc)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global Wn1c Pn1c lambdainverse;
lambda=lambdainverse*...
    (Wn1c*tau1+Pn1c*tau2+Ndotdot+Kdc*Cdot+Kpc*C);
end

