function [ tau2,lambda ] = bar_length_control( fa,fb,q2,q2dot,Lint,Ldot,tau1,Ndotdot,Cdot,C,Kdc,Kpc)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
global m rode_number Kpl Kdl Kil L;


tau2=zeros(3*rode_number,1);
for i = 1:rode_number
    b=q2(1+(i-1)*3:i*3);
    norm_b = norm(b);
    bdot=q2dot(1+(i-1)*3:i*3);
    fbi=fb(1+(i-1)*3:i*3,:);%fa' and fb' orthogonal to b
    fai=fa(1+(i-1)*3:i*3,:);
    tau2i = (6/m(i))*(fbi-fai)-(b/norm_b)*(Kpl*(norm_b-L(i))...
        +Kdl*(bdot'*b/norm_b-Ldot(i)));%+Kil*Lint(i));
    tau2(1+(i-1)*3:i*3) = tau2i;
end

for j = 1:1
    
    lambda = calc_lagrange_mul( tau1,tau2,Ndotdot,Cdot,C,Kdc,Kpc);
    
    for i = 1:rode_number
        b=q2(1+(i-1)*3:i*3);
        norm_b = norm(b);
        bdot=q2dot(1+(i-1)*3:i*3);
        fbi=fb(1+(i-1)*3:i*3);%fa' and fb' orthogonal to b
        fai=fa(1+(i-1)*3:i*3);
        tau2i = (6/m(i))*(fbi-fai)-(b/norm_b)*(Kpl*(norm_b-L(i))...
            +Kdl*(bdot'*b/norm_b-Ldot(i))+Kil*Lint(i));
        tau2(1+(i-1)*3:i*3) = tau2i;
    end
end


end

