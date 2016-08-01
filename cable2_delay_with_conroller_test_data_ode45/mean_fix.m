function [ mean_res ] = mean_fix( vector )
%MEAN_FIX Summary of this function goes here
%   Detailed explanation goes here

mean_res = 0;

for i=1:length(vector)
    mean_res = (mean_res*(i-1)+vector(i))/i;
end

end
