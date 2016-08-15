function [ output_args ] = draw_cable( list_point,figureN ,color)
%DRAW_CABLE Summary of this function goes here
%   Detailed explanation goes here


if nargin < 3
    figureN = 666;
end
    
if (length(list_point(1,:)) < 2 || length(list_point(1,:)) > 3)
    disp('Error in dimension of the cable');
    return 
end
figure(figureN)
if  length(list_point(1,:)) == 2
    hold on
    for i=1:length(list_point(:,1))-1
       plot([list_point(i,1),list_point(i+1,1)],[list_point(i,2),list_point(i+1,2)])
    end
    hold off
else %nbDim == 3
   hold on
   for i=1:length(list_point(:,1))-1
       r = mod(i,length(color))+1;
       plot3([list_point(i,1),list_point(i+1,1)],[list_point(i,2),list_point(i+1,2)],[list_point(i,3),list_point(i+1,3)],color(r))
   end 
   hold off
end
end

