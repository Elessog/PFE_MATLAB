function [ time] = fixtime(time)
%FIXTIME this function take a vector of integer value
%and return a vector where with each different time value by local extrapolaion
lenTime = length(time);
i=1;

while(i<lenTime)
    time_value =  time(i);
    j=0;
    while(i+j<lenTime)
        if time(i+j+1)==time_value
            j=j+1;
        else
            break;
        end
    end
    if j>0
        add_val = 1/(j+1);
        for k=1:j
            time(i+k) =time(i)+k*add_val; 
        end
    end
    i=i+j+1;
end

end

