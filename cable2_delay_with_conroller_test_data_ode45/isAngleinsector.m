function res = isAngleinsector(toCheck,begin,endR)

res1 =  (wrapTo2Pi(toCheck) >= wrapTo2Pi(begin));
res2 = wrapTo2Pi(toCheck) <= wrapTo2Pi(endR);
res =0;
if (all(res1))
    if(all(res2));
        res = 1;
    end
end
end
