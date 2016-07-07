function res = isAngleinsector(toCheck,begin,endR)
res= wrapTo2Pi(toCheck) >= wrapTo2Pi(begin) && wrapTo2Pi(toCheck) <= wrapTo2Pi(endR);
end
