function [SkewM] = skew(vector)
%returns the skew matrix of the vector

SkewM(1,1) = 0;

SkewM(1,2) = -vector(3);
SkewM(1,3) = vector(2);

SkewM(2,1) = vector(3);
SkewM(2,2) = 0;
SkewM(2,3) = -vector(1);

SkewM(3,1) = -vector(2);
SkewM(3,2) = vector(1);
SkewM(3,3) = 0;

end

