function G = GAMMA(omega,dt,k)
%calculates the skew series of the omega*dt term,
%if norm->0  the limits are calculated within


b = mod(k,2);
m = (k-b)/2;

norma = norm(omega);

if b == 0
    if norma > 0.1
    sinw = ((-1)^m)*sin(norma*dt);
    sum = 0;
    for n=0:(m-1)
        sign = (-1)^(m+n+1);
        ser_prod = (norma*dt)^(2*n+1);
        sum = sum + sign*ser_prod/factorial(2*n+1);
    end
    
    skew_gain = 1/(norma^(2*m+1))*(sinw+sum);
    
    sum = 0;
    
    cosw = ((-1)^(m+1))*cos(norma*dt);
    for n=0:m
        sign2 = (-1)^(m+n);
        ser_prod2 = (norma*dt)^(2*n);
        sum = sum + sign2*ser_prod2/factorial(2*n);
    end
    
    skew_sqr_gain = 1/(norma^(2*m+1+1))*(cosw+sum);
    else
        skew_gain = (dt^(2*m+1))/factorial(2*m+1);
        skew_sqr_gain = ((-1)^(2*m+1))*((dt^(2*m+2))/factorial(2*m+2));
    end

    
else
     if norma > 0.1
         
         sum=0;
         cosw = ((-1)^(m+1))*cos(norma*dt);
         
    for n=0:m
        sign2 = (-1)^(m+n);
        ser_prod2 = (norma*dt)^(2*n);
        sum = sum + sign2*ser_prod2/factorial(2*n);
    end
    
    skew_gain = (1/(norma^(2*m+2)))*(cosw + sum);
    
    sum = 0;
    sinw = ((-1)^(m+1))*(sin(norma*dt));
    for n = 0:m
        sign = (-1)^(n+m);
        ser_prod = ((norma*dt)^(2*n+1));
        sum  = sum + ((sign*ser_prod)/factorial(2*n+1));
    end
    
    skew_sqr_gain = (1/(norma^(2*m+3)))*(sinw + sum);
    
    
     else
         skew_gain = ((-1)^(2*m+1))*((dt^(2*m+2))/factorial(2*m+2));
         skew_sqr_gain = 0;
     end
end
G = ((dt^(k))/(factorial(k)))*eye(3) + skew_gain*skew(omega) +......
    skew_sqr_gain*skew(omega)*skew(omega);

end

    





