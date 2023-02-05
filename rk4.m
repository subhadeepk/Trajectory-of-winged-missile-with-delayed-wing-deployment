function [A] = rk4(t,A_dash,A,h)


f = @(t,A_dash) A_dash;

  k_1 = f(t,A_dash);
 k_2 = f(t+0.5*h,A_dash+0.5*h*k_1);
 k_3 = f((t+0.5*h),(A_dash+0.5*h*k_2));
 k_4 = f((t+h),(A_dash+k_3*h));
    
    A = A + (1/6)*(k_1+2*k_2+2*k_3+k_4)*h;
end

