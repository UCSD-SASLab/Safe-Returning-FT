function minor_ax = min_b(center,obstacle,major_ax,theta)
    xi = atan((center(2)-obstacle(2))/(center(1)-obstacle(1)));
    phi = theta-xi;
    dst = distance(center(1),center(2),obstacle(1),obstacle(2));
    A = cos(phi)^2 - major_ax^2;
    b = -major_ax^2*dst*sin(phi);
    fun = @(x) x;
    minor_ax = fmincon(fun,0,A,b);
    
end
