function V = logmapR(R)
%LOGMAP do the logarithmic mapping from SO3 to R3
    theta = acos((trace(R)-1)/2);
    v = 1  / (sin(theta)/2) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
    
    if theta == 0
        V = zeros(3,1);
    else
        V = theta * v;
    end
end

