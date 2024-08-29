function [x_pos, y_pos, z_pos] = saffAndKuijlaars(radius, nDirection)
    radius = radius *ones(1, nDirection);

    scalar = 1:nDirection;
    h_temp= -1 + 2*(scalar-1)/(nDirection-1);
    elevation= pi/2-acos(h_temp);       %sph2cart's definition of elevation angle is different. Conversion here.
    
    azimuth = zeros(1, nDirection);
    for iElevation = 2:(nDirection-1)
        azimuth(iElevation)= ...
        mod(azimuth(iElevation-1) + 3.6 /(sqrt(nDirection)*sqrt(1-h_temp(iElevation))),2*pi);
    end

    [x_pos, y_pos, z_pos] = sph2cart(azimuth, elevation, radius);
end

