function rECEF = WGSpos2ECEFpos(rWGS)
    %    
    % rWGS: (3,1) vector containing coordinates in WGS coordinates
    %       (phi, lambda, h)'
    
    a = 6.378137e6;  % (m) Earth radius
    e = 0.081819;  % (-) Earth orbit eccentricity
    
    % Conversion of a position from WGS84 to ECEF representation
    rECEF = zeros(3, 1);
    
    rWGS(1) = deg2rad(rWGS(1));
    rWGS(2) = deg2rad(rWGS(2));
    
    % See slide 6 from "Hilfsblatt"
    rECEF(1) = (a + rWGS(3)) * cos(rWGS(1)) * cos(rWGS(2));
    rECEF(2) = (a + rWGS(3)) * cos(rWGS(1)) * sin(rWGS(2));
    rECEF(3) = (a * (1 - e^2) + rWGS(3)) * sin(rWGS(1));

end

