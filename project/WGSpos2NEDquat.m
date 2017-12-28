function NEDquat = WGSpos2NEDquat(WGSpos)
    %    
    % rWGS: (3,1) vector containing coordinates in WGS coordinates
    %       (phi, lambda, h)'

    % Abbreviations
    sp = sin(WGSpos(1));  % sin of phi
    cp = cos(WGSpos(1));  % cos of phi
    sl = sin(WGSpos(2));  % sin of lambda
    cl = cos(WGSpos(2));  % cos of lambda
   
    % Attitude of an NED frame of which the origin is
    % located at the given WGS84 position
    
    R = eye(3);
    
    % See slide 8 from "Hilfsblatt"
    R = [-sp*cl, -sl, -cp*cl;
         -sp*sl,  cl, -cp*sl;
              cp,  0,    -sp];
    
    NEDquat = DCM2Quat(R);

end

