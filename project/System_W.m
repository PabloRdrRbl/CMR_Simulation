% Parameters
% WGS_pos - GPS measurement (in WGS84 format)
% NED_sigma - GPS uncertainty information expressed in {N}
function [WE] = System_W (WGS_pos, NED_sigma)    
    % TODO: implement the measurement covariance matrix
    % Note: In general, this is not a diagonal matrix!
    
    NEDquat = WGSpos2NEDquat(WGS_pos);
    R_EN = Quat2DCM(NEDquat);
    
    sg1 = NED_sigma(1);
    sg2 = NED_sigma(2);
    sg3 = NED_sigma(3);
    
    WE = eye(3);
    
    WE(1,1) = sg1*sg1;
    WE(2,2) = sg2*sg2;
    WE(3,3) = sg3*sg3;
    
    WE = abs(R_EN' * WE * R_EN);
    
end