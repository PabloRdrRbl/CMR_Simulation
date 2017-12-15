% Localization algorithm for a Differential Drive Mobile Robot based
% on the Extended Kalman Filter (EKF)

function filter = filter_ddrive_ekfslam()
    filter = filter_slam2d(@filterStep); % reuse drawing function from generic localization2d block     
    filter.depends = {'sensors/odometer', 'sensors/landmark_detector', 'environment/landmarks', 'platform'};
    
    filter.default_initialPose = [0 0 0]';
    filter.default_initialPoseCov = zeros(3, 3);
    
    filter.default_odometryError = 5 * pi / 180;    % sigma of assumed odometer uncertainty in rad/s
    filter.default_wheelRadiusError = 1e-3;         % sigma of assumed wheel diameter uncertainty in m
    filter.default_wheelDistanceError = 5e-3;       % sigma of assumed wheel distance uncertainty in m
    filter.default_bearingError = 5 * pi / 180;     % sigma of assumed bearing error in rad
    filter.default_rangeError = 2 / 100;            % sigma of assumed range error in percent of the aquired distance value
    
    filter.default_useBearing = true;               % enable update from bearing measurements
    filter.default_useRange = true;                 % enable update from range measurements 

    filter.default_useNumericPrediction = false;    % do mean prediction using ode45
    filter.default_useExactDiscretization = false;  % if true: do exact discretization, followed by linearization for covariance propagation
                                                    % if false (the default), linearize first and then discretize using the matrix exponential 
    
	
    filter.figures(end + 1).name = 'Covariance Plot';
    filter.figures(end).icon = fullfile(fileparts(mfilename('fullpath')), 'covariance_icon.png');
    filter.figures(end).init = @createCovFigure;
    filter.figures(end).draw = @updateCovFigure;
    
    filter.figures(end + 1).name = 'Error Plots';
    %filter.figures(end).icon = fullfile(fileparts(mfilename('fullpath')), 'covariance_icon.png');
    filter.figures(end).init = @createErrorFigure;
    filter.figures(end).drawLog = @updateErrorFigureLog;

    
    function [f, diag] = createCovFigure(block, blockName)       
        f = figure('Name', ['Covariance for ' blockName], 'NumberTitle', 'off');
        diag.ax = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', 'DataAspectRatio', [1 1 1], 'YDir', 'reverse');
        colormap(diag.ax, 'gray');
        colorbar('peer', diag.ax);
        diag.hCovImg = image('Parent', diag.ax, 'CData', [], 'CDataMapping', 'scaled');
    end
    function diag = updateCovFigure(block, f, diag, out, debug, state, varargin)
        set(diag.hCovImg, 'CData', state.cov);
    end	

    function [f, diag] = createErrorFigure(block, blockName)       
        f = figure('Name', ['Error plots for ' blockName], 'NumberTitle', 'off');
        diag.axErrX = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'OuterPosition', [0, 3/4, 1, 1/4]);
        %diag.hSigmaX = patch('Parent', diag.axErrX, 'XData', [], 'YData', [], 'FaceColor', [1, 0, 0], 'FaceAlpha', 0.5, 'LineStyle', 'none');
        diag.hSigmaX = [line('Parent', diag.axErrX, 'XData', [], 'YData', [], 'Color', [1 0 0], 'LineStyle', '--'), ...
                        line('Parent', diag.axErrX, 'XData', [], 'YData', [], 'Color', [1 0 0], 'LineStyle', '--')];
        diag.hErrX = line('Parent', diag.axErrX, 'XData', [], 'YData', [], 'Color', [0 0 1]);
        ylabel('error_X in m');
        diag.axErrY = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'OuterPosition', [0, 2/4, 1, 1/4]);
        diag.hSigmaY = [line('Parent', diag.axErrY, 'XData', [], 'YData', [], 'Color', [1 0 0], 'LineStyle', '--'), ...
                        line('Parent', diag.axErrY, 'XData', [], 'YData', [], 'Color', [1 0 0], 'LineStyle', '--')];
        diag.hErrY = line('Parent', diag.axErrY, 'XData', [], 'YData', [], 'Color', [0 0 1]);
        ylabel('error_Y in m');
        diag.axErrPhi = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'OuterPosition', [0, 1/4, 1, 1/4]);
        diag.hSigmaPhi = [line('Parent', diag.axErrPhi, 'XData', [], 'YData', [], 'Color', [1 0 0], 'LineStyle', '--'), ...
                          line('Parent', diag.axErrPhi, 'XData', [], 'YData', [], 'Color', [1 0 0], 'LineStyle', '--')];
        diag.hErrPhi = line('Parent', diag.axErrPhi, 'XData', [], 'YData', [], 'Color', [0 0 1]);
        ylabel('error_Phi in °');
        diag.axNees = axes('Parent', f, 'XGrid', 'on', 'YGrid', 'on', 'OuterPosition', [0, 0, 1, 1/4]);
        ylabel('NEES');
        xlabel('Time [s]');        
    end

    function diag = updateErrorFigureLog(block, f, diag, logPos, t, out, debug, state)
        t = t(1:logPos);
        [xhatlog, yhatlog, phihatlog] = arrayfun(@(s)deal(s.x(1), s.x(2), s.x(3)), state(1:logPos));
        [xlog, ylog, philog] = arrayfun(@(s)deal(s.realPose(1), s.realPose(2), s.realPose(3)), state(1:logPos));
        sigmaX = sqrt(arrayfun(@(s)s.cov(1, 1), state(1:logPos)));        
        set(diag.hSigmaX(1), 'XData', t, 'YData', 3 * sigmaX);
        set(diag.hSigmaX(2), 'XData', t, 'YData', -3 * sigmaX);
        set(diag.hErrX, 'XData', t, 'YData', xhatlog - xlog);                
        sigmaY = sqrt(arrayfun(@(s)s.cov(2, 2), state(1:logPos)));        
        set(diag.hSigmaY(1), 'XData', t, 'YData', 3 * sigmaY);
        set(diag.hSigmaY(2), 'XData', t, 'YData', -3 * sigmaY);
        set(diag.hErrY, 'XData', t, 'YData', yhatlog - ylog);        
        sigmaPhi = sqrt(arrayfun(@(s)s.cov(3, 3), state(1:logPos)));        
        set(diag.hSigmaPhi(1), 'XData', t, 'YData', 3 * sigmaPhi * 180 / pi);
        set(diag.hSigmaPhi(2), 'XData', t, 'YData', -3 * sigmaPhi * 180 / pi);
        set(diag.hErrPhi, 'XData', t, 'YData', (phihatlog - philog) * 180 / pi);        
    end	
end

function [state, out, debugOut] = filterStep(block, t, state, odometer, sensor, ~, realPose, varargin)
    debugOut = [];

    if isempty(state)            
        % Initialization
        state.x = block.initialPose(:);
        state.cov = block.initialPoseCov;
        state.lastInput = [0; 0]; % initial speed is zero
        state.t = 0;

        state.features = [];
    end

    % use shorter symbols 
    x = state.x;
    P = state.cov;
    u = state.lastInput;
    tNow = state.t;

    iPredict = 1;
    iUpdate = 1;

    while tNow < t
        % determine, which measurement to proceed next
        if iUpdate <= length(sensor)
            tNextUpdate = sensor(iUpdate).t;
        else tNextUpdate = t;
        end            
        while tNow < tNextUpdate
            if iPredict <= length(odometer)
                tNext = odometer(iPredict).t;
                if tNext <= tNextUpdate
                    [x, P] = doPrediction(x, P, u, tNext - tNow);
                    tNow = tNext;           
                    u = odometer(iPredict).data(:);
                    iPredict = iPredict + 1;
                else break;
                end
            else break;    
            end
        end

        if tNow < tNextUpdate
            [x, P] = doPrediction(x, P, u, tNextUpdate - tNow);
            tNow = tNextUpdate;
        end

        if iUpdate <= length(sensor)                
            [x, P, state.features] = doUpdate(x, P, state.features, sensor(iUpdate).data);
            iUpdate = iUpdate + 1;
        end
    end

    % put short-named intermediate variables back into the filter state
    state.x = x;
    state.pose = x(1:3);
    state.cov = P;
    state.lastInput = u;
    state.t = tNow;
    state.realPose = realPose(end).data;
    

    % the output of the localization filter is the estimated state (=pose) vector
    out.pose = [state.x(1:2)', mod(state.x(3) + pi, 2 * pi) - pi];
    out.cov = state.cov(1:3, 1:3);
    out.featurePositions = reshape(state.x(4:end), 2, [])';
    out.landmarkIds = state.features;
    autoCorrs = diag(state.cov(4:end, 4:end));
    crossCorrs = diag(state.cov(4:end, 4:end), 1);
    out.featureCovariances = [autoCorrs(1:2:end), crossCorrs(1:2:end), autoCorrs(2:2:end)]; 

    function [x, P] = doPrediction(x, P, u, T)        
        % Implementation of the prediction step

        R_R = block.wheelRadius(1);
        R_L = block.wheelRadius(end);

        a = block.wheelDistance / 2;
        
        dtheta_R = u(1);
        dtheta_L = u(2);
        
        % Prediction step
        
        % To simplify the equations, we write the inputs in terms
        % of v and omega
        v = (R_R * dtheta_R + R_L * dtheta_L) / 2;
        omega = (R_R * dtheta_R - R_L * dtheta_L) / (2 * a);
        
        % Some abreviations for the system matrix
        phi = x(3);
        sk = sin(phi);
        ck = cos(phi);
        skp1 = sin(phi + T * omega);
        ckp1 = cos(phi + T * omega);
        
        % do state prediction
        if block.useNumericPrediction
            % Use numeric integration
            % This is applicable to any nonlinear system            
            options = odeset('RelTol', 10^-13, 'AbsTol', eps);        
            [~, X] = ode45(@(t, x)vomega_model(x, v, omega), [0, T], x, options);
            x(1:3) = (X(end, :)).';                
        else
            % Exact (analytical) solution
            if abs(omega) < 1e-12
                x(1:3) = x(1:3) + [v * T * ck; ...
                                   v * T * sk; ...
                                            0];
            else
                x(1:3) = x(1:3) + [v / omega * (skp1 - sk); ...
                                   v / omega * (ck - ckp1); ...
                                   T * omega];
            end  
        end

        % do covariance prediction

        % Input error covariance
        N = diag([block.odometryError^2, block.odometryError^2]);

        if ~block.useExactDiscretization
            % linearize first...
            A = [0, 0, -v * sk; ...
                 0, 0,  v * ck; ...
                 0, 0,       0];
            
            B = [ R_R / 2 * ck,  R_L / 2 * ck; ...
                  R_R / 2 * sk,  R_L / 2 * sk; ...
                 R_R / (2 * a), -R_L / (2 * a)];
             
            % ... then discretize using the matrix exponential
            F = expm(A * T);
            
            I = eye(size(F));
            S = T * I;
            
            i = 2;
            while true
                D = T^i / factorial(i) * A^(i - 1);
                S = S + D;
                i = i + 1;
                if all(abs(D(:)) <= eps); break; end
            end
            H = S * B;
            
            F = blkdiag(F, eye(size(P, 1) - 3));
            P = F * P * F.';
            P(1:3, 1:3) = P(1:3, 1:3) + H * N * H.';
        else
            % discretize first (only applicable, if ODEs can be solved
            % analytically), then linearize the discrete model

            if abs(omega) < 1e-12
                A_dis = [1, 0, -v * T * sk; ...
                         0, 1, v * T * ck; ...
                         0, 0, 1];
                B_dis = [R_R * T / 2 * ck, R_L * T / 2 * ck; ...
                         R_R * T / 2 * sk, R_L * T / 2 * ck; ...
                                        0,                0];
            else
                A_dis = [1, 0, v / omega * (ckp1 - ck); ...
                         0, 1, v / omega * (skp1 - sk); ...
                         0, 0,                       1];
                B_dis = [R_R / (2 * omega) * (v * T / a * ckp1 + R_L * dtheta_L / (a * omega) * (sk - skp1)), ...
                                -R_L / (2 * omega) * (v * T / a * ckp1 + R_R * dtheta_R / (a * omega) * (sk - skp1)); ...
                         R_R / (2 * omega) * (v * T / a * skp1 - R_L * dtheta_L / (a * omega) * (ck - ckp1)), ...
                                -R_L / (2 * omega) * (v * T / a * skp1 - R_R * dtheta_R / (a * omega) * (ck - ckp1)); ...
                         R_R * T / (2 * a), -R_L * T / (2 * a)];
            end
            
            A_dis = blkdiag(A_dis, eye(size(P, 1) - 3));
            P = A_dis * P * A_dis.';            
            P(1:3, 1:3) = P(1:3, 1:3) + B_dis * N * B_dis.';
        end
    end

    function [x, P, features] = doUpdate(x, P, features, meas)
        % Implementation of the update step
        visIdx = meas.lmIds; % assume we can associate each measurement with a known landmark
        if isempty(visIdx); return; end

        % determine, which measurements belong to features already part
        % of the state vector and which we see for the first time (map
        % management)
        [~, fidx_old, midx_old] = intersect(features, visIdx);
        [~, midx_new] = setdiff(1:length(visIdx), midx_old);

        % interpretation of the index arrays
        % midx_old -> indices of measurements of known landmarks...
        % fidx_old -> ...and their associated indices in the state vector
        % midx_new -> indices of measurments of landmarks we see for the first time

        if ~isempty(midx_new)
            % feature initialization for first-time measurements. 
            % Regardless of the block.useBearing/block.useRange
            % settings, we always have to use both measurements here to
            % uniquely determine the initial feature estimate!

            % length of state vector before adding new features
            len = length(x);

            % range & bearing measurements as row vectors
            bearings = meas.bearing(midx_new)';
            ranges = meas.range(midx_new)';

            pose = x(1:3);
            c = cos(pose(3) + bearings);
            s = sin(pose(3) + bearings);
            
            % TODO: implement the landmark initialization here
            
                            
            features = [features; visIdx(midx_new)];
        end

        if ~isempty(midx_old)
            % process measurements of tracked landmarks

            % prepare innovation vector, output jacobi matrix and measurement noise covariance
            delta_y = zeros(0, 1);
            C = zeros(0, length(x));
            W = zeros(0, 0);

            % indices of the feature coordinates in the state vector
            x_idx = 2 + 2 * fidx_old';
            y_idx = 3 + 2 * fidx_old';

            % TODO: implement the update step here
            
            
            % compute Kalman gain matrix
            K = P * C' / (C * P * C' + W);

            % EKF update
            x = x + K * delta_y;
            P = (eye(size(P)) - K * C) * P;
        end
    end
end

function dx = vomega_model(x, v, omega)
    phi = x(3);
    dx = [v * cos(phi); ...
          v * sin(phi); ...
          omega];
end

