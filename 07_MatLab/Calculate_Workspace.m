function [max_radius, optimal_z] = Calculate_Workspace(params)
% Calculate_Workspace.m
% Computes the maximum continuous circular reachable radius and its optimal Z-height
% based on the physical arm lengths and motor joint limits.

L1 = params.bicep_length;    
L2 = params.forearm_length;  
sp = params.f - params.e;    % Effective radius offset

th_min = params.theta_min;   
th_max = params.theta_max;   
phi_deg = params.phi;        

% Discretization (Optimized for speed and accuracy)
r_test = 0:2:400;            % Radius search [mm]
z_test = -600:5:-100;        % Z depth search below base [mm]
az_deg = 0:15:345;           % 24 sample points around each circular slice

max_r_overall = 0;
best_z = NaN;

for r = r_test
    found_valid_z_for_this_r = false;

    for z = z_test
        circle_reachable = true;

        for az = az_deg * (pi/180)
            x = r * cos(az);
            y = r * sin(az);

            % Check all 3 arms
            for i = 1:3
                phi = phi_deg(i) * (pi/180);

                % Rotate (x, y) into arm frame
                xL =  x * cos(phi) + y * sin(phi);
                yL = -x * sin(phi) + y * cos(phi);

                rem_sq = L2^2 - yL^2;
                if rem_sq <= 0
                    circle_reachable = false; 
                    break; 
                end

                dx = xL - sp;
                E = -2 * L1 * dx;
                F =  2 * L1 * z;
                G = L1^2 + dx^2 + z^2 - rem_sq;

                qa = G - E;
                qb = 2 * F;
                qc = G + E;
                disc = qb^2 - 4 * qa * qc;

                if disc < 0
                    circle_reachable = false; 
                    break; 
                end

                % Two solutions for theta
                t1 = (-qb + sqrt(disc)) / (2 * qa);
                t2 = (-qb - sqrt(disc)) / (2 * qa);

                sol1 = 2 * atan(t1) * 180 / pi;
                sol2 = 2 * atan(t2) * 180 / pi;

                valid1 = (sol1 >= th_min && sol1 <= th_max);
                valid2 = (sol2 >= th_min && sol2 <= th_max);

                if ~(valid1 || valid2)
                    circle_reachable = false;
                    break;
                end
            end

            if ~circle_reachable
                break;
            end
        end

        if circle_reachable
            found_valid_z_for_this_r = true;
            if r > max_r_overall
                max_r_overall = r;
                best_z = z;
            end
            break; 
        end
    end

    if ~found_valid_z_for_this_r && r > 10
        break; % Reached the absolute boundary edge of the workspace
    end
end

max_radius = max_r_overall;
optimal_z = best_z;
end