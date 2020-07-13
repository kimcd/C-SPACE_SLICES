function H_point2W = point_frame(H_sphere2W, q_point2sphere, q_nxtpoint2sphere)
%% return H_point2W, the homogenous transformation matrix for a point 
%   
%   z-axis points outward
%   x-axis point in direction of next point
%   good description: https://stackoverflow.com/questions/9605556/how-to-project-a-point-onto-a-plane-in-3d


    q_point2sphere = reshape(q_point2sphere,3,1);
    q_nxtpoint2sphere = reshape(q_nxtpoint2sphere,3,1); 
    
    q_point2W = H_sphere2W * [q_point2sphere; 1]; 
    q_nxtpoint2W = H_sphere2W * [q_nxtpoint2sphere;1]; 
    
    q_point2W = q_point2W(1:3);
    q_nxtpoint2W = q_nxtpoint2W(1:3);
    
    z_unit = (q_point2W - H_sphere2W(1:3,4))/...
        norm(q_point2W - H_sphere2W(1:3,4)); %z-axis unit normal radially outward from sphere center through point
    
    dist_v = q_nxtpoint2W - q_point2W; % vector from point to next point
    proj_dist_v = dot(z_unit, dist_v); % length of projected vector onto unit norm
    
    projected_point2W = q_nxtpoint2W - proj_dist_v * z_unit; % projection of point onto the plane
    
    x_unit = (projected_point2W - q_point2W)/...
        norm(projected_point2W - q_point2W); % x-axis unit normal
    
    y_unit = cross(z_unit, x_unit); 

    H_point2W = [x_unit y_unit z_unit...
        [q_point2W(1); q_point2W(2);...
        q_point2W(3)]; [0 0 0 1]]; 
        
end

