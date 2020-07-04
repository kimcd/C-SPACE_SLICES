function q_square = build_square(n_points, center, length, theta)
%% TODO
% send error if n_points < 2
%%
vert = ones(2,4); 
vert(:,1) = [-length/2; -length/2];
vert(:,2) = [length/2; -length/2];
vert(:,3) = [length/2; length/2];
vert(:,4) = [-length/2; length/2];

%%
n = linspace(0,1,n_points);
points = zeros(2,4*(n_points-1)); % preallocate
start_i = n_points-2; 
end_i = n_points-1; 
for i = 1:4
    edge = vert(:,i) + n(1:end-1).*(vert(:,mod(i,4)+1)-vert(:,i)); 
    points(:,i*end_i-start_i:i*end_i) = edge;
end

%%
H = [cos(theta) -sin(theta) center(1); sin(theta) cos(theta) center(2); 0 0 1]; 
q_square = [points; ones(1,size(points,2))]; 
q_square = H * q_square;
q_square = q_square(1:2,:);
%%
% vert1 = [center(1)-length/2; center(2)-length/2]
% vert2 = [center(1)+length/2; center(2)-length/2]
% vert3 = [center(1)+length/2; center(2)+length/2]
% vert4 = [center(1)-length/2; center(2)+length/2]
% 
%q_square = [(length/2)*vert1, vert2, vert3, vert4]; 
end
