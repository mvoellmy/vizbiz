function plotMatches(matches, query_keypoints, database_keypoints)
% Plots matches between matched query and database keypoints.
% 
% Input:
%  - matches(1xN) : match indeces
%  - query_keypoints(2xN) : 2D query keypoints, each [v,u]
%  - database_keypoints(2xN) : 2D database keypoints, each [v,u]
%
% Output: none

[~, query_indices, match_indices] = find(matches);

x_from = query_keypoints(1, query_indices);
x_to = database_keypoints(1, match_indices);
y_from = query_keypoints(2, query_indices);
y_to = database_keypoints(2, match_indices);
plot([y_from; y_to], [x_from; x_to], 'g-', 'Linewidth', 1);

end
