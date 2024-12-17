
% Initialization of environment and parameters
global n; global grid_size; global grid_map; global goal_point; global obstacles; global obstacle_id; global ax; global current_position; global stop_sign; global if_started;
n = 30; % The size of gird space
grid_size = 30; % 
grid_map = zeros(n); % 0 for empty, 1 for obstacles, 2 for goal
start_point = [1, 1]; % start
current_position = start_point;
grid_map(current_position(1),current_position(2)) = 2;
goal_point = [29, 22]; % goal
grid_map(goal_point(1),goal_point(2)) = 3;
obstacles = {}; 
stop_sign = false;
if_started = false;


global isDrawing; global enableDrawing; global enableErasing;
isDrawing = false; 
enableDrawing = false;
enableErasing = false;


% Set parameters
global discount; global theta; global actions; global action_cost;
theta = 0.1; % Convergence threshold
actions = [-1, 0; 1, 0; 0, -1; 0, 1; -1, -1; -1, 1; 1, 1; 1, -1; 0, 0]; % 9 actions 
action_cost = [1, 1, 1, 1, 1.4, 1.4, 1.4, 1.4, 0];

% Initialize value function
global V;
global policy;
global reward;
V = zeros(n,n); 
reward = 100;

policy = zeros(n, n); % Optimal policy

% Create GUI
fig = figure('Name', 'Value Iteration and Path Planning', 'NumberTitle', 'off', 'Position', [50, 50, 800, 800]);
ax = axes('Parent', fig, 'Position', [0.18, 0.15, 0.80, 0.80]); 
axis off; 
hold on;

% Draw initial grid space
draw_grid();

% Set buttons and recall

uicontrol('Style', 'pushbutton', 'String', 'Start/Continue', 'Position', [10, 60, 120, 30], 'Callback', @start_value_iteration);
uicontrol('Style', 'pushbutton', 'String', 'Eraser', 'Position', [10, 140, 120, 30], 'Callback', @enable_erasing);
uicontrol('Style', 'pushbutton', 'String', 'Add Obstacles', 'Position', [10, 100, 120, 30], 'Callback', @enable_drawing);

set(fig, 'WindowButtonDownFcn', @startDrawing);   
set(fig, 'WindowButtonUpFcn', @stopDrawing);      
set(fig, 'WindowButtonMotionFcn', @drawing);      


% 更新GUI数据
% handles.grid_map = grid_map;
% handles.start_point = start_point;
% handles.goal_point = goal_point;
% handles.V = V;
% handles.policy = policy;
% handles.fig = fig;
% guidata(fig, handles);

% --------------------- Internal Functions-----------------------
%% 

% Draw grids
function draw_grid()
    global n; global grid_size; global grid_map; global current_position; global goal_point; global obstacles; global obstacle_id; global ax; global policy; global actions;
    cla(ax);
    for i = 1:n
        for j = 1:n
            color = [1, 1, 1]; % White
            if grid_map(i,j) == 1 % Obstacles
                color = [0, 0, 0]; % Black
            elseif grid_map(i,j) == 2 % Robot position
                color = [0, 1, 0]; % Green
            elseif grid_map(i,j) == 3 % Goal
                color = [1, 0, 0]; % Red
            end
            rectangle('Position', [(i-1)*grid_size, (j-1)*grid_size, grid_size, grid_size], ...
                'FaceColor', color, 'EdgeColor', 'k');
        end
    end
    % Draw obstacles, start point, goal point
    rectangle('Position', [(current_position(1)-1)*grid_size, (current_position(2)-1)*grid_size, grid_size, grid_size], ...
        'FaceColor', [0, 1, 0], 'EdgeColor', 'k');
    rectangle('Position', [(goal_point(1)-1)*grid_size, (goal_point(2)-1)*grid_size, grid_size, grid_size], ...
        'FaceColor', [1, 0, 0], 'EdgeColor', 'k');
    
    % Draw the path
    if_end = 1;
    transitional_position = current_position;
    if ~policy(current_position(1),current_position(2)); return; end
    while if_end
        
        movement=actions(policy(transitional_position(1),transitional_position(2)),:);
        next_position = transitional_position + movement;
        if all(movement == [0,0]) || (grid_map(next_position(1),next_position(2)) == 3)  % End path drawing if the action is [0,0] or the robot reaches the goal
            if_end = 0;
            continue;
        end
        rectangle('Position', [(next_position(1)-1)*grid_size, (next_position(2)-1)*grid_size, grid_size, grid_size], 'FaceColor', [0,1,1], 'EdgeColor', 'k');
        transitional_position = next_position;
    end

end
%% 

% Value iteration function
function start_value_iteration(~, ~)
    global V; global n;
    global policy;
    global theta; global actions; global action_cost; global n; global grid_map; global stop_sign; global goal_point; global reward; global if_started;
    if_started = true;
    % Initialize
    delta1 = Inf;
    delta2 = Inf;
    V = zeros(n,n); 
    V(goal_point(1),goal_point(2)) = reward; % Rewards for reaching the goal. It must be large enough or the robot will not move.
    V_new=V;
    last_iteration = 1; % To make sure the algorithm works, we need several more iterations after V and policy converge 

    while last_iteration > 0
        delta1 = 0;
        delta2 = 0;
        
        for i = 1:n
            for j = 1:n
                if grid_map(i, j) == 1; V_new(i,j) = 0;  continue;  end % skip obstacles
                if grid_map(i, j) == 3;   continue;  end % skip goal point      
                v = V(i,j);
                max_val = -Inf;
                best_action = 0;
                
                % Evaluation
                for action_idx = 1:size(actions, 1)
                    new_i = i + actions(action_idx, 1);
                    new_j = j + actions(action_idx, 2);

                    if new_i >= 1 && new_i <= n && new_j >= 1 && new_j <= n  && grid_map(new_i, new_j) ~= 1
                        % Detour. When there is an obstacle directly in front of the robot, it cannot go diagonally through it
                        if i < n && grid_map(i+1,j) == 1 && actions(action_idx, 1) == 1; continue; end
                        if i > 1 && grid_map(i-1,j) == 1 && actions(action_idx, 1) == -1; continue; end
                        if j < n && grid_map(i,j+1) == 1 && actions(action_idx, 2) == 1; continue; end
                        if j > 1 && grid_map(i,j-1) == 1 && actions(action_idx, 2) == -1; continue; end
                        value = -action_cost(action_idx) + V(new_i, new_j);
                        if value > max_val
                            max_val = value;
                            best_action = action_idx;
                        end
                    end
                end
                % Update
                V_new(i, j) = max_val;
                
                delta1 = max(delta1, abs(v - V_new(i,j)));
                delta2 = max(delta2,abs(best_action-policy(i,j)));
                policy(i, j) = best_action;

            end
        end
        V=V_new;
        %pause(0.1);
        %flipud(V')
        % To check whether V and policy converge. If so, we do several more iterations and finish                
        if delta1 <= theta && delta2 <= theta
           last_iteration = last_iteration -1;
        end

    end

    draw_grid();
    stop_sign = false;
    go_ahead();
end
%% 

% 
function draw_policy()
    global policy; global n; global grid_size; global grid_map; global obstacles; global obstacle_id; global ax; global actions;
    
    


    for i = 1:n
        for j = 1:n
            if grid_map(i,j) == 1
                continue; % 跳过障碍物
            end
            X=(i-0.5)*grid_size;
            Y=(j-0.5)*grid_size;

            switch policy(i, j)
                case 1 % left
                    quiver(X, Y, -1, 0, 5, 'MaxHeadSize', 1, 'LineWidth', 2, 'Color', 'blue');
                case 2 % right
                    quiver(X, Y, 1, 0, 5, 'MaxHeadSize', 1, 'LineWidth', 2, 'Color', 'blue');
                case 3 % down
                    quiver(X, Y, 0, -1, 5, 'MaxHeadSize', 1, 'LineWidth', 2, 'Color', 'blue');
                case 4 % up
                    quiver(X, Y, 0, 1, 5, 'MaxHeadSize', 1, 'LineWidth', 2, 'Color', 'blue');
                case 5 % l d
                    quiver(X, Y, -1, -1, 5, 'MaxHeadSize', 1, 'LineWidth', 2, 'Color', 'blue');
                case 6 % l u
                    quiver(X, Y, -1, 1, 5, 'MaxHeadSize', 1, 'LineWidth', 2, 'Color', 'blue');
                case 7 % r u
                    quiver(X, Y, 1, 1, 5, 'MaxHeadSize', 1, 'LineWidth', 2, 'Color', 'blue');
                case 8 % r d
                    quiver(X, Y, 1, -1, 5, 'MaxHeadSize', 1, 'LineWidth', 2, 'Color', 'blue');

            end
        end
    end
end


%% 
    % 
    function startDrawing(~, ~)
        global isDrawing; global enableDrawing; global enableErasing; global ax; global grid_size; global stop_sign;
       
        if enableDrawing || enableErasing; isDrawing = true; stop_sign = true; end
        % Get current position of the cursor
        mousePos = get(ax, 'CurrentPoint');
        pos = [ceil(mousePos(1,1)/grid_size),ceil(mousePos(1,2)/grid_size) ];
        if enableDrawing; add_to_obstacles(pos); return; end
        if enableErasing; delete_obstacles(pos); return; end

        hold on;
    end
%% 

    % 
    function drawing(~, ~)
        global isDrawing; global ax; global grid_size; global enableDrawing; global enableErasing;
        if isDrawing
            % % Get current position of the cursor
            mousePos = get(ax, 'CurrentPoint');
            
            pos = [ceil(mousePos(1,1)/grid_size),ceil(mousePos(1,2)/grid_size)];
            if enableDrawing; add_to_obstacles(pos); return; end
            if enableErasing; delete_obstacles(pos); return; end

        end
    end
%% 

    % 
    function stopDrawing(~, ~)
        global isDrawing; global enableDrawing; global enableErasing; global if_started;
        isDrawing = false;  % Stop drawing
        enableDrawing = false;
        enableErasing = false;
        if if_started; start_value_iteration(); end
    end

%% 

function enable_drawing(~, ~)
    global enableDrawing; global enableErasing;
    enableErasing = false;
    enableDrawing = true;
end

%% 
function add_to_obstacles(pos)
    global grid_map; global grid_size;
    if grid_map(pos(1),pos(2)) == 0
        grid_map(pos(1),pos(2)) = 1;
        rectangle('Position', [(pos(1)-1)*grid_size, (pos(2)-1)*grid_size, grid_size, grid_size], 'FaceColor', [0.8, 0.6, 0], 'EdgeColor', 'k');
    end
end

%% 
function enable_erasing(~,~)
    global enableDrawing; global enableErasing;
    enableErasing = true;
    enableDrawing = false;
end

%% 
% To delete obstacles. Remove obstacles by moving the mouse. The size of the eraser is 3*3.
function delete_obstacles(pos)    
    for i = pos(1)-1 : pos(1)+1
        for j = pos(2)-1 : pos(2)+1
            dlt(i,j);
        end
    end
end
% The auxiliary function to process each grid
function dlt(i,j)
    global grid_map; global grid_size; global n;
    if i < 1 || i > n || j < 1 || j > n; return; end
    if grid_map(i,j) == 1
        grid_map(i,j) = 0;
        rectangle('Position', [(i-1)*grid_size, (j-1)*grid_size, grid_size, grid_size], 'FaceColor', [1, 1, 1], 'EdgeColor', 'k');
    end
end
    
%% 

function go_ahead()
    global stop_sign; global policy; global grid_size; global grid_map; global current_position; global actions;
    while true
        pause(0.25);
        if stop_sign; return; end
        %current_position
        if ~policy(current_position(1),current_position(2)) || policy(current_position(1),current_position(2)) == 9; return; end
        movement = actions(policy(current_position(1),current_position(2)),:);
        next_position = current_position + movement;
        rectangle('Position', [(current_position(1)-1)*grid_size, (current_position(2)-1)*grid_size, grid_size, grid_size], 'FaceColor', [1, 1, 1], 'EdgeColor', 'k');
        grid_map(current_position(1),current_position(2)) = 0;
        rectangle('Position', [(next_position(1)-1)*grid_size, (next_position(2)-1)*grid_size, grid_size, grid_size], 'FaceColor', [0, 1, 0], 'EdgeColor', 'k');
        grid_map(next_position(1),next_position(2)) = 2;
        current_position = next_position;
        
    end

end
