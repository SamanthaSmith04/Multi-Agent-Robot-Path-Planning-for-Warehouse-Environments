function [simulated_robots] = FARMultiRobotController(G,robot_array,k,timeStep)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
epsilon = 0.00001; %precision for some parts

cell_dim = size(robot_array);
num_robots = cell_dim(2);

%Queue to Store Robots that need to reserve paths and plan
waiting_queue = [1:1:num_robots];

%Zero is a space holder, so there is no guarantee tha a node has been
%reserved
all_reserved_nodes = zeros(1,num_robots*k);
can_remove_reservation = zeros(1,num_robots);
currently_occuppied = zeros(1,num_robots);

%Completed - If completed then the robot can be skipped
running = ones(1,num_robots);

%As long as at least one robot is running:
currentTime = 0.0;

%init steps for each path
for i = 1:num_robots
    robot_array(i).pose = [robot_array(i).start 0]; % \in R^(1x3)
    robot_array(i).currentPose = [robot_array(i).start 0];
    robot_array(i).time = [currentTime];
    robot_array(i).plannedTime = currentTime; % Time that robot is planned upuntil
    robot_array(i).phi = [0,0];
    robot_array(i).reserved_node = zeros(1,k); % Nodes Currently Reserved by the robot
end

while sum(running)
    %Removed Reservations if possible
    for i=1:1:num_robots
        if(can_remove_reservation(i))
            if(~running(i))
                nnzm = nnz(robot_array(i).reserved_node);
                disp(i);
                disp(nnzm);
                currently_occuppied(i) = robot_array(i).reserved_node(nnzm);
            else
                currently_occuppied(i) = robot_array(i).reserved_node(k);
            end
            robot_array(i).reserved_node = zeros(1,k);
            all_reserved_nodes((i-1)*k+1:i*k) = robot_array(i).reserved_node;
        end
    end
    can_remove_reservation = zeros(1,num_robots); 
    
    %Break Queue into 3 parts to esnure fairness in reserving nodes
    no_movement_queue = [];
    wait_for_completion_queue = [];
    just_started_queue = [];
    
    for i=waiting_queue
        bot = robot_array(i);
        
        completed = 0; %Indicates if a robot has compelted its path
        % Attempt To Reserve Next k Nodes
        if currentTime > bot.plannedTime && running(i)
            
            %Compute how many nodes have been reserved so far...
            num_current = nnz(bot.reserved_node);
            for j=(num_current+1):1:k
                num_remain = size(bot.node_path);
                num_remain = num_remain(2);
                if(num_remain == 0)
                    completed = 1;
                    break;
                end
                
                %Assume node remaining in the path
                next_node = bot.node_path(1);
                
                % It means the last node will be removed if possible
                if(size(bot.node_path)==1)
                    completed = 1;
                end
                
                %If not reserved then I may reserve it
                if(~ismember(next_node,[all_reserved_nodes,currently_occuppied]))
                    bot.reserved_node(j) = next_node;
                    bot.node_path = bot.node_path(2:end); %% Node can be removed 
                else
                    if(completed)
                        completed = 0;
                    end
                    %Ensures that controller is aware that this still has 
                    %atleast one node to go
                    break;
                end
            end
            
            %First update global collection of reserved nodes
            all_reserved_nodes((i-1)*k+1:i*k) = bot.reserved_node;
            
            %If maximum allowed nodes have been reserved
            %Planning may occur
            if(completed || nnz(bot.reserved_node)==k)
                nnzNodes = nnz(bot.reserved_node);
                nodes = bot.reserved_node(1:nnzNodes);
                
                x_coo = G.Nodes.X(nodes);
                y_coo = G.Nodes.Y(nodes);
                road = [x_coo,y_coo];
                if completed
                   road = [road;bot.goal]; 
                   running(i) = 0;
                end

                startPose = bot.currentPose;
                
                [phi, time, robotPoses] = FARmotionPlan(road, startPose, timeStep);
                bot.phi = [bot.phi;phi];
                bot.time = [bot.time, time+bot.time(end)];
                bot.pose = [bot.pose;robotPoses];
                
                bot.plannedTime = bot.time(end);
                bot.currentPose = bot.pose(end,:);
                just_started_queue = [just_started_queue, i];
                %Even if it is 'completed' it will be added into the queue
                %So that nodes are removed from the reservation at the
                %right time
            else
                %If no movement, then the robot remains stationary 
                bot.phi = [bot.phi;0 0];
                bot.time = [bot.time, currentTime];
                bot.pose = [bot.pose;bot.pose(end,:)];
                bot.plannedTime = bot.time(end);
                bot.currentPose = bot.pose(end,:);
                no_movement_queue = [no_movement_queue, i];
            end
            
        else
            if (currentTime+timeStep >= bot.plannedTime)
                %If at the next time step the planned time will be up
                %then the node reservations held by the robot can be removed
                can_remove_reservation(i) = 1;
                if(running(i))
                    wait_for_completion_queue = [wait_for_completion_queue,i];
                end
            else
                wait_for_completion_queue = [wait_for_completion_queue,i];
            end
            %If its not done runnning it is complted and doesn't need to be
            %added back into the queue
        end
        robot_array(i) = bot;
        
    end
    
    waiting_queue = [no_movement_queue,wait_for_completion_queue,just_started_queue]; 
    currentTime = currentTime + timeStep;
end


% Record Time Of Completion For Each Robot
timeEnd = zeros(1,num_robots);
for i = 1:num_robots
     timeEnd(i) = robot_array(i).time(end);
     robot_array(i).endTime = timeEnd(i);
end

% Add To reach time End
fillTime = max(timeEnd);
for i = 1:1:num_robots
     additional = timeEnd(i) + timeStep;
     moreTime = [additional:timeStep:fillTime+epsilon];
     num_addition = length(moreTime);
     additionalPose = repmat(robot_array(i).pose(end,:),num_addition,1);
     additionalPhi = repmat([0,0],num_addition,1);
     robot_array(i).time = [robot_array(i).time,moreTime];
     robot_array(i).pose = [robot_array(i).pose;additionalPose];
     robot_array(i).phi = [robot_array(i).phi;additionalPhi];
end

simulated_robots = robot_array;
end

