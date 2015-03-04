function [ optimalPath ] = Astar( map, target, currPose )
%% A star algorithm to be used with particle filter navigating to target
% Requires input of: map, target node, estimation of current position
% Give output of path (made up of inputs for move and turn) 
% To start: Obstacle = -1, Robot = 1, Potential nodes = 2

ppa = 1; % the points per area square required (1 for 1 node per 1x1 square of map)
maxX = max(map(:,1));
maxY = max(map(:,2)); 

% creates a rectangular grid with values of -1 (obstacles) 
mapAstar = -1*(ones(maxX, maxY)); 

% find the nodes inside of the polygon
nodes = polygrid(map(:,1), map(:,2), ppa); 

% change the value of any nodes inside the map to 2 (potential node)
% --- NB -> Need to subtract 1 from optimal path answers
for i = 1:length(nodes)
    mapAstar(nodes(i,1)+1, nodes(i,2)+1) = 2; 
end 

% create open list
open = []; 

% create closed list
closed = []; 

% put nodes outside of the wall on closed list
count = 1;
for i = 1:maxX
    for j = 1:maxY
        if (mapAstar(i,j) == -1)
            closed(count, 1) = i;
            closed(count, 2) = j;
            count = count + 1; 
        end 
    end 
end 

closedCount = size(closed, 1); 

% set the start node (current position) as the first node

xNode = currPose(1);
yNode = currPose(2); 

xTarget = target(1);
yTarget = target(2); 

openCount = 1;
pathCost = 0; 
goalDistance = distanceLine(xNode, yNode, xTarget, yTarget); 

open(openCount,:) = insertOpen(xNode, yNode, xNode, yNode, ...
    pathCost, goalDistance, goalDistance); 

open(openCount, 1) = 0; 

closedCount = closedCount + 1; 
closed(closedCount, 1) = xNode;
closed(closedCount, 2) = yNode; 
noPath = 1; 

%% Begin Algorithm 

while((xNode ~= xTarget || yNode ~= yTarget) && noPath == 1)
    expArray = expandArray(xNode, yNode, pathCost, xTarget, yTarget, ...
        closed, maxX, maxY);
    expCount = size (expArray, 1); 
    
    for i = 1:expCount 
        flag = 0; 
        for j = 1: openCount
            if (expArray(i,1) == open(j,2) && expArray(i,2) == open(j,3))
                open(j,8) = min(open(j,8), expArray(i,5)); 
                if open(j,8) == expArray(i,5)
                    
                    open(j,4) = xNode; 
                    open(j,5) = yNode; 
                    open(j,6) = expArray(i,3);
                    open(j,7) = expArray(i,4); 
                end 
                flag = 1; 
            end 
        end 
        
        if flag == 0 
            openCount = openCount + 1; 
            open(openCount,:) = insertOpen(expArray(i,1), expArray(i,2), xNode, yNode, expArray(i,3), expArray(i,4), expArray(i,5));
        end % end of add new node onto the open list
    end % end of while loop 
    
    % Find node with smallest f(n) 
    indexMinNode = minFn(open, openCount, xTarget, yTarget); 
    
    if (indexMinNode ~= -1)
        % X and Y of node are set to node with min fn
        xNode = open(indexMinNode, 2); 
        yNode = open(indexMinNode, 3); 
        % update cost of reaching parent node
        pathCost = open(indexMinNode, 6); 
        
        % move node to closed list 
        closedCount = closedCount + 1; 
        closed(closedCount,1) = xNode;
        closed(closedCount,2) = yNode; 
        
        open(indexMinNode, 1) = 0; 
        
    else 
        noPath = 0; % no path to target - exit loop
    end % end of lowest fn node check 
end % end of while loop 
    
% Find the path by redtracing the parent node back to the start node

i = size(closed,1); 
optimalPath = []; 
xVal = closed(i,1); 
yVal = closed(i,2); 

i = 1; 
optimalPath(i,1) = xVal;
optimalPath(i,2) = yVal; 
i = i+1;

if ((xVal == xTarget) && (yVal == yTarget))
    iNode = 0; 
    
    % traverse through open to determin parent nodes
    
    parentX = open(nodeIndex(open, xVal, yVal), 4); 
    parentY = open(nodeIndex(open, xVal, yVal), 5); 
    
    while( parentX ~= currPose(1) || parentY ~= currPose(2))
        optimalPath(i,1) = parentX;
        optimalPath(i,2) = parentY; 
        
        iNode = nodeIndex(open, parentX, parentY); 
        
        parentX = open(iNode, 4); 
        parentY = open(iNode, 5);
    
        i = i+1; 
    end 
end 

