function [ initialClosed, maxX, maxY ] = AstarMap( map )
% function to create the A star map once per simulation
ppa = 1; % the points per area square required (1 for 1 node per 1x1 square of map)
map1 = map; 
for i= 1:1:2
    for j = 1:1:length(map)
        if mod(map(j,i),2) == 0
            map1(j,i) = map(j,i) / 2;
        else 
            map1(j,i) = (map(j,i) + 1)/2 ; 
        end 
    end 
end
   
maxX = max(map1(:,1));
maxY = max(map1(:,2)); 

% creates a rectangular grid with values of -1 (obstacles) 
mapAstar = -1*(ones(maxX, maxY)); 

% find the nodes inside of the polygon
nodes = polygrid(map1(:,1), map1(:,2), ppa); 

% nodesOdd = mod(nodes1,2) == 1;
% nodesEven = mod(nodes1,2) == 0; 
% 
% nodesOdd1 = (nodesOdd -1) /2; 
% nodesEven1 = nodesEven /2; 
% 
% nodes = [ nodesOdd1; nodesEven1] ; 

% change the value of any nodes inside the map to 2 (potential node)
% --- NB -> Need to subtract 1 from optimal path answers
for i = 1:length(nodes)
    mapAstar(nodes(i,1), nodes(i,2)) = 2; 
end 

% create closed list
initialClosed = []; 

% put nodes outside of the wall on closed list
count = 1;
for i = 1:maxX
    for j = 1:maxY
        if (mapAstar(i,j) == -1)
            initialClosed(count, 1) = i;
            initialClosed(count, 2) = j;
            count = count + 1; 
        end 
    end 
end 