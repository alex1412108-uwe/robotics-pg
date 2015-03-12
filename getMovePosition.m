function nextPosition = getMovePosition(optimalPath)
% Inputs: optimal Path -> [nx2] vector of X,Y coordinates
% Outputs: nextPosition -> [1x2] vector of furthest XY coordinate in line
% optimalPath = [0,0;1,1;2,2;1,2];
Move = 3;
if length(optimalPath) >= 5
   for i = 2:1:length(optimalPath)-1
        P_x(i) = optimalPath(i+1,1) - optimalPath(i,1);
        P_y(i) = optimalPath(i+1,2) - optimalPath(i,2);
        PathAng(i) = atan2(P_y(i),P_x(i));
        
        if length(PathAng) < 2 && PathAng(i) >= (PathAng(i-1) - pi/8) && PathAng(i) <= (PathAng(i-1) + pi/8) 
            Move = Move + 1;
        else
            break;
        end
   end
   nextPosition = optimalPath(Move+1,:);
else 
    nextPosition = optimalPath(2,:);
end
