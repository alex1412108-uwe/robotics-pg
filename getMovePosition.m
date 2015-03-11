function nextPosition = getMovePosition(nextPosition)
% Inputs: optimal Path -> [nx2] vector of X,Y coordinates
% Outputs: nextPosition -> [1x2] vector of furthest XY coordinate in line
optimalPath = [0,0;1,1;2,2;1,2];
Move = 1;
escape = 0;
while escape == 0
    for i = 1:1:length(optimalPath)-1
        P_x(i) = optimalPath(i+1,1) - optimalPath(i,1);
        P_y(i) = optimalPath(i+1,2) - optimalPath(i,2);
        PathAng(i) = atan2(P_y(i),P_x(i));
        if length(PathAng) < 2
            escape =0;
        elseif PathAng(i) == PathAng(i-1)
            Move = Move + 1;
        else
            escape = 1;
        end
    end
end
nextPosition = optimalPath(Move+1,:);