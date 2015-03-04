function expArray=expandArray(nodeX, nodeY, hn, xTarget, yTarget, closed, maxX, maxY)
    %Function to return an expanded array
    %This function takes a node and returns the expanded list
    %of successors,with the calculated fn values.
    %The criteria being none of the successors are on the CLOSED list.
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    
    expArray = [];
    expCount = 1;
    c2 = size(closed, 1);%Number of elements in CLOSED including the zeros
    for k= 1:-1:-1
        for j= 1:-1:-1
            if (k~=j || k~=0)  %The node itself is not its successor
                s_x = nodeX+k;
                s_y = nodeY+j;
                if( (s_x >0 && s_x <=maxX) && (s_y >0 && s_y <=maxY))%node within array bound
                    flag=1;                    
                    for c1=1:c2
                        if(s_x == closed(c1,1) && s_y == closed(c1,2))
                            flag=0;
                        end;
                    end;%End of for loop to check if a successor is on closed list.
                    if (flag == 1)
                        expArray(expCount,1) = s_x;
                        expArray(expCount,2) = s_y;
                        expArray(expCount,3) = hn+distanceLine(nodeX,nodeY,s_x,s_y);%cost of travelling to node
                        expArray(expCount,4) = distanceLine(xTarget,yTarget,s_x,s_y);%distance between node and goal
                        expArray(expCount,5) = expArray(expCount,3)+expArray(expCount,4);%fn
                        expCount=expCount+1;
                    end%Populate the exp_array list!!!
                end% End of node within array bound
            end%End of if node is not its own successor loop
        end%End of j for loop
    end%End of k for loop    