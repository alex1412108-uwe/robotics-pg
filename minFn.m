function iMin = minFn(open,openCount,xTarget,yTarget)
%Function to return the Node with minimum fn
% This function takes the list OPEN as its input and returns the index of the
% node that has the least cost
%
%   Copyright 2009-2010 The MathWorks, Inc.

 tempArray=[];
 k=1;
 flag=0;
 goalIndex=0;
 for j=1:openCount
     if (open(j,1)==1)
         tempArray(k,:)=[open(j,:) j]; %#ok<*AGROW>
         if (open(j,2)==xTarget && open(j,3)==yTarget)
             flag=1;
             goalIndex=j;%Store the index of the goal node
         end;
         k=k+1;
     end;
 end;%Get all nodes that are on the list open
 if flag == 1 % one of the successors is the goal node so send this node
     iMin=goalIndex;
 end
 %Send the index of the smallest node
 if size(tempArray ~= 0)
  [minFn,tempMin]=min(tempArray(:,8));%Index of the smallest node in temp array
  iMin=tempArray(tempMin,9);%Index of the smallest node in the OPEN array
 else
     iMin=-1;%The temp_array is empty i.e No more paths are available.
 end;