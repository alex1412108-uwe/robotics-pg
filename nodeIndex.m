function nIndex = nodeIndex(open,xVal,yVal)
    %This function returns the index of the location of a node in the list
    %OPEN
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    i=1;
    while(open(i,2) ~= xVal || open(i,3) ~= yVal )
        i=i+1;
    end;
    nIndex=i;
end