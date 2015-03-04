function newRow = insertOpen(xVal, yVal, parentX, parentY,hn,gn,fn)
%Function to Populate the OPEN LIST
%OPEN LIST FORMAT
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%-------------------------------------------------------------------------
%
%   Copyright 2009-2010 The MathWorks, Inc.
newRow=[1,8];
newRow(1,1)=1;
newRow(1,2)=xVal;
newRow(1,3)=yVal;
newRow(1,4)=parentX;
newRow(1,5)=parentY;
newRow(1,6)=hn;
newRow(1,7)=gn;
newRow(1,8)=fn;

end