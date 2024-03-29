%% Function to break map into a grid of points- 
%  modification of function polygrid by Sulimon Sattar, Mathworks, May 2013

    function [inPoints] = polygrid( xv, yv, ppa)

	N = sqrt(ppa); % Number of points, from points per square area 

    %Find the bounding rectangle, using the vector data from map 
	lower_x = min(xv);
	higher_x = max(xv);

	lower_y = min(yv);
	higher_y = max(yv);

    %Create a grid of points within the bounding rectangle
	inc_x = 1/N;
	inc_y = 1/N;
	
	interval_x = lower_x:inc_x:higher_x;
	interval_y = lower_y:inc_y:higher_y;
	[bigGridX, bigGridY] = meshgrid(interval_x, interval_y);
	
    %Filter grid to get only points in polygon
	in = inpolygon(bigGridX(:), bigGridY(:), xv, yv);
    
    %Return the co-ordinates of the points that are in the polygon
	inPoints = [bigGridX(in), bigGridY(in)];

    end