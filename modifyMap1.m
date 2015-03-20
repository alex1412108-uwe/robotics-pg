function[modifiedMap] = modifyMap1(map)

% %More maps will be used
% maps = cell(3,1); %needed for making jagged arrays
% % maps{1} = [0,0;60,0;60,45;45,45]; %Quadrilateral Map
% maps{1} = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
% maps{2} = [0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map
% maps{3} = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0]; %repeated features
% map = maps{1};
%% setup code
%you can modify the map to take account of your robots configuration space
robot = 12;
L = length(map);
modifiedMap = map;
%% point 1
vLx = map(L,1) - map(1,1); % x cord for vector 1 to L
vLy = map(L,2) - map(1,2); % y cord for vector 1 to L 

v1x = map(2,1) - map(1,1); % x cord for vector 1 to 2
v1y = map(2,2) - map(1,2); % y cord for vector 1 to 2

u1 = [vLx vLy]/sqrt(vLx^2+vLy^2); % unit vector 1L
u2 = [v1x v1y]/sqrt(v1x^2+v1y^2); % unit vector 12

newPoint = u1 + u2;  
modifiedMap(1,:) = map(1,:) + robot*newPoint; 
in = inpolygon(modifiedMap(1,1),modifiedMap(1,2), map(:,1), map(:,2));
if in == 0 
    modifiedMap(1,:) = map(1,:) - newPoint*robot; 
end 

%% point L

vllx = map(L-1,1) - map(L,1); % x cord for vector 1 to L
vlly = map(L-1,2) - map(L,2); % y cord for vector 1 to L 

vLlx = map(1,1) - map(L,1); % x cord for vector 1 to 2
vLly = map(1,2) - map(L,2); % y cord for vector 1 to 2

ull = [vllx vlly]/sqrt(vllx^2+vlly^2); % unit vector 1L
uLl = [vLlx vLly]/sqrt(vLlx^2+vLly^2); % unit vector 12

newPoint = ull + uLl;  
modifiedMap(L,:) = map(L,:) + robot*newPoint; 
in = inpolygon(modifiedMap(L,1),modifiedMap(L,2), map(:,1), map(:,2));
if in == 0 
    modifiedMap(L,:) = map(L,:) - newPoint*robot; 
end 

%% for loop for points 2 to L-1
for i = 2:L-1
    
    v1x = map(i-1,1) - map(i,1);
    v1y = map(i-1,2) - map(i,2);
    v2x = map(i+1,1) - map(i,1);
    v2y = map(i+1,2) - map(i,2);
    
    v1 = [v1x v1y]/sqrt(v1x^2 + v1y^2);
    v2 = [v2x v2y]/sqrt(v2x^2 + v2y^2);
    
    
    newPoint = v1 + v2;  
    modifiedMap(i,:) = map(i,:) + robot*newPoint; 
    in = inpolygon(modifiedMap(i,1),modifiedMap(i,2), map(:,1), map(:,2));
        if in == 0 
             modifiedMap(i,:) = map(i,:) - newPoint*robot; 
        end 
end
    modifiedMap(:,:) = round(modifiedMap(:,:)); 
end
