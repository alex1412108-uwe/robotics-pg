function[modifiedMap] = modifyMap(map)

% %More maps will be used
% maps = cell(3,1); %needed for making jagged arrays
% % maps{1} = [0,0;60,0;60,45;45,45]; %Quadrilateral Map
% maps{1} = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
% maps{2} = [0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map
% maps{3} = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0]; %repeated features
% map = maps{1};
%% setup code
%you can modify the map to take account of your robots configuration space
robot = sqrt(18);
L = length(map);
modifiedMap = map;
%% point 1
vLx = map(L,1) - map(1,1);
vLy = map(L,2) - map(1,2);

v1x = map(2,1) - map(1,1);
v1y = map(2,2) - map(1,2);

v1 = [vLx vLy];
v2 = [v1x v1y];

a1 = atan2( det([v1;v2;]) , dot(v1, v2) );
angleout = a1/2;
if angleout > 0
    angleout = -angleout;
end

%%
if v1x>vLx
    modifiedMap(1,1) = map(1,1) - robot*cos(angleout);
elseif v1x<vLx
    modifiedMap(1,1) = map(1,1) + robot*cos(angleout);
else
end

if v1y>vLy
    modifiedMap(1,2) = map(1,2) - robot*sin(angleout);
elseif v1y<vLy
    modifiedMap(1,2) = map(1,2) + robot*sin(angleout);
else
end

in = inpolygon(modifiedMap(1,1), modifiedMap(1,2), map(:,1),map(:,2));
if in == 0
    angleout = angleout + pi;
    if v1x>vLx
        modifiedMap(1,1) = map(1,1) - robot*cos(angleout);
    elseif v1x<vLx
        modifiedMap(1,1) = map(1,1) + robot*cos(angleout);
    else
    end
    
    if v1y>vLy
        modifiedMap(1,2) = map(1,2) - robot*sin(angleout);
    elseif v1y<vLy
        modifiedMap(1,2) = map(1,2) + robot*sin(angleout);
    else
    end
end

%% point L

vllx = map(L-1,1) - map(L,1);
vlly = map(L-1,2) - map(L,2);

vLx = map(1,1) - map(L,1);
vLy = map(1,2) - map(L,2);

v1 = [vllx vlly];
v2 = [vLx vLy];

a1 = atan2( det([v1;v2;]) , dot(v1, v2) );
angleout = a1/2;
if angleout > 0
    angleout = -angleout;
end

if vLx>vllx
    modifiedMap(L,1) = map(L,1) - robot*cos(angleout);
elseif vLx<vllx
    modifiedMap(L,1) = map(L,1) + robot*cos(angleout);
else
end

if vLy>vlly
    modifiedMap(L,2) = map(L,2) - robot*sin(angleout);
elseif vLy<vlly
    modifiedMap(L,2) = map(L,2) + robot*sin(angleout);
else
end

in = inpolygon(modifiedMap(L,1), modifiedMap(L,2), map(:,1),map(:,2));
if in == 0
    angleout = angleout + pi;
    
    if vLx>vllx
        modifiedMap(L,1) = map(L,1) - robot*cos(angleout);
    elseif vLx<vllx
        modifiedMap(L,1) = map(L,1) + robot*cos(angleout);
    else
    end
    
    if vLy>vlly
        modifiedMap(L,2) = map(L,2) - robot*sin(angleout);
    elseif vLy<vlly
        modifiedMap(L,2) = map(L,2) + robot*sin(angleout);
    else
    end
end

%% for loop for points 2 to L-1
for i = 2:L-1
    v1x = map(i-1,1) - map(i,1);
    v1y = map(i-1,2) - map(i,2);
    v2x = map(i+1,1) - map(i,1);
    v2y = map(i+1,2) - map(i,2);
    
    v1 = [v1x v1y];
    v2 = [v2x v2y];
    
    a1 = atan2( det([v1;v2;]) , dot(v1, v2) );
    
    angleout = a1/2;
    if angleout > 0
        angleout = -angleout;
    end
    
    if v2x>v1x
        modifiedMap(i,1) = map(i,1) - robot*cos(angleout);
    elseif v2x<v1x
        modifiedMap(i,1) = map(i,1) + robot*cos(angleout);
    else
    end
    
    if v2y>v1y
        modifiedMap(i,2) = map(i,2) - robot*sin(angleout);
    elseif v2y<v1y
        modifiedMap(i,2) = map(i,2) + robot*sin(angleout);
    else
    end
      
    in = inpolygon(modifiedMap(i,1), modifiedMap(i,2), map(:,1),map(:,2));
    if in == 0
        angleout = angleout + pi;
        if v2x>v1x
            modifiedMap(i,1) = map(i,1) - robot*cos(angleout);
        elseif v2x<v1x
            modifiedMap(i,1) = map(i,1) + robot*cos(angleout);
        else
        end
        
        if v2y>v1y
            modifiedMap(i,2) = map(i,2) - robot*sin(angleout);
        elseif v2y<v1y
            modifiedMap(i,2) = map(i,2) + robot*sin(angleout);
        else
        end
    end
end
