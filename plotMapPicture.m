clear all
clc
map = [0,0;0,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,...
    60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,...
    60;240,60;240,40;300,40;300,0];
modifiedMap = modifyMap1(map);
map1 = [map(:,:); map(1,:)];
modifiedMap1 = [modifiedMap(:,:); modifiedMap(1,:)];
x = zeros(201,1);
y = zeros(201,1); 
j = 1; 
        for i = 0:pi/100:2*pi
            x(j) = (4*cos(i) + 4);
            y(j) = (4*sin(i) + 4); 
            j=j+1; 
        end
        
plot(x(:),y(:),'lineWidth', 2, 'Color','b');
hold on
axis equal
plot(map1(:,1),map1(:,2),'lineWidth', 2, 'Color','r');
hold on
axis equal
plot(modifiedMap1(:,1), modifiedMap1(:,2),'lineWidth', 2, 'Color', 'g')
hold on
axis equal
legend('Robot','Map', 'Modified map')