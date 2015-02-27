%%test

%More maps will be used
maps = cell(3,1); %needed for making jagged arrays
maps{1} = [0,0;60,0;60,45;45,45]; %Quadrilateral Map
maps{1} = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]; %default map
maps{2} = [0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80]; %long map
maps{3} = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0]; %repeated features
map = maps{3};
modifiedMap = modifyMap(map);
plot(map(:,1),map(:,2),'lineWidth',2,'Color','r'); % draws arena
hold on;
plot(modifiedMap(:,1),modifiedMap(:,2),'lineWidth',2,'Color','g'); % draws arena
axis equal; %keeps the x and y scale the same
hold on; %