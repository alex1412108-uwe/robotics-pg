% Function to test bots
particles(10,1) = BotSim;
for i=1:10
    particles(i) = BotSim([0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105],[0,0.1,0.1]);
end

pos = zeros(10,2)+2;
for i=1:10
    pos(i,:) = particles(i).getBotPos();
end

disp(pos);

for i=1:10
    particles(i).move(10);
end

for i=1:10
    pos(i,:) = particles(i).getBotPos();
end

disp(pos);