function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%generate some random particles inside the map
num =300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
end

% Particle weight array (initialise to equal values)
particleWeight = zeros(1,num) + 1/num;

%% Localisation code
maxNumOfIterations = 30;
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
                                    % Returns distance to walls
                                    % NB: can't use crossing points... Duh.

    %% Write code for updating your particles scans
    % Predict probability for particle in current location
    % - P(measurement|location)
    % - - Based on estimated ultrasound error
    
    
    %% Write code for scoring your particles 
    % Score particles based on botScan:particleEstimate
    % Update in particleWeight(i)
    % - Using Bayes Theorem:    (m=measurement, L=location)
    % - - P(L|m) = 
    %         P(m|L) * P(L) /
    %         P(m)
    %
    % - - Where: P(m)=P(m|L)*P(L) + P(m|~L)*P(~L)
    %                  and P(L)+P(~L)=1
    %
    % - NB: If using probabilities as weight, normalise to sum to 1
    
    
    %% Write code for resapling your particles
    % Any particle with weight lower than threshold deleted
    % - New random point based on remaining particles?
    % - Any particles outside walls should be eliminated
    % - Set all particle weights equal
    
    particleWeight(1,:) = 1/num;
    
    %% Write code to check for convergence 
    % Check how close particles are to each other
    % - if within tolerance, set convergence=1
    
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn = 0.5;
    move = 2;
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        drawnow;
    end
end
end
