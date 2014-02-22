function  LocalizationLabReal()
    % LocalizationLabReal is a robot controller which uses a Bayes filter
    % for Localization and a wavefront planner to drive a lego nxt to the
    % desired location regardless of its starting location.  
    %
    % Written by Trevor Decker (tdecker@andrew.cmu.edu) for CMU's 16-311
    % v0.1 2/21/2014
    %
    % uses RWTH frame work and is based on RWTH's demo code

    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Setup %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    config();  %sets up global variables for robots configuration 
    desiredPose = [.1,.1,0];% desired end location 
    map = mapGenerator;
    configMap = calculateConfigurationSpace(map,1);
    global dbf;
    global dt = .1;
    dbf = DiscreteBayesFilter(3,@transitionModel,@observationModel);

    % Set up communication to the nxt
    
    %% Check toolbox installation
    % verify that the RWTH - Mindstorms NXT toolbox is installed.
    if verLessThan('RWTHMindstormsNXT', '3.00');
    error(strcat('This program requires the RWTH - Mindstorms NXT Toolbox ' ...
        ,'version 3.00 or greater. Go to http://www.mindstorms.rwth-aachen.de ' ...
        ,'and follow the installation instructions!'));
    end%if

    %% Clear and close
    COM_CloseNXT all
    clear all
    close all
    
    %% Open Bluetooth connetion
    h = COM_OpenNXT('bluetooth.ini');
    COM_SetDefaultNXT(h);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%% Main Control Loop %%%%%%%%%%%%%%%%%%%%%%%%%
    while(1)
    %the best guess of where we currently are
    [belivedPose(1); belivedPose(2); belivedPose(3); p] = dbf.GetMean();
    if p < .0001
        dPose = lostMotion();
        %determines the angle we should be at
        update(dpose);
    elseif atDesiredLocation(belivedPose,finish)
        path = WaveFrontPlanner(configMap,[belivedPose(1),belivedPose(2)],finish);
        if size(path,1) < 1 || (path(1,1) == -1 && length(path) == 1)
            dPose = lostMotion();
        else
            dPose(1:2) = belivedPose(1:2)' - [path(1,1), path(1,2)];
            dPose(3) = -wrapToPi(belivedPose(3) - atan2(-dPose(2),-dPose(1)));

        end
        %determines the angle we should be at
        update(dPose);
    else
         dth = wrapToPi(belivedPose(3) - finish(3));
         if( abs(dth) < MAXERRORTH)
             display('done');
             break;
         else
             dPose =  [0,0,-dth];
             update(dPose);
             
        end
        
    end
    end
    
    
   
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% clean up %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Close Bluetooth connection
    COM_CloseNXT(h);
    
end

function update(dPose)
        global dbf;
        global dt;
        dbf.Smooth('Laplace',.00001);
        measurement = ImagePipelineObservation()
        dbf.ObservationUpdate(map,measurement);
        displacement = drive(speed,turning,dt);
        dbf.TransitionUpdate(map,displacement);
end

