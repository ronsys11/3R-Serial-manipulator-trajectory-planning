%% Manipulator Homing
clc
clear
a = arduino('COM4', 'Uno', 'Libraries', 'Servo');
servo1 = servo(a, 'D9', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6);
servo2 = servo(a, 'D10', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6);
servo3 = servo(a, 'D6', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6);
writePosition(servo1,0/180);
writePosition(servo2,0/180);
writePosition(servo3,133/180 ); % Home offsets have been added here, 120 + 13
home = 1;
while true
    if home == 1
        fprintf('Current position is [-140  0  297] \n')
        fprintf('Press any key to Start: ')
        pause;
        clc
    else 
        fprintf('Current position is [%.2f %.2f %.2f] \n',x(k),y(k),z(k)) 
        fprintf('Press any key to continue: ')
        pause;
        clc
    end
    %% workspace plot (ndgrid is used for faster calculation)
    % Joint constraints
    range_t1 = (0:1:160);
    range_t2 = (0:1:80);
    range_t3 = (-60:1:70);
    [t1, t2, t3] = ndgrid(range_t1, range_t2, range_t3);
    % Solving for X,Y,Z using forward kinematics equation
    x1 = round(+ 140.*sind(t3).*sind(t2).*cosd(t1) - 140.*cosd(t3).*cosd(t1).*cosd(t2) - 180.*sind(t2).*cosd(t1),1);
    y1 = round( + 140.*sind(t3).*sind(t1).*sind(t2) - 140.*cosd(t3).*sind(t1).*cosd(t2) - 180.*sind(t2).*sind(t1), 1);
    z1 = round(- 140.*sind(t3).*cosd(t2) - 140.*cosd(t3).*sind(t2) + 180.*cosd(t2) + 117,1); 
    plot3(x1(:),y1(:),z1(:),'.')
    xlabel('X axis')
    ylabel('Y axis')
    zlabel('Z axis')
    grid on
    %% Point selection on the workspace plot
    % Turn on data cursor mode
    dcm_obj = datacursormode(gcf);
    set(dcm_obj,'DisplayStyle','datatip','SnapToDataVertex','off','Enable','on')
    points = input('\nEnter the no of points : [0 for Home]');
    if points > 0 % Check the input for Home position or not
        for k=1:1:points
            fprintf('Select point no %d in the Workspace\n',k);
            pause;% wait for user input
            cursor_info = getCursorInfo(dcm_obj);
            point = cursor_info.Position; % Store the point coordinates in arrays
            x(k) = [point(1)];
            y(k) = [point(2)];
            z(k) = [point(3)];
            fprintf('Selected point %d is [%.2f  %.2f  %.2f]\n',k,x(k),y(k),z(k))
        end
    elseif points == 0 % if the user input is Home, write home coordinates to X,Y,Z
        points = points+1;
        x = [-140];
        y = [0];
        z = [297];
    end
    %% inverse kinematics model
    for k=1:1:points % Solving for each point and writing to servos in the same loop.
        % joint constraints
        range_t1 = (0:1:160);
        range_t2 = (0:1:80);
        range_t3 = (-60:1:70);
        [t1, t2, t3] = ndgrid(range_t1, range_t2, range_t3);
        % Solve for theta 1, 2 & 3.
        theta1 = round((-x(k)) + 140.*sind(t3).*sind(t2).*cosd(t1) - 140.*cosd(t3).*cosd(t1).*cosd(t2) - 180.*sind(t2).*cosd(t1), 1);
        theta2 = round( (-y(k)) + 140.*sind(t3).*sind(t1).*sind(t2) - 140.*cosd(t3).*sind(t1).*cosd(t2) - 180.*sind(t2).*sind(t1), 1);
        theta3 = round((-z(k)) - 140.*sind(t3).*cosd(t2) - 140.*cosd(t3).*sind(t2) + 180.*cosd(t2) + 117 , 1); 
        logicInd = (theta1 == 0) & (theta2 == 0) & (theta3 == 0);
        % store the Raw solutions To variables
        Ra1 = t1(logicInd);
        Ra2 = t2(logicInd);
        Ra3 = t3(logicInd);
        %% offsets
        % Default offsets.
        Ra3 = -Ra3; % Raw value inverted to account for 4bar mechanism's physical inversion.
        DOa1 = Ra1+0; %add home offsets
        DOa2 = Ra2+0;
        DOa3 = (Ra3+120) - DOa2; % 120 for servo home offset and -a2 for the link2 compensation
        IN = [DOa1 DOa2 DOa3];
        % Servo motor offsets is only dependent on the absolute value from 0 of the servo
        for i=1:1:3 % Check the range of servo actuation and add offsets accordingly.
            if ( IN(i) >10 && IN(i) <= 45) 
                IN(i) = IN(i)+5;
            elseif (IN(i) > 45 && IN(i) <= 90)
                IN(i) = IN(i)+10;
            elseif (IN(i) > 90 && IN(i) <= 135)
                IN(i) = IN(i)+15;
            elseif (IN(i) > 135 && IN(i) <= 180)
                IN(i) = IN(i)+20;
            end
        end
        OFa1 = IN(1);
        OFa2 = IN(2);
        OFa3 = IN(3);
        %% conversion to 0 to 1 scale as required by writeposition function
        a1 = OFa1/180;
        a2 = OFa2/180;
        a3 = OFa3/180;
        %% Joint space trajectory planning
        % Check if the manipulator is starting from Home or not.
        if home == 1 
            prea1 = 0/180;
            prea2 = 0/180; % home offsets have to be added here
            prea3 = 133/180;
            H1 = [prea1 prea2 prea3]; 
            H2 = [a1 a2 a3];
        else
            H1 = [prea1 prea2 prea3]; % Values for the starting point is stored in the previous iteration
            H2 = [a1 a2 a3];
        end
        % Finding the absolute angular displacement of each joint and
        % fixing trajectory time based on the largest displacement.
        diff1 = abs(prea1 - a1); 
        diff2 = abs(prea2 - a2);
        diff3 = abs(prea3 - a3);
        if ( diff1 <= 0.25 && diff2 <= 0.25 && diff3 <= 0.25)
            tt = 0.5;
        elseif (diff1 <= 0.5 && diff2 <= 0.5 && diff3 <= 0.5)
            tt = 1;
        elseif (diff1 <= 0.75 && diff2 <= 0.75 && diff3 <= 0.75)
            tt = 1.5;
        else 
            tt = 2;
        end
        t = [0:0.05:tt]'; %tt is the time taken for trajectory
        ti = (tt/0.05)+1;  %0.05 is the step value
        H = jtraj(H1,H2,t);
        H = abs(H);
        % Accessing elements of output from trajectory planner and writing
        % to the corresponding servos
        tic
        for j = 1:1:ti % ti is the no of rows in H matrix
            writePosition(servo1,H(j,1)); 
            writePosition(servo2,H(j,2));
            writePosition(servo3,H(j,3));
            %pause(0.0015) % Delay has to given to achieve the given
            %trajectory time.
        end
        time = round(toc,2);
        fprintf('\n')
        prea1 = a1; % store the current joint angles to use as previous position in the trajectory planner.
        prea2 = a2;
        prea3 = a3;
        home = home+1; % Manipulator not in home position anymore
    
        %% Display parameters on the command window after each point is achieved
        fprintf ('Time taken for trajectory %d is %.2f sec in %.2f steps\n',k,time,ti)
        fprintf ('Joint_angles in degrees [%d   %d  %d]\n',Ra1,Ra2,-Ra3)
        fprintf ('Position of the EE X Y Z is [%.2f   %.2f   %.2f]\n\n',x(k),y(k),z(k))
        pause(1) % wait in the given position for 1 second then move on to the next position.
    end      
end