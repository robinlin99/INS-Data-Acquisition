clear all
clc

% Initiate Arduino communication
arduino=serial('/dev/tty.usbmodem1421','BaudRate',9600);
fopen(arduino)

pause(5);

% Initiate all body and global state vectors at t = 0, 0dt
AccVector_body = zeros(1,3,'uint32');
AccVector_global = zeros(1,3,'uint32');
VelVector_global = zeros(1,3,'uint32');
AngVelVector_body = zeros(1,3,'uint32');
PosVector_global = zeros(1,3,'uint32');
Orientation = zeros(1,3,'uint32');

% Create data storage matrices
Pos_Data = zeros(4,10000);
Vel_Data = zeros(4,10000);
Accel_Data = zeros(4,10000);
Orientation_Data = zeros(4,10000);

% Define gravitational field strength; acceleration due to gravity
g = 10;
% Define sample size 
sample_size = 50000;
% Define identity matrix I
I = eye(3);

% Time index t = k 
% Initiate initial direction cosine rotation matrix
DCM_RotMatrix = angle2dcm(yaw, pitch, roll);

% Initiate state vector recording and update cycle
% Time index t = k+1 and beyond
if fscanf(arduino,"%d") == 1.00
    while sample_epoch < sample_size
        % Pass acceleration sensor data into state vector; breakdown into x,y,z components
        AccVector_body(1) = A_x;
        AccVector_body(2) = A_y;
        AccVector_body(3) = A_z;
    
        % Pass angular velocity sensor data into state vector; breakdown into
        % x,y,z components
        AngVelVector_body(1) = W_x;
        AngVelVector_body(2) = W_y;
        AngVelVector_body(3) = W_z;
    
        % Pass orientation data into state vector; breakdown into
        % x,y,z components
        Orientation(1) = yaw;
        Orientation(2) = pitch;
        Orientation(3) = roll;
    
        % Define sigma
        sigma = sqrt((AngVelVector_body(1)*time_differential)^2+(AngVelVector_body(2)*time_differential)^2+(AngVelVector_body(3)*time_differential)^2);
    
        % Define matrix B (integral of skew symmetric form of angular rate
        % state vector)
        B = [0 -AngVelVector_body(3)*time_differential AngVelVector_body(2)*time_differential; AngVelVector_body(3)*time_differential 0 -AngVelVector_body(2)*time_differential; -AngVelVector_body(2)*time_differential AngVelVector_body(1)*time_differential 0];
    
        % DCM rotation matrix update algorithm
        DCM_RotMatrix = DCM_RotMatrix*(I+(sind(sigma)/sigma)*B+((1-cosd(sigma))/sigma^2)*B^2);
    
        % State mechanisation recursive functions 
        % Transform body frame to global frame
        AccVector_global = AccVector_body*DCM_RotMatrix;
        % Index one (x component) of VelVector_global 
        VelVector_global(1) = VelVector_global(1) + time_differential*AccVector_global(1);
        % Index two (y component) of VelVector_global 
        VelVector_global(2) = VelVector_global(2) + time_differential*AccVector_global(2);
        % Index three (z component) of VelVector_global 
        VelVector_global(3) = VelVector_global(3) + time_differential*(AccVector_global(3)-g);
        % Index one (x component) of PosVector_global
        PosVector_global(1) = PosVector_global(1) + time_differential*VelVector_global(1);
        % Index two (y component) of PosVector_global
        PosVector_global(2) = PosVector_global(2) + time_differential*VelVector_global(2);
        % Index three (z component) of PosVector_global
        PosVector_global(3) = PosVector_global(3) + time_differential*VelVector_global(3);
    
        % Position state vector recording
        Pos_Data(1,sample_epoch+2) = sample_epoch+1;
        Pos_Data(2,sample_epoch+2) = PosVector_global(1);
        Pos_Data(3,sample_epoch+2) = PosVector_global(2);
        Pos_Data(4,sample_epoch+2) = PosVector_global(3);
    
        % Orientation state vector recording
        Orientation_Data(1,sample_epoch+2) = sample_epoch+1;
        Orientation_Data(2,sample_epoch+2) = Orientation(1);
        Orientation_Data(3,sample_epoch+2) = Orientation(2);
        Orientation_Data(4,sample_epoch+2) = Orientation(3); 
    end
else
    disp('System not initiating');
end
fclose(arduino);
disp('making plot...just a sec');
