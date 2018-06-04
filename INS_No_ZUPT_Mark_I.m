clear all
clc


% Angular velocity in rad/s, acceleration in m/x^2, euler's angle in
% degrees
% ALL VARIABLES ARE SIGNED FLOATS
% Initiating all states
AccVector_body = zeros(3,1);
AccVector_global = zeros(3,1);
VelVector_global = zeros(3,1);
AngVelVector_body = zeros(3,1);
PosVector_global = zeros(3,1);
Orientation = zeros(3,1);

time_differential = 0.001;

% Defining matrices for data storage
Pos_Data = zeros(4,1000);
Vel_Data = zeros(4,10000);
Accel_Data = zeros(4,301);
Orientation_Data = zeros(4,301);
Ang_Vel_Data = zeros(4,301);

% Defining acceleration due to gravity
g = [0; 0; 10.2];

% Defining sample size
sample_size = 30000;

% Defining 3x3 identity matrix
identity = eye(3);

% Initiating arduino
arduino=serial('/dev/cu.usbmodem1411','BaudRate',115200);
fopen(arduino);

pause(4);


% Read initial orientation at t = k index (serial communication)
yaw_angle_degrees = fscanf(arduino, "%f");
pitch_angle_degrees = fscanf(arduino, "%f");
roll_angle_degrees = fscanf(arduino, "%f");

YT = degtorad(yaw_angle_degrees);
PT = degtorad(pitch_angle_degrees);
RT = degtorad(roll_angle_degrees);

% Compute initial direction cosine matrix
DCM_RotMatrix = angle2dcm(YT,PT,RT);


for sample_epoch = 1:sample_size
        % Data acquisition
        AccVector_body(1) = fscanf(arduino, '%f');
        AccVector_body(2) = fscanf(arduino, '%f');
        AccVector_body(3) = fscanf(arduino, '%f');
        Orientation(1) = degtorad(fscanf(arduino, '%f'));
        Orientation(2) = degtorad(fscanf(arduino, '%f'));
        Orientation(3) = degtorad(fscanf(arduino, '%f'));
        AngVelVector_body(1) = fscanf(arduino, '%f');
        AngVelVector_body(2) = fscanf(arduino, '%f');
        AngVelVector_body(3) = fscanf(arduino, '%f');
        
        % Sample data processing
        % Direction Cosine Matrix Update
        sigma = sqrt((AngVelVector_body(1)*time_differential)^2+(AngVelVector_body(2)*time_differential)^2+(AngVelVector_body(3)*time_differential)^2);
        B = [0 -AngVelVector_body(3)*time_differential AngVelVector_body(2)*time_differential; AngVelVector_body(3)*time_differential 0 -AngVelVector_body(2)*time_differential; -AngVelVector_body(2)*time_differential AngVelVector_body(1)*time_differential 0];
        DCM_RotMatrix = DCM_RotMatrix*(identity+(sin(sigma)/sigma)*B+((1-cos(sigma))/sigma^2)*B^2);
        
        % State mechanisation recursive functions 
        % Transform body frame to global frame
        AccVector_global = DCM_RotMatrix*AccVector_body;
        VelVector_global = VelVector_global + time_differential*(AccVector_global-g);
        PosVector_global = PosVector_global + time_differential*VelVector_global;
       
        % Sample data recording
        Accel_Data(1,sample_epoch+1) = sample_epoch;
        Accel_Data(2,sample_epoch+1) = AccVector_global(1);
        Accel_Data(3,sample_epoch+1) = AccVector_global(2);
        Accel_Data(4,sample_epoch+1) = AccVector_global(3)-10.2;
        Vel_Data(1,sample_epoch+1) = sample_epoch;
        Vel_Data(2,sample_epoch+1) = VelVector_global(1);
        Vel_Data(3,sample_epoch+1) = VelVector_global(2);
        Vel_Data(4,sample_epoch+1) = VelVector_global(3);
        Pos_Data(1,sample_epoch+1) = sample_epoch;
        Pos_Data(2,sample_epoch+1) = PosVector_global(1);
        Pos_Data(3,sample_epoch+1) = PosVector_global(2);
        Pos_Data(4,sample_epoch+1) = PosVector_global(3);
        Orientation_Data(1,sample_epoch+1) = sample_epoch; 
        Orientation_Data(2,sample_epoch+1) = Orientation(1); 
        Orientation_Data(3,sample_epoch+1) = Orientation(2); 
        Orientation_Data(4,sample_epoch+1) = Orientation(3); 
        Ang_Vel_Data(1,sample_epoch+1) = sample_epoch; 
        Ang_Vel_Data(2,sample_epoch+1) = AngVelVector_body(1);
        Ang_Vel_Data(3,sample_epoch+1) = AngVelVector_body(2);
        Ang_Vel_Data(4,sample_epoch+1) = AngVelVector_body(3);     
end
fclose(arduino);
disp('complete!');
disp('plotting...');

% GUI configuration, create two subplots across the upper half of the figure and a third subplot that spans the lower half of the figure
% Acceleration data visualization 
subplot(2,2,1);
AccX = Accel_Data(2,:);
n = Accel_Data(1,:);
scatter(n,AccX);
title('Acceleration in X, Y, Z Directions');
hold on
AccY = Accel_Data(3,:);
scatter(n,AccY);
AccZ = Accel_Data(4,:);
scatter(n,AccZ);
hold off

% Velocity data visualization
subplot(2,2,2);
VX = Vel_Data(2,:);
n = Vel_Data(1,:);
scatter(n,VX);
title('Velocity in X, Y, Z Directions');
hold on
VY = Vel_Data(3,:);
scatter(n,VY);
VZ =Vel_Data(4,:);
scatter(n,VZ);
hold off

% Position data visualization
subplot(2,2,[3,4]);
PosX = Pos_Data(2,:);
PosY = Pos_Data(3,:);
PosZ = Pos_Data(4,:);
scatter3(PosX,PosY,PosZ,'filled');
title('3D Position');
view(-30,10);

