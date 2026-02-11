package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

public class Constants {
    public class DrivetrainConst {
        //CAN IDs for all our drivesystem
            public static final int FLDrive = 1;
            public static final int FLTURN = 2;
            public static final int FRDrive = 3;
            public static final int FRTurn = 4;
            public static final int BRDrive = 5;
            public static final int BRTurn = 6;
            public static final int BLDrive = 7;
            public static final int BLTurn = 8;
            public static final double sideLength = .552; //meters from one module center to the other. //.622 previously, one encoder center to another
            public static final double halfSideLength = sideLength/2;
            public static final double kMaxVelocity = 3.5; //note that this value must be lower than the max speed for a swerve module, because for a given path a module may need to move further than the drivetrain.
            public static final double kMaxAccel = 5;
            public static final double kMaxChassisRotsPerSecond = 2 * (2 * Math.PI); //front coefficient is how many rots/sec we have. 
            public static final double kMaxChassisRotsPerSecondPerSecond = 8 * (2 * Math.PI); //Acceleration
    }
    public class ModuleConst {

    }
    public class DriveConst {
            //TODO update module-specific constants when new modules ordered (gear ratio, max speed, etc.)
            public static final double kWheelDiameter = .1016; // 0.1016 M wheel diameter (4")
            public static final double kWheelCircumference = Math.PI * kWheelDiameter;
            public static final double turningWheelGearRatio = 150/7; //standard steering gear ratio on MK4i 
            public static final double drivingWheelGearRatio = 8.14; //L3 gear ratio for driving

            public static final double turnEncoderScaler = 2* Math.PI;
            public static final double rotationsToMetersScaler = (kWheelCircumference/drivingWheelGearRatio);
            public static final double rpmToVelocityScaler = (kWheelCircumference)/(60*drivingWheelGearRatio); //SDS Mk4I standard gear ratio from motor to wheel, divide by 60 to go from secs to mins

            public static final double kMaxModuleSpeed = 3.5; // 5.88 meters per second or 19.3 ft/s (max speed of SDS Mk4i with Vortex motor)
            public static final double kMaxModuleAccel = 5;
            public static final double kP = 0.05;//0.228;
            public static final double kI = 0.0; //.1
            public static final double kD = 0.01; //0.0095;      
            public static final double kS = 0.05;
            public static final double kV = 2.77;
            public static final double kA = 0;
            public static final double kVelocityTolerance = 0.001; // m/s  
            public static final double kClosedLoopRampRate = 0.07;
            public static final int kMaxDriveAmps = 40;

            public static final double kMaxRamSpeed = 4; //meters per second. there's a trade-off where lower speed means we push better, but if it's too low it's not gonna matter.
            public static final int kRamSpeedRPMLimit = (int) ((kMaxRamSpeed/rotationsToMetersScaler)*60); //converts desired speed to max RPM. see upper comment about lower=better
            public static final double SpeedLimiter = .7; //SDS Mk4I standard gear ratio from motor to wheel, divide by 60 to go from secs to mins

    }   
    public class TrajectoryConst {
        public static final double kMaxSpeed = DrivetrainConst.kMaxVelocity;
        public static final double kMaxAcceleration = DrivetrainConst.kMaxVelocity;
        
        //REGULAR SELF-ALIGN 
        //translation section of our pid loops for auto alignment
        public static final double kPT = 4.5;
        public static final double kIT = 0;
        public static final double kDT = 0.9;
        
        //rotation section of our pid loops for auto alignment.
        public static final double kPRot = .05;
        public static final double kIRot = 0;
        public static final double kDRot = 0;   

        
        //PP STANDS FOR PATHPLANNER
        //translation section
        public static final double kPTPP = 1;
        public static final double kITPP = 0;
        public static final double kDTPP = 0;
        
        //rotation section of our pid loops for auto alignment.
        public static final double kPRotPP = 1;
        public static final double kIRotPP = 0;
        public static final double kDRotPP = 0;   


        //C STANDS FOR CHOREO
        //translation section of our pid loops for auto alignment
        public static final double kPTC = 1;
        public static final double kITC = 0;
        public static final double kDTC = 0;
        
        //rotation section of our pid loops for auto alignment.
        public static final double kPRotC = 1;
        public static final double kIRotC = 0;
        public static final double kDRotC = 0;  
    }

    public class CameraConstants {
        public static final String pvCamOne = "camera1";
        public static final String pvCamTwo = "camera2";
    }
}