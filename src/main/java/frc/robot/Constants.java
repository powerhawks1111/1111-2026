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
            public static final double sideLength = .622; //meters from one module center to the other. //.622 previously, one encoder center to another
            public static final double halfSideLength = sideLength/2;
            public static final double kMaxVelocity = 2; //note that this value must be lower than the max speed for a swerve module, because for a given path a module may need to move further than the drivetrain.
            public static final double kMaxAccel = 5;
            public static final double kMaxChassisRotsPerSecond = 2 * (2 * Math.PI); //front coefficient is how many rots/sec we have. 
            public static final double kMaxChassisRotsPerSecondPerSecond = 8 * (2 * Math.PI); //Acceleration.
            
    }
    public class ModuleConst {

    }
    public class DriveConst {
            //TODO update module-specific constants when new modules ordered (gear ratio, max speed, etc.)
            public static final double kWheelDiameter = .1016; // 0.1016 M wheel diameter (4")
            public static final double kWheelCircumference = Math.PI * kWheelDiameter;
            public static final double turningWheelGearRatio = 150/7; //standard steering gear ratio on MK4i 
            public static final double drivingWheelGearRatio = 6.12; //L3 gear ratio for driving

            public static final double turnEncoderScaler = 2* Math.PI;
            public static final double rotationsToMetersScaler = (kWheelCircumference/drivingWheelGearRatio);
            public static final double rpmToVelocityScaler = (kWheelCircumference)/(60*drivingWheelGearRatio); //SDS Mk4I standard gear ratio from motor to wheel, divide by 60 to go from secs to mins

            public static final double kMaxModuleSpeed = 2.5; // 5.88 meters per second or 19.3 ft/s (max speed of SDS Mk4i with Vortex motor)
            public static final double kMaxModuleAccel = 5;
            public static final double kP = 0.072;//0.228;
            public static final double kI = 0.0; //.1
            public static final double kD = 0.2; //0.0095;      
            public static final double kS = 0.05;
            public static final double kV = 0.17499;
            public static final double kA = 3.44;
            public static final double kVelocityTolerance = 0.001; // m/s  
            public static final double kClosedLoopRampRate = 0.07;
            public static final int kMaxDriveAmps = 50;

            public static final double kMaxRamSpeed = 

    }   
    public class TrajectoryConst {
        public static final double kMaxSpeed = DrivetrainConst.kMaxVelocity;
        public static final double kMaxAcceleration = DrivetrainConst.kMaxVelocity;
        
        //translation section of our pid loops for auto alignment
        public static final double kPT = 0;
        public static final double kIT = 0;
        public static final double kDT = 0;
        
        //rotation section of our pid loops for auto alignment.
        public static final double kPRot = 0;
        public static final double kIRot = 0;
        public static final double kDRot = 0;   
    }

    public class CameraConstants {
        public static final String pvCamOne = "camera1";
        public static final String pvCamTwo = "camera2";
    }
}