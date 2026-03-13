package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

public class Constants {

    public class IntakeConst {
        //as of right now, both the intake and the rollers have a 1:1 conversion factor. 
        public static final double positionConversionFactor = 1; //for kicker position
        public static final double velocityConversionFactor = 1; //for roller velocity
        
        public static final int rollerSpeed = 5000; //rpm for rollers - can determine empirically with time
    }

    public class SpindexerConst {
        public static final double bpsConversionFactor = 5; //assuming no skips, 1 rot/sec = 5 balls 

    }

    public class DrivetrainConst {
        //CAN IDs for all our drivesystem
            public static final double sideLength = .552; //meters from one module center to the other. //.622 previously, one encoder center to another
            public static final double halfSideLength = sideLength/2;
            public static final double kMaxVelocity = 1.5; //note that this value must be lower than the max speed for a swerve module, because for a given path a module may need to move further than the drivetrain.
            public static final double kMaxAccel = 12;
            public static final double kMaxChassisRotsPerSecond = 1 * (2 * Math.PI); //front coefficient is how many rots/sec we have. 
            public static final double kMaxChassisRotsPerSecondPerSecond = 4 * (2 * Math.PI); //Acceleration
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

            public static final double kMaxModuleSpeed = DrivetrainConst.kMaxVelocity; // 5.88 meters per second or 19.3 ft/s (max speed of SDS Mk4i with Vortex motor)
            public static final double kMaxModuleAccel = DrivetrainConst.kMaxAccel;
            public static final double kP = 0.05;//0.228;
            public static final double kI = 0.0; //.1
            public static final double kD = 0.01; //0.0095;      
            public static final double kS = 0.05;
            public static final double kV = 2.77;
            public static final double kA = 0;
            public static final double kVelocityTolerance = 0.01; // m/s  , 0.01
            public static final double kClosedLoopRampRate = 0.07;
            public static final int kMaxDriveAmps = 40;

            public static final double kMaxRamSpeed = .5; //meters per second. there's a trade-off where lower speed means we push better, but if it's too low it's not gonna matter.
            public static final int kRamSpeedRPMLimit = (int) ((kMaxRamSpeed/rotationsToMetersScaler)*60); //converts desired speed to max RPM. see upper comment about lower=better
            public static final double SpeedLimiter = .7; //SDS Mk4I standard gear ratio from motor to wheel, divide by 60 to go from secs to mins

    }   
    public class TrajectoryConst {
        public static final double kMaxSpeed = DrivetrainConst.kMaxVelocity;
        public static final double kMaxAcceleration = DrivetrainConst.kMaxAccel;
        
        //PP STANDS FOR PATHPLANNER
        //translation section
        public static final double kPTPP = 0.1;
        public static final double kITPP = 0;
        public static final double kDTPP = 0;
        
        //rotation section of our pid loops for auto alignment.
        public static final double kPRotPP = 0.5;
        public static final double kIRotPP = 0;
        public static final double kDRotPP = 0;   

    }

    public class CameraConst {
        public static final String pvCamOne = "OV9281";
        public static final String pvCamTwo = "camera2";
    }

    public class CANID {
        //Should replace based on wiring so a break is immediately able to be determined.
        //drivetrain
        public static final int FLDrive = 1;
        public static final int FLTURN = 2;
        public static final int FRDrive = 3;
        public static final int FRTurn = 4;
        public static final int BRDrive = 5;
        public static final int BRTurn = 6;
        public static final int BLDrive = 7;
        public static final int BLTurn = 8;

        //shooter
        public static final int LeftS = 11;
        public static final int RightS = 12;
        public static final int Hood = 13;
        public static final int Turret = 14;

        //kicker
        public static final int FrontK = 17;
        public static final int BackK = 16;

        //spindexer
        public static final int Spindexer = 15;

        //intake
        public static final int Rollers = 10;
        public static final int Flipper = 9;
    }

    public class FIELD_CONST {
        //all units in meters, based on offsets from blue origin
        public static final double[] BLUE_HUB = {4.62, 4.025};
        public static final double[] RED_HUB = {11.92, 4.025};
        
        public static final double[] BLUE_CYCLE_NORTH = {1,1};
        public static final double[] BLUE_CYCLE_SOUTH = {1,1};
        public static final double[] RED_CYCLE_NORTH = {1,1};
        public static final double[] RED_CYCLE_SOUTH = {1,1};
    }

    public class TurretConst {
        //relative to the front of the robot along x axis (which is zero), our turret capabilities in radians
        public static final double turretMin = 0 * Math.PI; 
        public static final double turretMax = 0 * Math.PI;

        public static final double positionConversionFactor = 0;
    }
    
    public class HoodConst {
        public static final double positionConversionFactor = 0;
        
    }

    public class FlywheelConst {
        public static final double closedLoopRampRate = 1;

    }
}