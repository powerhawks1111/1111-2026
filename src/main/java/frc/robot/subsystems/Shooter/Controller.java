package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConst;

public class Controller extends SubsystemBase {
    //vars for static shot
    private static double shooterPureSpeed = 0;
    private static double shooterPureAngle = 0;
    private static double timeOfFlight = 0;
    private static double[] staticShot = {0,0,0};

    private static InterpolatingDoubleTreeMap rpmMap;
    private static InterpolatingDoubleTreeMap hoodMap;
        
    public Controller() {
        rpmMap = new InterpolatingDoubleTreeMap();
        hoodMap = new InterpolatingDoubleTreeMap();

        rpmMap.put(3.0 + 1.75, 3500.0);
        rpmMap.put(6.0 + 1.75, 4000.0);
        rpmMap.put(9.0 + 1.75, 4500.0);
        rpmMap.put(12.0  + 1.75, 4700.0);

        hoodMap.put(3.0 + 1.75, 0.20);
        hoodMap.put(6.0 + 1.75, 0.25);
        hoodMap.put(9.0 + 1.75, 0.3);
        hoodMap.put(12.0 + 1.75, .4);
    }

    /**
     * 
     * @param distance the raw distance to the hub. NOTE, it's in FEET
     * @return double[] -> {RPM, angle}
     */
    public static double[] getShooterSimple(double distance) {
        double[] data = {
            rpmMap.get(distance),
            hoodMap.get(distance)
        };
        return data;
    }

    /**
     * Method needed to convert where our robot center is on the field to where our turret is (where we're actually shooting from)
     * @param currentPosition taken in by our odometry
     * @return our turret's position on the field. 
     */
    public static Translation2d robotToTurretTranslation(Pose2d currentPosition) {
        //treats our turret position as a polar coordinate, with the navx angle relative to the field being our angle value to multiply our radius (robotToTurretCenter) value by to get offsets. 
        return new Translation2d(
            currentPosition.getX() + (TurretConst.robotToTurretCenterX * Math.cos(currentPosition.getRotation().getRadians())), 
            currentPosition.getY() + (TurretConst.robotToTurretCenterY * Math.sin(currentPosition.getRotation().getRadians())) //appears that y coordinate isn't negative? strange.
        );
    }

    /**
     * calculates pure kinematics of stationary shot. all math used is available in a google doc in the shared drive
     * @param distance meters away as hypotenuse
     * @param heightDifference in meters, changes based on hub or cycling 
     * @param impactAngle angle we want our ball to hit
     * @return [angle, shotVelocity, timeOfFlight]
     */
    public static double[] calculateShooterStatic(double distance, double heightDifference, double impactAngle) {
        //calclates ball angle
        shooterPureAngle = Math.atan(
            ((2 * heightDifference) / distance) + (Math.tan(impactAngle))
        );
        staticShot[0] = shooterPureAngle;

        //calculates ball velocity
        shooterPureSpeed = Math.sqrt(
            (9.81 * distance * distance)
            /
            (
                (2 * Math.cos(shooterPureAngle) * Math.cos(shooterPureAngle)) * ((distance * Math.tan(shooterPureAngle)) - heightDifference)
            )
        );
        staticShot[1] = shooterPureSpeed;
        timeOfFlight = distance / (shooterPureSpeed * Math.cos(shooterPureAngle)); //we know we go x distance with a velocity, d =vx * t 
        staticShot[2] = timeOfFlight;
        return staticShot;
    }

    /**
     * Calculates if we can hit the target, and if so, what angle it has to be at. 
     * @param currentPose from odometry
     * @param target translation 2d of target
     * @return Turret angle in radians. 10 if we cannot hit the angle. 
     */
    public static double calculateTurret(Pose2d currentPose, Translation2d target) {
        //creates a range of values that our turret can be in, field relative
        double turretPositionAdjusted = currentPose.getRotation().getRadians(); //gets centerline of turret range of motion
        //0 is the front of robot, with ccw positive
        double turretMinAdj = turretPositionAdjusted + TurretConst.turretMin; //ah frick make sure not to double invert 
        double turretMaxAdj = turretPositionAdjusted + TurretConst.turretMax;
        
        //calculate position of robot to hub, need to add on robot rotation bc this is only based on raw odometry
        double robotRawToHub = Math.atan((target.getY() -  currentPose.getY())/(target.getX() -  currentPose.getX())); 
        SmartDashboard.putNumber("robotRawToHub", robotRawToHub);
        
        if((turretMinAdj < robotRawToHub) && (robotRawToHub < turretMaxAdj)) { //if within capabilities
            return (robotRawToHub - currentPose.getRotation().getRadians()); //used to be: currentPose.getRotation().getRadians() - robotRawToHub 
        } else {
            return 10; //ten is way outside the range of 6.28 radians (hopefully)
        }
    }

    //public static double  //WHAT WAS I GOING TO PUT HERE WHAT DID I FORGET
    
    /**
     * 
     * @param target target on the field to hit. make sure to subtract shooter height
     * @param currentPosition odometry position on the field
     * @param currentVelocities x, y velocities of the robot. 
     */
    public double[] shootOnTheMove(Translation3d target, Pose2d currentPose, ChassisSpeeds currentVelocities, double impactAngle) {
        double distance = Math.sqrt((target.getY() -  currentPose.getY()) + (target.getX() -  currentPose.getX()));
        double[] previousShot = Controller.calculateShooterStatic(distance, target.getZ(), impactAngle); //initial
        for (int i = 0; i < 10; i++) {
            previousShot = Controller.calculateShooterStatic(
                Math.sqrt((target.getY() -  currentPose.getY() + (currentVelocities.vyMetersPerSecond * previousShot[2])) + (target.getX() -  currentPose.getX() + (currentVelocities.vxMetersPerSecond * previousShot[2]))), 
                target.getZ(), 
                impactAngle);
        }
        return previousShot; 
    }

    public static double[] improvedSOTM() {
        return null;
    }
    
    public static double hypotenuseCalculator(Translation2d target, Translation2d position) {
        return (Math.sqrt(Math.pow((target.getX() - position.getX()), 2) + Math.pow((target.getY() - position.getY()), 2))); 
    }

}