package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConst;

public class Controller extends SubsystemBase {
    //vars for static shot
    private static double shooterPureSpeed = 0;
    private static double shooterPureAngle = 0;
    private static double timeOfFlight = 0;
    private static double[] staticShot = {0,0,0};
        
    public Controller() {
                
    }

    /**
     * calculates pure kinematics of stationary shot
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
     * @param turretPosition see const file but in radians it's the offset from the x axis
     * @param target translation 2d of target
     * @return Turret angle in radians. 10 if we cannot hit the angle. 
     */
    public static double calculateTurret(Pose2d currentPose, double turretPosition, Translation2d target) {
        //creates a range of values that our turret can be in, field relative
        double turretPositionAdjusted = currentPose.getRotation().getRadians(); //gets centerline of turret range of motion
        double turretMinAdj = turretPositionAdjusted - TurretConst.turretMin;
        double turretMaxAdj = turretPositionAdjusted + TurretConst.turretMax;
        
        //calculate position of robot to hub, need to add on robot rotation bc this is only based on raw odometry
        //double xDist = target.getX() -  currentPose.getX();
        //double yDist = target.getY() -  currentPose.getY();
        double robotRawToHub = Math.atan((target.getY() -  currentPose.getY())/(target.getX() -  currentPose.getX())); 

        if((turretMinAdj < robotRawToHub) && (robotRawToHub < turretMaxAdj)) {
            return (currentPose.getRotation().getRadians() - robotRawToHub); //angle from front of robot to hub
        } else {
            return 10; //ten is way outside the range of 6.28 radians (hopefully)
        }
    }
    /**
     * 
     * @param target target on the field to hit. make sure to subtract shooter height
     * @param currentPosition odometry position on the field
     * @param currentVelocities x, y velocities of the robot. technically not supposed to use this but see if i care
     */
    public double[] shootOnTheMove(Translation3d target, Pose2d currentPose, Translation2d currentVelocities, double impactAngle) {
        double distance = Math.sqrt((target.getY() -  currentPose.getY()) + (target.getX() -  currentPose.getX()));
        double[] previousShot = Controller.calculateShooterStatic(distance, target.getZ(), impactAngle); //initial
        for (int i = 0; i < 30; i++) {
            previousShot = Controller.calculateShooterStatic(
                Math.sqrt((target.getY() -  currentPose.getY() + (currentVelocities.getY() * previousShot[2])) + (target.getX() -  currentPose.getX() + (currentVelocities.getX() * previousShot[2]))), 
                target.getZ(), 
                impactAngle);
        }
        return previousShot;
    }

    
}