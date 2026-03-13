package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Controller {

    //vars for static shot
    private double shooterPureSpeed = 0;
    private double shooterPureAngle = 0;
    private double timeOfFlight = 0;
    private double[] staticShot = {0,0,0};
    
        public Controller() {
            
        }
    
        /**
         * calculates pure kinematics of stationary shot
         * @param distance meters away as hypotenuse
         * @param heightDifference in meters, changes based on hub or cycling 
         * @param impactAngle angle we want our ball to hit
         * @return [angle, shotVelocity, timeOfFlight]
         */
        public double[] calculateShooterStatic(double distance, double heightDifference, double impactAngle) {
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

}
