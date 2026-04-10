package frc.robot.subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.DriveConst;
import frc.robot.Constants.DrivetrainConst;
import frc.robot.Constants.TrajectoryConst;
import frc.robot.subsystems.Shooter.Controller;

import com.studica.frc.*;
import com.studica.frc.AHRS.NavXComType;;

public class Drivetrain extends SubsystemBase{
    private final Translation2d m_frontLeftLocation = new Translation2d(DrivetrainConst.halfSideLength, DrivetrainConst.halfSideLength);
    private final Translation2d m_frontRightLocation = new Translation2d( DrivetrainConst.halfSideLength, -DrivetrainConst.halfSideLength);
    private final Translation2d m_backLeftLocation = new Translation2d(-DrivetrainConst.halfSideLength,  DrivetrainConst.halfSideLength); 
    private final Translation2d m_backRightLocation = new Translation2d( -DrivetrainConst.halfSideLength, -DrivetrainConst.halfSideLength);

    private final SwerveModule m_frontLeft = new SwerveModule(CANID.FLDrive, CANID.FLTURN); 
    private final SwerveModule m_frontRight = new SwerveModule(CANID.FRDrive, CANID.FRTurn); 
    private final SwerveModule m_backRight = new SwerveModule(CANID.BRDrive, CANID.BRTurn); 
    private final SwerveModule m_backLeft = new SwerveModule(CANID.BLDrive, CANID.BLTurn); 

    private final static AHRS navx = new AHRS(NavXComType.kUSB1); //ensure "spi" is switched "on" on navx2
        
        private SwerveModulePosition[] m_positions = {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()};
        private SwerveModuleState[] m_swerveModuleStates;
        private SwerveModuleState[] m_robotState = {m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState()}; //needed for pathplanner
        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    
        private final SwerveDrivePoseEstimator m_PoseEstimator;
        private final StructPublisher<Pose2d> posePub = NetworkTableInstance.getDefault()
            .getStructTopic("Robot/CurrentPose", Pose2d.struct).publish(); //use as template for publishing data to NetworkTables.
        private final Field2d m_field = new Field2d();
        private static PIDController m_rotLockController;
            
                public Drivetrain() {
                    m_rotLockController = new PIDController(DriveConst.autoLockP, DriveConst.autoLockP, DriveConst.autoLockP);
                m_rotLockController.enableContinuousInput(0, 2 * Math.PI);
                m_rotLockController.setTolerance(0.0872665); //five degrees 
        
                m_PoseEstimator = new SwerveDrivePoseEstimator(
                    m_kinematics, 
                    navx.getRotation2d(), 
                    m_positions, new Pose2d()
                ); //TODO add lookup from SmartDashboard based on starting auto path
        
                 try {
                    AutoBuilder.configure(
                        this::getEstimatedPose, // Robot pose supplier
                        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                        (speeds, feedforwards) -> driveWithChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                                new PIDConstants(TrajectoryConst.kPTPP, TrajectoryConst.kITPP, TrajectoryConst.kDTPP), // Translation PID constants
                                new PIDConstants(TrajectoryConst.kPRotPP, TrajectoryConst.kIRotPP, TrajectoryConst.kDRotPP) // Rotation PID constants
                        ),
                        RobotConfig.fromGUISettings(), // The robot configuration
                        () -> {
                          var alliance = DriverStation.getAlliance();
                          if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                          }
                          return false;
                        },
                        this // Reference to this subsystem to set requirements
                    );
                } catch (IOException e) {
                    e.printStackTrace();
                } catch (ParseException e) {
                    e.printStackTrace();
                }
            }
            @Override
            public void periodic() {
                m_positions[0] = m_frontLeft.getPosition();
                m_positions[1] = m_frontRight.getPosition();
                m_positions[2] = m_backLeft.getPosition();
                m_positions[3] = m_backRight.getPosition();
                m_PoseEstimator.update(
                    navx.getRotation2d(), 
                    m_positions
                );
                SmartDashboard.putNumber("navx value: ", navx.getRotation2d().getDegrees());
                posePub.set(m_PoseEstimator.getEstimatedPosition());
                
        
                m_field.setRobotPose(m_PoseEstimator.getEstimatedPosition());
        
                // PathPlannerLogging.setLogCurrentPoseCallback(
                // (pose) -> {
                // m_field.setRobotPose(pose);
                // });
                
                SmartDashboard.putData(m_field);
                SmartDashboard.putNumber("Pose Est X", m_PoseEstimator.getEstimatedPosition().getX());
                SmartDashboard.putNumber("Pose Est Y", m_PoseEstimator.getEstimatedPosition().getY());
                SmartDashboard.putNumber("Pose Est Heading", m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
                }
        
            /**
             * Method to drive robot in tele mode; can vary speed, esp based on odometry
             * @param x desired x output (0-1, scaled in method)
             * @param y desired y output (0-1, scaled in method)
             * @param rot desired rot output (0-1, scaled in method)
             * @param maxAllowedVelocity max velocity allowed for this run. this is not max overall velocity, just what we want it to have
             * @param maxRotsPerSec (input max rots per sec, we multiply by 2 pi in method)
             */
            public void drive(double x, double y, double rot, double maxAllowedVelocity, double maxRotsPerSec) { 
                x *= maxAllowedVelocity;
                y *= maxAllowedVelocity;
                rot *= (maxRotsPerSec * Math.PI * 2);
        
                m_swerveModuleStates = m_kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, navx.getRotation2d())
                );  
        
                SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates, DrivetrainConst.kMaxVelocity);
                    m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
                    m_frontRight.setDesiredState(m_swerveModuleStates[1]);
                    m_backLeft.setDesiredState(m_swerveModuleStates[2]);
                    m_backRight.setDesiredState(m_swerveModuleStates[3]);
            }
        
            //simplest method for driving, used for auto
            public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds) {
                m_swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
                //SwerveDriveKinematics.desaturateWheelSpeed(m_swerveModuleStates, DrivetrainConst.kMaxVelocity);
                m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
                m_frontRight.setDesiredState(m_swerveModuleStates[1]);
                m_backLeft.setDesiredState(m_swerveModuleStates[2]);
                m_backRight.setDesiredState(m_swerveModuleStates[3]);
            }
        
            public ChassisSpeeds getChassisSpeeds() {
                m_robotState[0] = m_frontLeft.getState();
                m_robotState[1] = m_frontRight.getState();
                m_robotState[2] = m_backLeft.getState();
                m_robotState[3] = m_backRight.getState();
                return m_kinematics.toChassisSpeeds(m_robotState); //look here if thing break - could be that this is returning desired states and not actual ones.
            }
        
            public ChassisSpeeds getFieldRelativeSpeeds() {
                double navxHeading = (navx.getRotation2d().getRadians());
                m_robotState[0] = m_frontLeft.getState();
                m_robotState[1] = m_frontRight.getState();
                m_robotState[2] = m_backLeft.getState();
                m_robotState[3] = m_backRight.getState();
                ChassisSpeeds robotRelative = m_kinematics.toChassisSpeeds(m_robotState); //look here if thing break - could be that this is returning desired states and not actual ones.
                return new ChassisSpeeds(
                    robotRelative.vxMetersPerSecond * Math.cos(navxHeading),
                    robotRelative.vyMetersPerSecond * Math.sin(navxHeading),
                    0 //wont calculate for now 
                );
            }
        
            public Pose2d getEstimatedPose() {
                return m_PoseEstimator.getEstimatedPosition();
            }
        
            public void updatePoseWithVision(EstimatedRobotPose pose) {
                m_PoseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
            }
        
            //resets without any recalibration, use this method because it's very fast. 
            public void resetNavx() {
                navx.reset();
                //MUST call resetPosition after resetting the NavX: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html 
                m_PoseEstimator.resetPose(
                    new Pose2d(
                        m_PoseEstimator.getEstimatedPosition().getX(),
                        m_PoseEstimator.getEstimatedPosition().getX(),
                        navx.getRotation2d()
                    )
                );
            }
        
            public void resetPose(Pose2d pose) {
                m_PoseEstimator.resetPose(pose);
            }
        
            //TODO fix for field flipping, then fix for concatenating additional paths
            //flipped if on red alliance
            public Command pathfind(Pose2d targetPose) {
                PathConstraints pathConstraints = new PathConstraints(
                    DrivetrainConst.kMaxVelocity, 
                    DrivetrainConst.kMaxAccel, 
                    DrivetrainConst.kMaxChassisRotsPerSecond, 
                    DrivetrainConst.kMaxChassisRotsPerSecondPerSecond
                );
                return AutoBuilder.pathfindToPose(targetPose, pathConstraints, 0.1);
            }
            
    public Command aimDrivetrainCommand(Translation2d position, Translation2d target) {
        return this.runEnd(
            () -> drive(0, 0, 
            m_rotLockController.calculate(
                navx.getRotation2d().getRadians(), 
                Math.atan(
                    (target.getY() - position.getY())
                    /
                    (target.getX() - position.getX())
                )
            )
            , 10, 10),
            () -> drive(0, 0, 0, 0, 0)
        );
    }


}