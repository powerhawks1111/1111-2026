package frc.robot.subsystems;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Constants.DriveConst;
import frc.robot.Constants.DrivetrainConst;
import frc.robot.Constants.TrajectoryConst;
import com.studica.frc.*;
import com.studica.frc.AHRS.NavXComType;;

public class Drivetrain extends SubsystemBase{
    private final Translation2d m_frontLeftLocation = new Translation2d(DrivetrainConst.halfSideLength, DrivetrainConst.halfSideLength);
    private final Translation2d m_frontRightLocation = new Translation2d( DrivetrainConst.halfSideLength, -DrivetrainConst.halfSideLength);
    private final Translation2d m_backLeftLocation = new Translation2d(-DrivetrainConst.halfSideLength,  DrivetrainConst.halfSideLength); 
    private final Translation2d m_backRightLocation = new Translation2d( -DrivetrainConst.halfSideLength, -DrivetrainConst.halfSideLength);

    private final SwerveModule m_frontLeft = new SwerveModule(DrivetrainConst.FLDrive, DrivetrainConst.FLTURN); 
    private final SwerveModule m_frontRight = new SwerveModule(DrivetrainConst.FRDrive, DrivetrainConst.FRTurn); 
    private final SwerveModule m_backRight = new SwerveModule(DrivetrainConst.BRDrive, DrivetrainConst.BRTurn); 
    private final SwerveModule m_backLeft = new SwerveModule(DrivetrainConst.BLDrive, DrivetrainConst.BLTurn); 

    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI); //ensure "spi" is switched "on" on navx2
    
    private SwerveModulePosition[] m_positions = {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()};
    private SwerveModuleState[] m_swerveModuleStates;
    private SwerveModuleState[] m_robotState = {m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState()}; //needed for pathplanner
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    //pose controllers for choreo + misc. wpilib tasks. pathplanner uses own controllers, same kP kI kD though. probably.
    private final PIDController xTranslationController = new PIDController(TrajectoryConst.kPT, TrajectoryConst.kIT, TrajectoryConst.kDT);
    private final PIDController yTranslationController = new PIDController(TrajectoryConst.kPT, TrajectoryConst.kIT, TrajectoryConst.kDT);
    private final PIDController rotController = new PIDController(TrajectoryConst.kPRot, TrajectoryConst.kIRot, TrajectoryConst.kDRot); //different from other controller bc continuous input is from -Pi to PI
    
    private final PIDController choreoTranslationController = new PIDController(TrajectoryConst.kPT, TrajectoryConst.kIT, TrajectoryConst.kDT);
    private final PIDController choreoRotController = new PIDController(TrajectoryConst.kPRot, TrajectoryConst.kIRot, TrajectoryConst.kDRot); //different from other controller bc continuous input is from -Pi to PI
    
    private final SwerveDrivePoseEstimator m_PoseEstimator;
    private final StructPublisher<Pose2d> posePub = NetworkTableInstance.getDefault()
        .getStructTopic("Robot/CurrentPose", Pose2d.struct).publish(); //use as template for publishing data to NetworkTables.
    private final Field2d m_field = new Field2d();

    public Drivetrain() {
        m_PoseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics, 
            navx.getRotation2d(), 
            m_positions, new Pose2d()); //TODO add lookup from SmartDashboard based on starting auto path

        rotController.enableContinuousInput(-Math.PI, Math.PI); //for choreo

         try {
            AutoBuilder.configure(
                this::getEstimatedPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveWithChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(TrajectoryConst.kPT, TrajectoryConst.kIT, TrajectoryConst.kDT), // Translation PID constants
                        new PIDConstants(TrajectoryConst.kPRot, TrajectoryConst.kIRot, TrajectoryConst.kDRot) // Rotation PID constants
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
        posePub.set(m_PoseEstimator.getEstimatedPosition());

        m_field.setRobotPose(m_PoseEstimator.getEstimatedPosition());
        SmartDashboard.putData(m_field);
    }

    //drives based on manual input
    public void drive(double x, double y, double rot, boolean fieldRelative) { 
        x *= DrivetrainConst.kMaxVelocity;
        y *= DrivetrainConst.kMaxVelocity;
        rot *= DrivetrainConst.kMaxChassisRotsPerSecond;
        if (fieldRelative) {
            m_swerveModuleStates = m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, navx.getRotation2d())
            );  
        } else {
            m_swerveModuleStates = m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromRobotRelativeSpeeds(x, y, rot, navx.getRotation2d())
            );  
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates, DrivetrainConst.kMaxVelocity);
            m_frontLeft.setDesiredState(m_swerveModuleStates[0]);
            m_frontRight.setDesiredState(m_swerveModuleStates[1]);
            m_backLeft.setDesiredState(m_swerveModuleStates[2]);
            m_backRight.setDesiredState(m_swerveModuleStates[3]);
    }

    //simplest method for driving, used for auto
    public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        m_swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates, DrivetrainConst.kMaxVelocity);
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

    //for Choreo specifically
    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = m_PoseEstimator.getEstimatedPosition();
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + choreoTranslationController.calculate(pose.getX(), sample.x),
            sample.vy + choreoTranslationController.calculate(pose.getY(), sample.y),
            sample.omega + choreoRotController.calculate(pose.getRotation().getRadians(), sample.heading)
        );
        driveWithChassisSpeeds(speeds);
    }
    //TODO if we have time, tune kalman filter
    public Pose2d getEstimatedPose() {
        return m_PoseEstimator.getEstimatedPosition();
    }

    //TODO call whenever we get apriltag data
    public void resetPose(Pose2d pose) {
        m_PoseEstimator.resetPose(pose);
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

    //TODO fix for field flipping, then fix for concatenating additional paths
    //flipped if on red alliance
    public Command pathfind(Pose2d targetPose, boolean isFlipped) {
        PathConstraints pathConstraints = new PathConstraints(
            DrivetrainConst.kMaxVelocity, 
            DrivetrainConst.kMaxAccel, 
            DrivetrainConst.kMaxChassisRotsPerSecond, 
            DrivetrainConst.kMaxChassisRotsPerSecondPerSecond
        );
        return AutoBuilder.pathfindToPose(targetPose, pathConstraints, 0.0);
    }

    /*
     * This is meant for very simple auto alignment. This means that the only thing we need is our "target pose", which is really just 
     * a measure of how much error we have from where we want to be. This is driven entirely from robot-relative frames, but depending on
     * where we place our Limelight camera, we may need to flip axes and/or signs. 
     */
    public Command simpleAutoMove(Pose2d targetPose) {
        return Commands.run(
            () -> driveWithChassisSpeeds(
                new ChassisSpeeds(
                    xTranslationController.calculate(targetPose.getX(), 0),
                    yTranslationController.calculate(targetPose.getY(), 0),
                    rotController.calculate(targetPose.getRotation().getDegrees(), 0) //NOTE: Drive is CCW positive (or it should be)
                )
            ), this //the "this" may not be necessary
        )
        .repeatedly() //we repeat command until interrupted
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf); //we cancel ourself because if another drive command is called, it's because that's the most recent desired output.
    }
}
