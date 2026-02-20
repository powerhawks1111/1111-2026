// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.CameraConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Vision m_vision = new Vision();
  private final AutoFactory autoFactory;
  private final CommandPS4Controller m_driverController = new CommandPS4Controller(0);
    
    private final StructSubscriber<Pose2d> poseSub = NetworkTableInstance.getDefault()
      .getStructTopic("Robot/CurrentPose", Pose2d.struct).subscribe(new Pose2d());
  
    public RobotContainer() {
      autoFactory = new AutoFactory(
        m_drivetrain::getEstimatedPose, // A function that returns the current robot pose
        m_drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
        m_drivetrain::followTrajectory, // The drive subsystem trajectory follower 
        true, // If alliance flipping should be enabled 
        m_drivetrain 
      );
  
      configureBindings();
    }
  
    private void configureBindings() {
      m_drivetrain.setDefaultCommand(
      Commands.run(
        () -> m_drivetrain.drive(
          -m_driverController.getRawAxis(1), 
          -m_driverController.getRawAxis(0), 
          -m_driverController.getRawAxis(4), 
          true), 
        m_drivetrain)
    );

    m_driverController.button(1).onTrue(
      Commands.runOnce(() -> m_drivetrain.resetNavx(), m_drivetrain)
    );

    m_driverController.button(2).onTrue(
      Commands.runOnce(() -> m_drivetrain.resetPose(new Pose2d(3,3, new Rotation2d())), m_drivetrain)
    );

    m_driverController.button(3).onTrue(
      m_drivetrain.pathfind(new Pose2d(3,3,new Rotation2d())) //resets to 0,0,0 hopefully 
    );

    m_driverController.button(4).onTrue(
      m_drivetrain.simpleAutoMove(new Pose2d(3, 3, new Rotation2d())) //resets to 0,0,0
    );

  }

  public void test() {
    SmartDashboard.putBoolean("seesTag ", m_vision.tagInSight());
    SmartDashboard.putNumber("Vision X", m_vision.getPoseMultiTag().getX());
    SmartDashboard.putNumber("Vision Y", m_vision.getPoseMultiTag().getY());
    SmartDashboard.putNumber("Vision Rot", m_vision.getPoseMultiTag().getRotation().getDegrees());
  }

  public void updateTelemetry() {
    SmartDashboard.putNumber("OdometryX", poseSub.get().getX());
    SmartDashboard.putNumber("OdometryY", poseSub.get().getTranslation().getY()); //TODO for some reason this is backwards?
    SmartDashboard.putNumber("OdometryRot", poseSub.get().getRotation().getDegrees());
  }

   public Command getAutonomousCommand() {
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile("jack");
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }

    //return autoFactory.trajectoryCmd("SquarePath");
  }

}
