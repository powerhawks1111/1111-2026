// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CameraConst;
import frc.robot.Constants.FIELD_CONST;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootWithRange;
import frc.robot.commands.StopShootCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter.Controller;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.Turret;

public class RobotContainer {

  private final Drivetrain m_drivetrain = new Drivetrain();

  private final Vision camLeft = new Vision(CameraConst.pvCamOne, CameraConst.leftCamTransform);
  private final Vision camRight = new Vision(CameraConst.pvCamTwo, CameraConst.rightCamTransform);


  // private final Intake m_intake = new Intake();
  // private final Spindexer m_spindexer = new Spindexer();
  // private final Kicker m_kicker = new Kicker();
  // private final Turret m_turret = new Turret();
  // private final Hood m_hood = new Hood();
  // private final Flywheel m_flywheel = new Flywheel();
  private final CommandXboxController m_driver = new CommandXboxController(0);
  private final CommandXboxController m_operator = new CommandXboxController(1);
  private final Optional<DriverStation.Alliance> alliance;

  private final Translation2d our_hub;
  
  private final Field2d m_robotField = new Field2d();
  private final Field2d m_turretField = new Field2d();

  //private final SendableChooser<Command> autoChooser;
  
    public RobotContainer() {
      //autoChooser = AutoBuilder.buildAutoChooser();
      //SmartDashboard.putData("Auto Chooser", autoChooser);
      
      alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          boolean isBlueAlliance = alliance.get() == DriverStation.Alliance.Blue;
          if (isBlueAlliance) {
            our_hub = FIELD_CONST.BLUE_HUB;
          } else {
            our_hub = FIELD_CONST.RED_HUB;
          }
      } else {
        our_hub = FIELD_CONST.BLUE_HUB;
      }
    SmartDashboard.putString("Alliance Detected In Code", alliance.toString());

    //block of inputs needed from SmartDashboard for simulation. can delete after ensuring success.
    SmartDashboard.putNumber("Robot Position X (Meters)", 0);
    SmartDashboard.putNumber("Robot Position Y (Meters)", 0);
    SmartDashboard.putNumber("Robot Velocity X (Meters)", 0);
    SmartDashboard.putNumber("Robot Velocity Y (Meters)", 0);
    SmartDashboard.putNumber("Robot Rotation (Radians)", 0);

    configureBindings();
  }
  
  private void configureBindings() {
        
  }

  public void simulate() {
    //will eventually switch to below, for now need to simulate
    //Pose2d currentPose = m_drivetrain.getEstimatedPose();
    Pose2d currentPose = new Pose2d(
      SmartDashboard.getNumber("Robot Position X (Meters)", 3),
      SmartDashboard.getNumber("Robot Position Y (Meters)", 3),
      new Rotation2d(SmartDashboard.getNumber("Robot Rotation (Radians)", 0))
    );

    Pose2d turretPose = new Pose2d(
      Controller.robotToTurretTranslation(currentPose),
      currentPose.getRotation()
    );
    

    m_robotField.setRobotPose(currentPose);
    SmartDashboard.putData("Robot Field", m_robotField);


    SmartDashboard.putNumber(
      "Robot Hypotenuse To Target (Meters)", 
      Controller.hypotenuseCalculator(currentPose.getTranslation(), our_hub)
    );
    
    SmartDashboard.putNumber(
      "Turret Hypotenuse To Target (Meters)", 
      Controller.hypotenuseCalculator(
        Controller.robotToTurretTranslation(currentPose), our_hub)
    );

    SmartDashboard.putNumber(
      "Turret Hypotenuse To Target (Feet)", 
      Units.metersToFeet(
        Controller.hypotenuseCalculator(
        Controller.robotToTurretTranslation(currentPose), our_hub)
      )
    );

    SmartDashboard.putNumber(
      "Flywheel Velocity Final (Feet/Second)", 
      0
    );

    SmartDashboard.putNumber(
      "Hood Angle Pure (Degrees)", 
      0
    );

    SmartDashboard.putNumber(
      "Turret Position X (Meters)", 
      Controller.robotToTurretTranslation(currentPose).getX()
    );

    SmartDashboard.putNumber(
      "Turret Position Y (Meters)", 
      Controller.robotToTurretTranslation(currentPose).getY()
    );

    SmartDashboard.putNumber(
      "Turret Angle (Radians)", 
      Controller.calculateTurret(turretPose, our_hub)
    );
    
    m_turretField.setRobotPose(
      new Pose2d(
        turretPose.getTranslation(),
        new Rotation2d(Controller.calculateTurret(turretPose, our_hub) + turretPose.getRotation().getRadians())
      )
    );
    SmartDashboard.putData("Turret Field", m_turretField);
  }

  public void updateVision() {
    // Optional<EstimatedRobotPose> estimate = m_vision.EstimatePose();
    // if (estimate.isPresent()) {
    //   Pose2d m_pose = estimate.get().estimatedPose.toPose2d();
    //   SmartDashboard.putNumber("X value", m_pose.getX());
    //   SmartDashboard.putNumber("Y value", m_pose.getY());
    //   SmartDashboard.putNumber("Rot value", m_pose.getRotation().getDegrees());
    //   m_drivetrain.updatePoseWithVision(estimate.get());
    // }

    Optional<EstimatedRobotPose> estimateLeft = camLeft.EstimatePose();
    if (estimateLeft.isPresent()) {
      m_drivetrain.updatePoseWithVision(estimateLeft.get());
    }

    Optional<EstimatedRobotPose> estimateRight = camRight.EstimatePose();
    if (estimateRight.isPresent()) {
      m_drivetrain.updatePoseWithVision(estimateRight.get());
    }
  }

  // public Command getAutonomousCommand() {
  //   return autoChooser.getSelected();
  // }

  // public Command resetOdometry(Pose2d pose) {
  //   return Commands.runOnce(() -> m_drivetrain.resetPose(pose), m_drivetrain);
  // }

  // public Command resetNavX() {
  //   return Commands.runOnce(() -> m_drivetrain.resetNavx(), m_drivetrain);
  // }
}
