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
  //private final CommandXboxController m_operator = new CommandXboxController(1);
  private final Optional<DriverStation.Alliance> alliance;

  private final Translation2d our_hub;

  private final SendableChooser<Command> autoChooser;
  
    public RobotContainer() {
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);

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

    configureBindings();
  }
  
  private void configureBindings() {
        m_drivetrain.setDefaultCommand(
          Commands.run(
            () -> m_drivetrain.drive(
            -m_driver.getRawAxis(1), 
            -m_driver.getRawAxis(0), 
            -m_driver.getRawAxis(4), 4, 2), 
            m_drivetrain)
        );

        m_driver.a().onTrue(
          resetNavX()
        );
        m_driver.b().onTrue(
          resetOdometry(new Pose2d())
        );
        
  }

  public void updateVision() {
    Optional<EstimatedRobotPose> estimateLeft = camLeft.EstimatePose();
    if (estimateLeft.isPresent()) {
      m_drivetrain.updatePoseWithVision(estimateLeft.get());
    }

    Optional<EstimatedRobotPose> estimateRight = camRight.EstimatePose();
    if (estimateRight.isPresent()) {
      m_drivetrain.updatePoseWithVision(estimateRight.get());
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command resetOdometry(Pose2d pose) {
    return Commands.runOnce(() -> m_drivetrain.resetPose(pose), m_drivetrain);
  }

  public Command resetNavX() {
    return Commands.runOnce(() -> m_drivetrain.resetNavx(), m_drivetrain);
  }

}
