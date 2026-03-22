// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.FIELD_CONST;
import frc.robot.Constants.IntakeConst;
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
  private final Vision m_vision = new Vision();
  //private final Intake m_intake = new Intake();
  //private final Spindexer m_spindexer = new Spindexer();
  private final Kicker m_kicker = new Kicker();
  //private final Turret m_turret = new Turret();
  private final Hood m_hood = new Hood();
  private final Flywheel m_flywheel = new Flywheel();
  private final Controller m_controller = new Controller();


  private final SendableChooser<Command> autoChooser;
  private final CommandPS4Controller m_driverController = new CommandPS4Controller(0);
    
    private final StructSubscriber<Pose2d> poseSub = NetworkTableInstance.getDefault()
      .getStructTopic("Robot/CurrentPose", Pose2d.struct).subscribe(new Pose2d());
  
    public RobotContainer() {
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);

      SmartDashboard.putNumber("KickerSpeed", 0);
      SmartDashboard.putNumber("Hood", 0);
      SmartDashboard.putNumber("RPM", 0);
      configureBindings();
    }
  
    private void configureBindings() {
      m_drivetrain.setDefaultCommand(
        Commands.run(
          () -> m_drivetrain.drive(
            -m_driverController.getRawAxis(1), 
            -m_driverController.getRawAxis(0), 
            -m_driverController.getRawAxis(4), 
            2.5, 2), 
          m_drivetrain)
      );
      m_driverController.axisGreaterThan(3, .5).whileTrue( //right trigger find it
        Commands.run(
          () -> m_drivetrain.drive(
            0, //.getRawAxis(1), 
            0, //m_driverController.getRawAxis(0), 
            m_drivetrain.rotLock(
              m_driverController.getRawAxis(4), 
              -m_driverController.getRawAxis(5)), 
          0, 0), m_drivetrain)
      );
    }

  public void updateVision() {
    Optional<EstimatedRobotPose> estimate = m_vision.EstimatePose();
    if (estimate.isPresent()) {
      Pose2d m_pose = estimate.get().estimatedPose.toPose2d();
      SmartDashboard.putNumber("X value", m_pose.getX());
      SmartDashboard.putNumber("Y value", m_pose.getY());
      SmartDashboard.putNumber("Rot value", m_pose.getRotation().getDegrees());
      m_drivetrain.updatePoseWithVision(estimate.get());
    }
  }

  public void resetOdom() {
    if (m_driverController.button(1).getAsBoolean()) {
      m_drivetrain.resetPose(new Pose2d(12.947, 0.571, new Rotation2d(Math.PI)));
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void runShooterTesting() {
    m_kicker.setSameSpeed(
      SmartDashboard.getNumber("KickerSpeed", 0)
    );
    m_hood.adjustHood(
      SmartDashboard.getNumber("Hood", 0)
    );
    m_flywheel.runFlywheel(
      SmartDashboard.getNumber("RPM", 0)
    );
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  }
  
  //TELEOP COMMANDS ALL THE WAY DOWN
  //flip to Command.run() instead of .runOnce() ?

  // //INTAKE
  // public Command deployIntake() {
  //   return Commands.runOnce(() -> m_intake.setFlip(0), m_intake);
  // }
  // public Command retractIntake() {
  //   return Commands.runOnce(() -> m_intake.setFlip(0), m_intake);
  // }
  // public Command runRollers(boolean reversed) {
  //   if(reversed) {
  //   return Commands.runOnce(() -> m_intake.setRollerSpeed(IntakeConst.rollerSpeed), m_intake);
  //   } else {
  //   return Commands.runOnce(() -> m_intake.setRollerSpeed(-IntakeConst.rollerSpeed), m_intake);
  //   }
  // }
  // public Command stopRollers() {
  //   return Commands.runOnce(() -> m_intake.setRollerSpeed(0), m_intake);
  // }

  // //SPINDEXER
  // public Command runSpindexer(double bps) {
  //   return Commands.runOnce(() -> m_spindexer.runSpindexer(bps), m_spindexer);
  // }

  // //KICKER
  // public Command runKicker(double speed) {
  //   return Commands.runOnce(() -> m_kicker.setSameSpeed(speed), m_kicker);
  // }
  // //SHOOTER
  // public Command positionTurret(double position) {
  //   return Commands.runOnce(() -> m_turret.adjustTurret(position), m_turret);
  // }
  // public Command positonHood(double position) {
  //   return Commands.runOnce(() -> m_hood.adjustHood(position), m_hood);
  // }
  // public Command runFlywheel(double speed) {
  //   return Commands.runOnce(() -> m_flywheel.runFlywheel(speed), m_flywheel);
  // }
  
  // public Command shootFromBaseOfHub() {
  //   return Commands.parallel(
  //     positionTurret(0), positonHood(0), runFlywheel(0)
  //   );
  // }

  // public Command shootStaticIntoHub(boolean blueAlliance) {
  //   Pose2d m_pose = poseSub.get();
  //   double distance = 0;
  //   Translation2d hub = new Translation2d();
  //   if(blueAlliance) {
  //     distance = Controller.hypotenuseCalculator(FIELD_CONST.BLUE_HUB, m_pose.getTranslation());
  //     hub = FIELD_CONST.BLUE_HUB;
  //   } else {
  //     distance = Controller.hypotenuseCalculator(FIELD_CONST.RED_HUB, m_pose.getTranslation());
  //     hub=FIELD_CONST.RED_HUB;
  //   }
  //   double[] shotData = Controller.calculateShooterStatic( //double distance, double heightDifference, double impactAngle
  //     distance, 
  //     FIELD_CONST.HUB_SHOOTER_DIFFERENCE, 
  //     65
  //     );
  //   double turretPosition = Controller.calculateTurret(m_pose, hub);
    
  //   return Commands.parallel(
  //     positonHood(shotData[0]),
  //     runFlywheel(shotData[1]), 
  //     positionTurret(turretPosition)
  //   );

  // }

}
