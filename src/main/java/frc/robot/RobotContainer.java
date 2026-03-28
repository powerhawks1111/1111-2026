// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CameraConst;
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

  private final Vision camLeft = new Vision(CameraConst.pvCamOne, new Transform3d());
  private final Vision camRight = new Vision(CameraConst.pvCamTwo, new Transform3d());

  private final Intake m_intake = new Intake();
  private final Spindexer m_spindexer = new Spindexer();
  private final Kicker m_kicker = new Kicker();
  private final Turret m_turret = new Turret();
  private final Hood m_hood = new Hood();
  private final Flywheel m_flywheel = new Flywheel();
  private final Controller m_controller = new Controller();
  private final CommandXboxController m_driver = new CommandXboxController(0);
  private final CommandXboxController m_operator = new CommandXboxController(1);

  private final Translation2d our_hub;

  private final SendableChooser<Command> autoChooser;
    
  private final StructSubscriber<Pose2d> poseSub = NetworkTableInstance.getDefault()
      .getStructTopic("Robot/CurrentPose", Pose2d.struct).subscribe(new Pose2d());
  
    public RobotContainer() {
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
      var alliance = DriverStation.getAlliance();
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
      

      SmartDashboard.putNumber("X", 0);
      SmartDashboard.putNumber("Y", 0);
      SmartDashboard.putNumber("ROT", 0);

      configureBindings();
    }
  
    private void configureBindings() {
      m_driver.button(1).onTrue(resetNavX());

      m_driver.button(2).onTrue(resetOdometry(new Pose2d()));
      
      m_drivetrain.setDefaultCommand(
        Commands.run(
          () -> m_drivetrain.drive(
            -m_driver.getRawAxis(1), 
            -m_driver.getRawAxis(0), 
            -m_driver.getRawAxis(4), 
            4, 2), 
          m_drivetrain)
      );

      m_operator.button(6).toggleOnFalse(m_intake.stopRollers());
      m_operator.button(6).toggleOnTrue(m_intake.runRollers(false));

/*    TODO: Need to decide whether to use this or not 
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
 */    

      //m_operator.button(1).toggleOnFalse(runFlywheel(0));
      //m_operator.button(1).toggleOnTrue(runFlywheel(4000));
      //m_operator.button(2).whileFalse(runKicker()).whileTrue(runKicker());
      //m_operator.button(2).whileFalse(runSpindexer(0)).whileTrue(runSpindexer(.8));
      //positonHood(0);

    //m_operator.button(1).whileTrue(runKicker(.5).alongWith(runSpindexer(4))).whileFalse(runKicker(0).alongWith(runSpindexer(0)));
    //m_operator.button(2).whileTrue(runFlywheel(SmartDashboard.getNumber("RPM", 0))); 
      //m_driver.button(1).onTrue();  
      
      // m_driver.button(2).onTrue();
      
      // m_driver.button(3).onTrue();
  }

  public void shootIntegrated() {
    double[] input = Controller.getShooterSimple(
      Feet.convertFrom(
        Controller.hypotenuseCalculator(
          our_hub, 
          m_drivetrain.getEstimatedPose().getTranslation()), 
          Meters)
    );
    double turretSetpoint = Controller.calculateTurret(m_drivetrain.getEstimatedPose(), our_hub);
    if (turretSetpoint < 10) {
      m_turret.adjustTurret(turretSetpoint);
    }
    m_flywheel.setSpeed(input[0]);
    m_hood.adjustHood(input[1]);

  }

  public void test() {
    double[] input = m_controller.getShooterSimple(7.75);
    SmartDashboard.putNumber("RPM", input[0]);
    SmartDashboard.putNumber("Hood", input[1]);

    SmartDashboard.putNumber("Turret Angle Degrees", 
      Math.toDegrees(Controller.calculateTurret(new Pose2d(new Translation2d(
        SmartDashboard.getNumber("X", 0),
        SmartDashboard.getNumber("Y", 0)), 
        new Rotation2d(
          SmartDashboard.getNumber("ROT", 0)
        ))
        , FIELD_CONST.BLUE_HUB)
    ));
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
      Pose2d m_pose = estimateLeft.get().estimatedPose.toPose2d();
      SmartDashboard.putNumber("X value LEFT", m_pose.getX());
      SmartDashboard.putNumber("Y value LEFT", m_pose.getY());
      SmartDashboard.putNumber("Rot value LEFT", m_pose.getRotation().getDegrees());
      m_drivetrain.updatePoseWithVision(estimateLeft.get());
    }

    Optional<EstimatedRobotPose> estimateRight = camRight.EstimatePose();
    if (estimateRight.isPresent()) {
      Pose2d m_pose = estimateRight.get().estimatedPose.toPose2d();
      SmartDashboard.putNumber("X value RIGHT", m_pose.getX());
      SmartDashboard.putNumber("Y value RIGHT", m_pose.getY());
      SmartDashboard.putNumber("Rot value RIGHT", m_pose.getRotation().getDegrees());
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

  // public Command shootOnTheMoveIntoHub(boolean blueAlliance) {
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
  //     55
  //     );

  //   Translation2d virtTarget = new Translation2d(
  //     hub.getX() + (m_drivetrain.getFieldRelativeSpeeds().vxMetersPerSecond * shotData[2]), (m_drivetrain.getFieldRelativeSpeeds().vyMetersPerSecond * shotData[2])
  //   );
  //   double turretPosition = Controller.calculateTurret(m_pose, hub); //have to replace hub w/ virt target
    
  // }

}
