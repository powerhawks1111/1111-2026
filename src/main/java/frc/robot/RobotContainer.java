// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.spark.config.SparkMaxConfig;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.units.measure.Angle;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CameraConst;
import frc.robot.Constants.FIELD_CONST;
import frc.robot.Constants.IntakeConst;
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

//  private final Vision caml = new Vision(CameraConst.LeftCamName, CameraConst.leftCamTransform);
  private final Vision camR = new Vision(CameraConst.RightCamName, CameraConst.rightCamTransform);

  private final Intake m_intake = new Intake();
  private final Spindexer m_spindexer = new Spindexer();
  private final Kicker m_kicker = new Kicker();
  //private final Turret m_turret = new Turret();
  private final Hood m_hood = new Hood();
  private final Flywheel m_flywheel = new Flywheel();
  private final Controller m_controller = new Controller();
  private final CommandXboxController m_driver = new CommandXboxController(0);
  private final CommandXboxController m_operator = new CommandXboxController(1);
  private Optional<DriverStation.Alliance> alliance;
  
    private Translation2d our_hub;
    private Translation2d left_shuttle;
    private Translation2d right_shuttle; 
      
        private final SendableChooser<Command> autoChooser;
        
          public RobotContainer() {
      
            SmartDashboard.putNumber("X Sim", 0);
            SmartDashboard.putNumber("Y Sim", 0);
            SmartDashboard.putNumber("ROTSIM", 0);
            SmartDashboard.putNumber("RPM for shuttle", 4000);
            SmartDashboard.putNumber("Hood for shuttle", 0.75);
      
            SmartDashboard.putNumber("Distance X", 0);
            SmartDashboard.putNumber("HeightDifference", 0);
            SmartDashboard.putNumber("Impact Angle Degrees", 0);
      
            NamedCommands.registerCommand("Stop Shooter", new StopShootCommand(m_flywheel, m_hood, m_spindexer, m_kicker));
      //    NamedCommands.registerCommand("Shoot", new ParallelCommandGroup().addCommands(m_flywheel.runFlywheel(), m_hood.positonHood(), m_spindexer.runSpindexer(), m_kicker.runKicker())));
            NamedCommands.registerCommand("Shoot", shootFromDistanceCommand().withTimeout(5.5));
            NamedCommands.registerCommand("Reset Pose", resetOdometry(new Pose2d(0, 0, new Rotation2d(-1*Math.PI/4))));
            //ParallelCommandGroup().addCommands(m_flywheel.runFlywheel(), m_hood.positonHood(), m_spindexer.runSpindexer(), m_kicker.runKicker())));
           // NamedCommands.registerCommand("Extend Intake", m_intake.setVoltageManual(2).until(
            //  () -> m_intake.stopIntakeCheck()
            //).andThen(m_intake.stopIntakeExtendCommand()));//andThen(m_intake.stopIntakeExtendCommand()));
            NamedCommands.registerCommand("Run Intake",m_intake.runRollers(false).withTimeout(5).andThen(m_intake.stopRollers()));
            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Chooser", autoChooser);
      
            alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                boolean isBlueAlliance = alliance.get() == DriverStation.Alliance.Blue;

              

                if (isBlueAlliance) {
                  our_hub = FIELD_CONST.BLUE_HUB;
                  left_shuttle = FIELD_CONST.BLUE_SHUTTLE_LEFT;
                  left_shuttle = FIELD_CONST.BLUE_SHUTTLE_RIGHT;
                } else {
                  our_hub = FIELD_CONST.RED_HUB;
                  right_shuttle = FIELD_CONST.RED_SHUTTLE_LEFT;
                  right_shuttle = FIELD_CONST.RED_SHUTTLE_RIGHT;
                }
                
              } else {
                our_hub = FIELD_CONST.RED_HUB;
                right_shuttle = FIELD_CONST.RED_SHUTTLE_LEFT;
                right_shuttle = FIELD_CONST.RED_SHUTTLE_RIGHT;
              }

            configureBindings();
          }
        
          private void configureBindings() {
            m_driver.y().onTrue(resetNavX()); // Resets NavX , sets to 0

            m_driver.b().onTrue(resetOdometry(new Pose2d())); // TODO RESET TO ALLIANCE HUB BASE

            m_drivetrain.setDefaultCommand(
                Commands.run(
                    () -> m_drivetrain.drive(
                        -m_driver.getLeftY(),
                        -m_driver.getLeftX(),
                        -m_driver.getRightX(), 5, 2),
                    m_drivetrain));
            // auto aligns and shoots simultaniously from the drivers right trigger
            m_driver.rightTrigger().whileTrue(
                Commands.parallel(
                    m_drivetrain.aimDrivetrainCommand(m_drivetrain.getEstimatedPose(), our_hub),
                    Commands.run(
                        () -> shootFromDistanceManual(), m_flywheel),
                    Commands.run(
                        () -> m_spindexer.setSpeed(.7), m_spindexer),
                    Commands.runEnd(
                        () -> m_kicker.setSpeed(0.6),
                        () -> m_kicker.setSpeed(0.0), m_kicker)

                ));
            // SHOOTER
            m_operator.rightTrigger(.5).whileTrue(
                Commands.parallel(
                    Commands.run(
                       () -> shootFromDistanceManual(), m_flywheel),
                    Commands.run(
                        () -> m_spindexer.setSpeed(.7), m_spindexer),
                    Commands.runEnd(
                        () -> m_kicker.setSpeed(0.6),
                        () -> m_kicker.setSpeed(0.0), m_kicker)))
                .whileFalse(
                    Commands.run(() -> resetShooter()));

            // SHUTTLE
            m_operator.b().whileTrue(
                Commands.parallel(
                    Commands.run(
                        () -> shuttle(), m_flywheel),
                    Commands.run(
                        () -> m_spindexer.setSpeed(.7), m_spindexer),
                    Commands.run(
                        () -> m_kicker.setSpeed(0.6), m_kicker)))
                .whileFalse(
                    Commands.run(() -> resetShooter()));
            //unjams spindexer and kicker
            m_operator.y().whileTrue(
                Commands.runEnd(
                    () -> m_spindexer.setSpeed(-.5),
                    () -> m_spindexer.setSpeed(0),
                    m_spindexer).alongWith(
                        Commands.runEnd(
                            () -> m_kicker.setSpeed(-0.6),
                            () -> m_kicker.setSpeed(0),
                            m_kicker)));
            
            //runs intake rollers
            m_operator.leftTrigger(.5).whileTrue(
                Commands.runEnd(
                    () -> m_intake.setRollerSpeed(.5),
                    () -> m_intake.setRollerSpeed(0),
                    m_intake));
            //boost intake roller
                      m_operator.a().whileTrue(
                Commands.runEnd(
                    () -> m_intake.setRollerSpeed(.6),
                    () -> m_intake.setRollerSpeed(0),
                    m_intake));
            // TODO JAM MODE
            //Extends intake
            m_operator.leftBumper().whileTrue(
                Commands.runEnd(
                    () -> m_intake.setExtend(IntakeConst.IntakeExtendSetPoint),
                    () -> m_intake.stopIntake(),
                    m_intake));
            //retracts intake
            m_operator.rightBumper().whileTrue(
                Commands.runEnd(
                    () -> m_intake.setExtend(0), 
                    () -> m_intake.stopIntake(),
                    m_intake));

          }
/*  public Command feedFuelShooterReady(){
    if (Commands.parallel(
         m_drivetrain.aimDrivetrainCommand(m_drivetrain.getEstimatedPose(), our_hub),
          Commands.run(
            () -> shootFromDistanceManual(), m_flywheel),
          Commands.runEnd(
            () -> m_kicker.setSpeed(0.6),
            () -> m_kicker.setSpeed(0.0), m_kicker))){
            m_spindexer.setSpeed(0.7);
            } else {
            m_spindexer.setSpeed(0.0)
          }
 }

 public Command feedFuelShooterRead(){
    if (m_drivetrain.aimDrivetrainCommand(m_drivetrain.getEstimatedPose(), our_hub),
        m_flywheel.shootFromDistanceManual(), m_flywheel
          Commands.runEnd(
            () -> m_kicker.setSpeed(0.6),
            () -> m_kicker.setSpeed(0.0), m_kicker))){
            m_spindexer.setSpeed(0.7);
            } else {
            m_spindexer.setSpeed(0.0)
          }
 }
*/
public Command shootFromDistanceCommand(){
    return Commands.runEnd(() -> {
      shootFromDistanceManual();
       m_spindexer.setSpeed(.7);
       m_kicker.setSpeed(.6);
       m_intake.runRollers(false); 
    }, 
    () -> { 
       resetShooter();
    m_spindexer.setSpeed(0);
    m_kicker.setSpeed(0);
    m_intake.stopRollers();
    
    });
  }

public void runIntake(double roller, double voltage) {
  m_intake.setRollerSpeed(roller);
  m_intake.setExtend(voltage);
}
      
public static double scaleImpactAngle(double distance) {
  return (80 - (3.8598 * distance));
}


public void runContinuouslyForShotCalc() {   
        alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              boolean isBlueAlliance = alliance.get() == DriverStation.Alliance.Blue;
              if (isBlueAlliance) {
                our_hub = FIELD_CONST.BLUE_HUB;
          } else {
            our_hub = FIELD_CONST.RED_HUB;
          }
        } else {
          our_hub = FIELD_CONST.RED_HUB;
        }

  Pose2d currentPose = m_drivetrain.getEstimatedPose();

  //Pose2d currentPose =
    // new Pose2d(new Translation2d(
    //     SmartDashboard.getNumber("X", 0),
    //     SmartDashboard.getNumber("Y", 0)), 
    //     new Rotation2d(
    //       SmartDashboard.getNumber("ROT", 0)
    //     )
  //);

  
  double distance = Controller.hypotenuseCalculator(
    our_hub, 
    currentPose.getTranslation()
  );

    SmartDashboard.putNumber("DISTANCEFROMTARGET", distance);

    double[] shooterRaw = Controller.calculateShooterStatic(
      distance, //TODO SIM
      FIELD_CONST.HUB_SHOOTER_DIFFERENCE, //had to use other height - 2 meter iirc? hopefully will fix with metal coz our shooter ASS
        Math.toRadians(
        scaleImpactAngle(distance)
        )
      );

    double[] realData = Controller.getValuesFromMath(
      shooterRaw[1], Math.toDegrees(shooterRaw[0])
    );

    // m_flywheel.setSpeed(
    //   (realData[0])
    // );
    // m_hood.adjustHood(realData[1]);

      SmartDashboard.putNumber("Angle From Math", shooterRaw[0]);
      SmartDashboard.putNumber("velocity From Math", shooterRaw[1]);

      SmartDashboard.putNumber("RPM From Math", realData[0]);
      SmartDashboard.putNumber("Hood From Math", realData[1]);
      SmartDashboard.putNumber("IntakeExtendSetpoint", IntakeConst.IntakeExtendSetPoint);
      SmartDashboard.putNumber("IntakeEncoderReading", m_intake.ExtendingMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("IntakeP",IntakeConst.kPExtend);

  }

  // public Command shootFromDistance() {
  //   return Commands.parallel(
  //       m_flywheel.runFlyWheelWithInput(
  //       SmartDashboard.getNumber("RPM From Math", 0)
  //     ),
  //     m_hood.positionHoodWithInput(
  //       SmartDashboard.getNumber("Hood From Math", 0)
  //     )
  //   );
  // }

    public void shootFromDistanceManual() {
      m_flywheel.setSpeed(
        SmartDashboard.getNumber("RPM From Math", 0)
      );
      m_hood.adjustHood(
        SmartDashboard.getNumber("Hood From Math", 0)
      );
  }

  public void shuttle() {
      m_flywheel.setSpeed(
        SmartDashboard.getNumber("RPM for shuttle", 4000)
      );
      m_hood.adjustHood(
        SmartDashboard.getNumber("Hood for shuttle", 0.75)
      );
      
  }

   public void resetShooter() {
      m_flywheel.setSpeed(
        2000
      );
      m_hood.adjustHood(
        0
      );
  }

  // public void test() {
  //   double[] input = Controller.getShooterSimple(7.75);
  //   SmartDashboard.putNumber("RPM", input[0]);
  //   SmartDashboard.putNumber("Hood", input[1]);

  //   SmartDashboard.putNumber("Turret Angle Degrees", 
  //     Math.toDegrees(Controller.calculateTurret(new Pose2d(new Translation2d(
  //       SmartDashboard.getNumber("X", 0),
  //       SmartDashboard.getNumber("Y", 0)), 
  //       new Rotation2d(
  //         SmartDashboard.getNumber("ROT", 0)
  //       ))
  //       , FIELD_CONST.BLUE_HUB)
  //   ));
  // }

  public Command shootWithDistanceToHub() {
    return Commands.parallel(
      m_drivetrain.aimDrivetrainCommand(m_drivetrain.getEstimatedPose(), our_hub)
    );
  }

  // public Command cycleNorth() {
    
  // }
  // public Command cycleSouth() {
    
  // }


  // public void temp() {
  //   double[] shooterRaw = Controller.calculateShooterStatic(
  //     SmartDashboard.getNumber("Distance X", 0), 
  //     SmartDashboard.getNumber("HeightDifference", 0), 
  //     Math.toRadians(
  //       SmartDashboard.getNumber("Impact Angle Degrees", 0)
  //     ));

  //   double[] realData = Controller.getValuesFromMath(
  //     shooterRaw[1], Math.toDegrees(shooterRaw[0])
  //   );

  //     m_flywheel.setSpeed(
  //       (realData[0])
  //     );
  //     m_hood.adjustHood(realData[1]);

  //     SmartDashboard.putNumber("Angle From Math", shooterRaw[0]);
  //     SmartDashboard.putNumber("velocity From Math", shooterRaw[1]);


  //     SmartDashboard.putNumber("RPM From Math", realData[0]);
  //     SmartDashboard.putNumber("Hood From Math", realData[1]);

  // }

  public void updateVision() {
    // Optional<EstimatedRobotPose> estimatel = caml.EstimatePose();
    // if (estimatel.isPresent()) {
    //   m_drivetrain.updatePoseWithVision(estimatel.get());
    //   Pose2d m_pose = estimatel.get().estimatedPose.toPose2d();
    // }

    Optional<EstimatedRobotPose> estimater = camR.EstimatePose();
    if (estimater.isPresent()) {
      m_drivetrain.updatePoseWithVision(estimater.get());
      Pose2d m_pose = estimater.get().estimatedPose.toPose2d();
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

public void tempsim() {
  Pose2d currentPose =
    new Pose2d(new Translation2d(
        SmartDashboard.getNumber("X Sim", 0),
        SmartDashboard.getNumber("Y Sim", 0)), 
        new Rotation2d(
          SmartDashboard.getNumber("ROTSIM", 0)
        )
  );

  m_drivetrain.simulateAutoLock(currentPose.getTranslation(), our_hub);
}

}
