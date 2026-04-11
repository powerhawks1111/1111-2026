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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

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

  private final Vision caml = new Vision(CameraConst.LeftCamName, CameraConst.leftCamTransform);
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
      
        private final SendableChooser<Command> autoChooser;
        
          public RobotContainer() {
      
            SmartDashboard.putNumber("X", 0);
            SmartDashboard.putNumber("Y", 0);
            SmartDashboard.putNumber("ROT", 0);
      
            SmartDashboard.putNumber("Distance X", 0);
            SmartDashboard.putNumber("HeightDifference", 0);
            SmartDashboard.putNumber("Impact Angle Degrees", 0);
      
            NamedCommands.registerCommand("Stop Shooter", new StopShootCommand(m_flywheel, m_hood, m_spindexer, m_kicker));
      //    NamedCommands.registerCommand("Shoot", new ParallelCommandGroup().addCommands(m_flywheel.runFlywheel(), m_hood.positonHood(), m_spindexer.runSpindexer(), m_kicker.runKicker())));
            NamedCommands.registerCommand("Shoot", shootFromDistanceCommand().withTimeout(10));
            NamedCommands.registerCommand("Reset Pose", resetOdometry(new Pose2d(0, 0, new Rotation2d(-1*Math.PI/4))));
            //ParallelCommandGroup().addCommands(m_flywheel.runFlywheel(), m_hood.positonHood(), m_spindexer.runSpindexer(), m_kicker.runKicker())));
            NamedCommands.registerCommand("Lower Intake", m_intake.setVoltageManual(2).withTimeout(0.85).andThen(m_intake.stopIntakeFlipCommand()));
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
                our_hub = FIELD_CONST.RED_HUB;
              }
            configureBindings();
          }
        
  private void configureBindings() {
            m_driver.button(1).onTrue(resetNavX());
      
            m_driver.button(2).onTrue(resetOdometry(new Pose2d())); //TODO RESET TO ALLIANCE HUB BASE

            m_drivetrain.setDefaultCommand(
              Commands.run(
                () -> m_drivetrain.drive(
                  -m_driver.getRawAxis(1), 
                  -m_driver.getRawAxis(0), 
                  -m_driver.getRawAxis(4), 5, 2), m_drivetrain)
            );

        }

public Command shootFromDistanceCommand(){
    return Commands.runEnd(() -> {
      shootFromDistanceManual();
       m_spindexer.setSpeed(.7);
       m_kicker.setDiffSpeeds(.6, .6);
    }, 
    () -> { 
       resetShooter();
    m_spindexer.setSpeed(0);
    m_kicker.setDiffSpeeds(0,0);
    
    });
  }

public void runButtonNew() {
  //SHOOTER
  if(m_operator.rightTrigger(.5).getAsBoolean()) {
    shootFromDistanceManual();
    m_spindexer.setSpeed(.7);
    m_kicker.setDiffSpeeds(.6, .6);
  } else if (m_operator.y().getAsBoolean()) {
    resetShooter();
    m_spindexer.setSpeed(-.5);
    m_kicker.setDiffSpeeds(-.25, -.25);
  } else {
    resetShooter();
    m_spindexer.setSpeed(0);
    m_kicker.setDiffSpeeds(0,0);
  }

  //INTAKE
  if(m_operator.leftTrigger(.5).getAsBoolean()) {
    m_intake.setRollerSpeed(-.7);
  } else {
    m_intake.setRollerSpeed(0);
  }

  if(m_operator.leftBumper().getAsBoolean()) {
    m_intake.setFlip(0.15);
  } else if (m_operator.rightBumper().getAsBoolean()) {
    m_intake.setFlip(-.2);
  } else {
    m_intake.setFlip(0);
  }

  //ROLLER
  if(m_operator.leftTrigger(.4).getAsBoolean()) {
    m_intake.setRollerSpeed(-.6);
  } else {
    m_intake.setRollerSpeed(0);
  }

}

public void runIntake(double roller, double voltage) {
  m_intake.setRollerSpeed(roller);
  m_intake.setFlip(voltage);
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

  // Pose2d currentPose =
  //   new Pose2d(new Translation2d(
  //       SmartDashboard.getNumber("X", 0),
  //       SmartDashboard.getNumber("Y", 0)), 
  //       new Rotation2d(
  //         SmartDashboard.getNumber("ROT", 0)
  //       )
  // );
  
  
  double distance = Controller.hypotenuseCalculator(
    our_hub, 
    currentPose.getTranslation()
  );

    SmartDashboard.putNumber("DISTANCEFROMTARGET", distance);

    double[] shooterRaw = Controller.calculateShooterStatic(
      distance, //TODO SIM
      1.321, //had to use other height - 2 meter iirc? hopefully will fix with metal coz our shooter ASS
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
      System.out.println(
        SmartDashboard.getNumber("Hood From Math", 0));
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
      m_drivetrain.aimDrivetrainCommand(our_hub, our_hub)
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
    Optional<EstimatedRobotPose> estimatel = caml.EstimatePose();
    if (estimatel.isPresent()) {
      m_drivetrain.updatePoseWithVision(estimatel.get());
      Pose2d m_pose = estimatel.get().estimatedPose.toPose2d();
    }

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

  public void configNew() {
  //SHOOTER
    m_operator.rightTrigger(.5).onTrue(
        Commands.parallel(
          Commands.run(
            () -> shootFromDistanceManual(), m_flywheel),
          Commands.run(
            () -> m_spindexer.setSpeed(.7), m_spindexer),
          Commands.run(
            () -> m_kicker.setDiffSpeeds(0.6, 0.6), m_kicker)
        )).onFalse(
        Commands.parallel(
          Commands.run(
            () -> shootFromDistanceManual(), m_flywheel),
          Commands.run(
            () -> m_spindexer.setSpeed(0), m_spindexer),
          Commands.run(
            () -> m_kicker.setDiffSpeeds(0, 0), m_kicker)
        )
    );

    m_operator.y().onTrue(
      Commands.runEnd(
        () -> m_spindexer.setSpeed(-.5), 
        () -> m_spindexer.setSpeed(0), 
        m_spindexer).alongWith(
          Commands.runEnd(
            () -> m_kicker.setDiffSpeeds(-0.6, -0.6), 
            () -> m_kicker.setDiffSpeeds(0, 0), 
            m_kicker)
        )
    );

  m_operator.leftTrigger(.5).and(m_operator.a()).onTrue(
    Commands.runEnd(
      () -> m_intake.setRollerSpeed(-.7), 
      () -> m_intake.setRollerSpeed(0), 
      m_intake)
  );
  
  if(m_operator.leftBumper().getAsBoolean()) {
    m_intake.setFlip(0.15);
  } else if (m_operator.rightBumper().getAsBoolean()) {
    m_intake.setFlip(-.2);
  } else {
    m_intake.setFlip(0);
  }

  //ROLLER
  if(m_operator.leftTrigger(.4).getAsBoolean()) {
    m_intake.setRollerSpeed(-.6);
  } else {
    m_intake.setRollerSpeed(0);
  }

}

}
