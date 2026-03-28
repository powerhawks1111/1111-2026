package frc.robot.subsystems;


import com.revrobotics.spark.*;
import com.revrobotics.*;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.IntakeConst;

public class Intake extends SubsystemBase{
    private SparkFlex flipDownMotor;
    private SparkFlexConfig flipDownMotorConfig;

    private SparkFlex rollerMotor;
    private SparkFlexConfig rollerMotorConfig;

    public Intake() {
      SmartDashboard.putNumber("INTAKE", 0);
        flipDownMotor = new SparkFlex(CANID.Flipper, MotorType.kBrushless);
        rollerMotor = new SparkFlex(CANID.Rollers, MotorType.kBrushless);

        flipDownMotorConfig = new SparkFlexConfig();
        rollerMotorConfig = new SparkFlexConfig();
        flipDownMotorConfig.idleMode(IdleMode.kBrake);
        flipDownMotorConfig.inverted(true);

        flipDownMotorConfig.encoder.positionConversionFactor(IntakeConst.positionConversionFactor);
        flipDownMotorConfig.closedLoop
            .pid(0, 0, 0);

        flipDownMotor.configure(flipDownMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  

    }

    public void setRollerSpeed(double rpm) {
        //rollerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
        rollerMotor.set(rpm);
    }

    //TODO: Might need to use feed forward to overcome gravity the first 90 degrees and fight against it the last 90 degrees of rotation
    public void setFlip(double position) {
      //0 -> 12.03
        flipDownMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void periodic() {
    }
    
  public Command setVoltageManual(double voltage) {
      return this.run(() -> flipDownMotor.setVoltage(
        voltage
      )
    );
  }

  public Command deployIntake() { 
    return this.runOnce(() -> setFlip(0));  //TODO: need to find position. 
  }
  public Command retractIntake() {
    return this.runOnce(() -> setFlip(0)); // TODO: will we want this?
  }
  public Command runRollers(boolean reversed) {
    if(reversed) {
    return this.run(() -> setRollerSpeed(.6
    ));
    } else {
    return this.run(() -> setRollerSpeed(-.6));
    }
  }

  public Command stopRollers() {
    return this.run(() -> setRollerSpeed(0)); // TODO: change this to setvoltage to 0
  }
  public void runIntakeManually(double leftTrigger, double rightTrigger) {
      if(leftTrigger > rightTrigger) {
        flipDownMotor.set(-leftTrigger/2); //max of 6v allowed 
      }
      if(leftTrigger < rightTrigger) {
        flipDownMotor.set(rightTrigger/2); //max of 6v allowed 
      }
  }

  public Command runIntakeFlipManuallyCommand(double left, double right) {
    return this.run(() -> runIntakeManually(left, right));
  }

  public Command stopIntakeFlipCommand() {
    return this.run(() -> flipDownMotor.setVoltage(0));
  }
}