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
    private SparkMax flipDownMotor;
    private SparkMaxConfig flipDownMotorConfig;

    private SparkFlex rollerMotor;
    private SparkFlexConfig rollerMotorConfig;

    public Intake() {
        flipDownMotor = new SparkMax(CANID.Flipper, MotorType.kBrushless);
        rollerMotor = new SparkFlex(CANID.Rollers, MotorType.kBrushless);

        flipDownMotorConfig = new SparkMaxConfig();
        rollerMotorConfig = new SparkFlexConfig();
        rollerMotorConfig.inverted(true);

        flipDownMotorConfig.encoder.positionConversionFactor(IntakeConst.positionConversionFactor);
        flipDownMotorConfig.closedLoop
            .pid(0, 0, 0);
        
        rollerMotorConfig.closedLoop
            .pid(0, 0, 0);

        flipDownMotor.configure(flipDownMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  

        SmartDashboard.putNumber("Intake Roller Percent", 0);
        SmartDashboard.putNumber("Intake Flip Percent", 0);
    }

    public void setRollerSpeed(int rpm) {
        rollerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    //TODO: Might need to use feed forward to overcome gravity the first 90 degrees and fight against it the last 90 degrees of rotation
    public void setFlip(double position) {
        flipDownMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        rollerMotor.set(SmartDashboard.getNumber("Intake Roller Percent", 0));
        flipDownMotor.set(SmartDashboard.getNumber("Intake Flip Percent", 0));
    }
    
  public Command deployIntake() { 
    return this.runOnce(() -> setFlip(0));  //TODO: need to find position. 
  }
  public Command retractIntake() {
    return this.runOnce(() -> setFlip(0)); // TODO: will we want this?
  }
  public Command runRollers(boolean reversed) {
    if(reversed) {
    return this.runOnce(() -> setRollerSpeed(IntakeConst.rollerSpeed));
    } else {
    return this.runOnce(() -> setRollerSpeed(-IntakeConst.rollerSpeed));
    }
  }
  public Command stopRollers() {
    return this.runOnce(() -> setRollerSpeed(0)); // TODO: change this to setvoltage to 0
  }
}