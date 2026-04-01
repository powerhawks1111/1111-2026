package frc.robot.subsystems;


import com.revrobotics.spark.*;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.IntakeConst;

public class Intake extends SubsystemBase{
    private SparkFlex flipDownMotor;
    private SparkFlexConfig flipDownMotorConfig;

    private SparkFlex rollerMotor;
    private SparkFlexConfig rollerMotorConfig;

    public Intake() {
        flipDownMotor = new SparkFlex(CANID.Flipper, MotorType.kBrushless);
        rollerMotor = new SparkFlex(CANID.Rollers, MotorType.kBrushless);

        flipDownMotorConfig = new SparkFlexConfig();
        rollerMotorConfig = new SparkFlexConfig();
        flipDownMotorConfig.idleMode(IdleMode.kCoast);
        flipDownMotorConfig.inverted(true);

        flipDownMotorConfig.encoder.positionConversionFactor(IntakeConst.positionConversionFactor);
        flipDownMotorConfig.closedLoop
            .pid(IntakeConst.kPFlip, IntakeConst.kIFlip, IntakeConst.kDFlip);

        flipDownMotor.configure(flipDownMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  

    }

    public void setRollerSpeed(double rpm) {
        //rollerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
        rollerMotor.set(rpm);
    }

    public void setFlip(double position) {
      //0 -> 12.03
      flipDownMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void periodic() {

    }
}