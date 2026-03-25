package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.SpindexerConst;

public class Spindexer extends SubsystemBase{
    private SparkMax spindexerMotor;
    private SparkMaxConfig spindexerMotorConfig;

    public Spindexer() {
        spindexerMotor = new SparkMax(CANID.Spindexer, MotorType.kBrushless);
        spindexerMotorConfig = new SparkMaxConfig();

        spindexerMotorConfig.encoder.velocityConversionFactor(SpindexerConst.bpsConversionFactor);
        spindexerMotorConfig.closedLoop
            .pid(SpindexerConst.kP, SpindexerConst.kI, SpindexerConst.kD);
        spindexerMotorConfig.inverted(false);
        spindexerMotor.configure(spindexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      SmartDashboard.putNumber("Spindexer", 0);
    }

    public void setSpeed(double bps) {
        spindexerMotor.getClosedLoopController().setSetpoint(bps, ControlType.kVelocity);
    }

    public Command runSpindexer(double bps) {
      return this.runOnce(() -> setSpeed(bps));
    }

    @Override
    public void periodic() {
       spindexerMotor.set(
          SmartDashboard.getNumber("Spindexer", 0)
        );
    }
}
