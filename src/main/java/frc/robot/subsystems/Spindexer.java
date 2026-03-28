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

        SmartDashboard.putNumber("Spindexer", 0);
        spindexerMotorConfig.encoder
          .positionConversionFactor(SpindexerConst.motorRotsToFuel)
          .velocityConversionFactor(SpindexerConst.motorRotsToFuelPerSec);
        spindexerMotorConfig.closedLoop
            .pid(SpindexerConst.kP, SpindexerConst.kI, SpindexerConst.kD);
        spindexerMotorConfig.inverted(true);
        spindexerMotor.configure(spindexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setSpeed(double bps) {
        //spindexerMotor.getClosedLoopController().setSetpoint(bps, ControlType.kVelocity);
        spindexerMotor.set(bps);
    }

    public Command runSpindexer() {
      return this.run(() -> setSpeed(SmartDashboard.getNumber("Spindexer", .2)));
    }

    public Command stopSpindexer() {
      return this.run(() -> setSpeed(0));
    }

    public Command reverseSpindexer() {
      return this.run(() -> setSpeed(-.2));
    }

    // @Override
    // public void periodic() {
    //   //  setSpeed(
    //   //     SmartDashboard.getNumber("Spindexer", 0)
    //   //   );
    //   spindexerMotor.set(SmartDashboard.getNumber("Spindexer", 0));
    //   SmartDashboard.putNumber("Encoder Reading", spindexerMotor.getEncoder().getPosition());

    // }
}
