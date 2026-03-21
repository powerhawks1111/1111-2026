package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.SpindexerConst;

public class Spindexer extends SubsystemBase{
    private SparkMax spindexerMotor;
    private SparkMaxConfig spindexerMotorConfig;

    public Spindexer() {
        spindexerMotor = new SparkMax(CANID.Spindexer, MotorType.kBrushless);
        spindexerMotorConfig = new SparkMaxConfig();

        spindexerMotorConfig.encoder.positionConversionFactor(SpindexerConst.bpsConversionFactor);
        spindexerMotorConfig.closedLoop
            .pid(0, 0, 0);
        
        spindexerMotor.configure(spindexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void runSpindexer(double bps) {
        spindexerMotor.getClosedLoopController().setSetpoint(bps, ControlType.kVelocity);
    }
    public void testSpindexer() {
      spindexerMotor.set(SmartDashboard.getNumber("velocity", SpindexerConst.testspeed));
    }

    public Command testSpindexerCommmand() {
        return new StartEndCommand(() -> this.testSpindexer(), () -> this.spindexerMotor.set(0), this);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("spindexer speed", spindexerMotor.getEncoder().getVelocity());
        
    }
}
