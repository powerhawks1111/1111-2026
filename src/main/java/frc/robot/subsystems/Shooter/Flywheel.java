package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.*;
import com.revrobotics.*;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.FlywheelConst;

public class Flywheel extends SubsystemBase{
    private final SparkFlex leftShoot;
    private final SparkFlex rightShoot; 

    private final SparkFlexConfig leftShootConfig;
    private final SparkFlexConfig rightShootConfig;

    public Flywheel() {
        leftShootConfig = new SparkFlexConfig();
        rightShootConfig = new SparkFlexConfig();

        leftShootConfig.idleMode(IdleMode.kBrake);
        rightShootConfig.idleMode(IdleMode.kBrake);

        leftShoot = new SparkFlex(CANID.LeftS, MotorType.kBrushless);
        rightShoot = new SparkFlex(CANID.RightS, MotorType.kBrushless);

        // leftShootConfig.voltageCompensation(12);
        // rightShootConfig.voltageCompensation(12);

        rightShootConfig.closedLoopRampRate(FlywheelConst.closedLoopRampRate);
        rightShootConfig.closedLoop.pid(FlywheelConst.kP, FlywheelConst.kI, FlywheelConst.kD);
        rightShootConfig.closedLoop.feedForward
            .kV(FlywheelConst.kV)
            .kS(FlywheelConst.kS);

        // leftShootConfig.follow(CANID.RightS);
        // leftShootConfig.inverted(true);

        leftShoot.configure(leftShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightShoot.configure(leftShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runFlywheel(double rpm) {
        rightShoot.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    
    }

    public boolean atRPM() {
        return rightShoot.getClosedLoopController().isAtSetpoint();
    }

    public void kinematicsToRealWorld(double[] matchData) {
        double realAngle = 0; //y = mx+b unless better equation 
        double realRPM = 0; // y=mx+b unless better equation
    }

    
}
