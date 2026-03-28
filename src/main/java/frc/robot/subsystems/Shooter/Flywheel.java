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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.FlywheelConst;

public class Flywheel extends SubsystemBase{
    private final SparkFlex leftShoot;
    private final SparkFlex rightShoot; 

    private final SparkFlexConfig leftShootConfig;
    private final SparkFlexConfig rightShootConfig;

    private final SparkClosedLoopController m_controller;
    public Flywheel() {
        SmartDashboard.putNumber("RPM", 0);
        leftShootConfig = new SparkFlexConfig();
        rightShootConfig = new SparkFlexConfig();

        leftShootConfig.idleMode(IdleMode.kCoast);
        rightShootConfig.idleMode(IdleMode.kCoast);

//        leftShootConfig.smartCurrentLimit(80);
//        rightShootConfig.smartCurrentLimit(80);

        leftShoot = new SparkFlex(CANID.LeftS, MotorType.kBrushless);
        rightShoot = new SparkFlex(CANID.RightS, MotorType.kBrushless);

        leftShootConfig.voltageCompensation(12);
        rightShootConfig.voltageCompensation(12);

        //rightShootConfig.openLoopRampRate(2);
        rightShootConfig.closedLoop.pid(FlywheelConst.kP, FlywheelConst.kI, FlywheelConst.kD);
        rightShootConfig.closedLoop.feedForward
            .kV(FlywheelConst.kV)
            .kS(FlywheelConst.kS);

        leftShootConfig.follow(CANID.RightS, true);
        //leftShootConfig.inverted(true);

        leftShoot.configure(leftShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightShoot.configure(rightShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_controller = rightShoot.getClosedLoopController();
        
    }

    public void setSpeed(double rpm) {
        m_controller.setSetpoint(rpm, ControlType.kVelocity);
    }

    //TODO: Test if this should be runOnce or run
    public Command runFlywheel() {
        return this.run(() -> setSpeed(SmartDashboard.getNumber("RPM", 3500)));
    }

    @Override
    public void periodic() {
        setSpeed(
          SmartDashboard.getNumber("RPM", 0)
        );
        SmartDashboard.putNumber("Velocity", rightShoot.getEncoder().getVelocity());
        SmartDashboard.putNumber("Current", rightShoot.getOutputCurrent());
    }
}
