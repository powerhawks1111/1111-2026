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
    private final SparkMax hoodMotor; //adjusts hood angle
    private final SparkMax turretMotor; //spins the shooter

    private final SparkFlexConfig leftShootConfig;
    private final SparkFlexConfig rightShootConfig;
    private final SparkMaxConfig hoodConfig;
    private final SparkMaxConfig turretConfig;

    public Flywheel() {
        leftShootConfig = new SparkFlexConfig();
        rightShootConfig = new SparkFlexConfig();
        hoodConfig = new SparkMaxConfig();
        turretConfig = new SparkMaxConfig();

        leftShootConfig.idleMode(IdleMode.kBrake);

        leftShoot = new SparkFlex(CANID.LeftS, MotorType.kBrushless);
        rightShoot = new SparkFlex(CANID.RightS, MotorType.kBrushless);
        hoodMotor = new SparkMax(CANID.Hood, MotorType.kBrushless);
        turretMotor = new SparkMax(CANID.Turret, MotorType.kBrushless);

        leftShootConfig.voltageCompensation(12);
        rightShootConfig.voltageCompensation(12);

        rightShootConfig.closedLoopRampRate(FlywheelConst.closedLoopRampRate);
        rightShootConfig.closedLoop.pid(0, 0, 0);
        
        leftShootConfig.follow(CANID.LeftS);
        leftShootConfig.inverted(true);

        leftShoot.configure(leftShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightShoot.configure(leftShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hoodMotor.configure(leftShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretMotor.configure(leftShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
