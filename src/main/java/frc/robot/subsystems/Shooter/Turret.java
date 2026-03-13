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

public class Turret {

    public Turret() {
        
    }
}


/**
 * package frc.robot.subsystems;

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
import frc.robot.Constants.DriveConst;
import frc.robot.Constants.ShootConst;

public class Shooter {
    private final SparkFlex leftShoot;
    private final SparkFlex rightShoot; 
    private final SparkMax hoodMotor; //adjusts hood angle
    private final SparkMax turretMotor; //spins the shooter

    private final SparkFlexConfig leftShootConfig;
    private final SparkFlexConfig rightShootConfig;
    private final SparkMaxConfig hoodConfig;
    private final SparkMaxConfig turretConfig;

    public Shooter() {

        leftShootConfig = new SparkFlexConfig();
        rightShootConfig = new SparkFlexConfig();
        hoodConfig = new SparkMaxConfig();
        turretConfig = new SparkMaxConfig();

        leftShootConfig.idleMode(IdleMode.kBrake);

        leftShoot = new SparkFlex(ShootConst.leftShootID, MotorType.kBrushless);
        rightShoot = new SparkFlex(ShootConst.rightShootID, MotorType.kBrushless);
        hoodMotor = new SparkMax(ShootConst.hoodID, MotorType.kBrushless);
        turretMotor = new SparkMax(ShootConst.turretID, MotorType.kBrushless);

        leftShoot.configure(leftShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightShoot.configure(leftShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hoodMotor.configure(leftShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretMotor.configure(leftShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}

 */