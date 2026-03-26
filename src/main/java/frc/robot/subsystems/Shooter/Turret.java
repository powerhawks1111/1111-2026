package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.*;
import com.revrobotics.*;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.SpindexerConst;
import frc.robot.Constants.TurretConst;

import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Turret extends SubsystemBase{

    private SparkMax turretMotor;
    private SparkMaxConfig turretMotorConfig;

    public Turret() {
        turretMotor = new SparkMax(CANID.Turret, MotorType.kBrushless);
        turretMotorConfig = new SparkMaxConfig();

        turretMotorConfig.idleMode(IdleMode.kBrake);
        turretMotorConfig.encoder.positionConversionFactor(TurretConst.positionConversionFactor);
        turretMotorConfig.closedLoop
            .pid(TurretConst.kP, TurretConst.kI, TurretConst.kD);

        turretMotor.configure(turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void adjustTurret(double position) {
        turretMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
    }
    public Command positionTurret(double position) {
        return this.runOnce(() -> adjustTurret(position));
    }
    
}
    
        
