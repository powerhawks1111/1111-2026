package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANID;
import frc.robot.Constants.HoodConst;
import frc.robot.Constants.SpindexerConst;

public class Hood {
    private SparkMax hoodMotor;
    private SparkMaxConfig hoodMotorConfig;

    public Hood() {
        hoodMotor = new SparkMax(CANID.Hood, MotorType.kBrushless);
        hoodMotorConfig = new SparkMaxConfig();

        hoodMotorConfig.encoder.positionConversionFactor(HoodConst.positionConversionFactor);
        hoodMotorConfig.closedLoop
            .pid(0, 0, 0);
        
        hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void runSpindexer(double position) {
        hoodMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
    }
}
