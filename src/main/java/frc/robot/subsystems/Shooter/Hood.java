package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.HoodConst;
import frc.robot.Constants.SpindexerConst;

public class Hood extends SubsystemBase{
    private SparkMax hoodMotor;
    private SparkMaxConfig hoodMotorConfig;
    private SparkClosedLoopController m_Controller;
    public Hood() {
        hoodMotor = new SparkMax(CANID.Hood, MotorType.kBrushless);
        hoodMotorConfig = new SparkMaxConfig();
        hoodMotorConfig.inverted(true);
        
        hoodMotorConfig.encoder
            .positionConversionFactor(HoodConst.positionConversionFactor);

        hoodMotorConfig.closedLoop
            .pid(HoodConst.kP, HoodConst.kI, HoodConst.kD);
        hoodMotor.getEncoder().setPosition(0);
        hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_Controller = hoodMotor.getClosedLoopController();
    }

    public void adjustHood(double position) {
        SmartDashboard.putNumber("HoodPositionOutput", hoodMotor.getEncoder().getPosition());
        hoodMotor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
    }
}
