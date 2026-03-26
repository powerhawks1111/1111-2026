package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
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

        hoodMotorConfig.encoder.positionConversionFactor(HoodConst.positionConversionFactor);
        hoodMotorConfig.closedLoop
            .pid(HoodConst.kP, HoodConst.kI, HoodConst.kD);
        hoodMotorConfig.inverted(true);
        hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_Controller = hoodMotor.getClosedLoopController();
    }

    public void adjustHood(double position) {
        m_Controller.setSetpoint(position, ControlType.kPosition);

        
    }

    public boolean atBottom() { 

        
        if(hoodMotor.getOutputCurrent() > 20) {
           
            hoodMotor.getEncoder().setPosition(0);
            return true; 

        }
        else{

            return false; 

            
        }
    }

    public Command zeroHood() { 
    
        return this.run(() -> hoodMotor.setVoltage(-1)).until(this::atBottom);

    }
}
