package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.IntakeConst;

public class Kicker extends SubsystemBase{

    private SparkFlex frontMotor;
    private SparkFlexConfig frontMotorConfig;

    public Kicker() {
        frontMotor = new SparkFlex(CANID.FrontK, MotorType.kBrushless);


        frontMotorConfig = new SparkFlexConfig();
        frontMotorConfig.idleMode(IdleMode.kCoast);


        frontMotorConfig.voltageCompensation(11);
     


//        frontMotorConfig.voltageCompensation(11);

        frontMotorConfig.closedLoop
            .pid(0, 0, 0);

        frontMotor.configure(frontMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  

 
        SmartDashboard.putNumber("Front KickerSpeed", 0);
    }

    public void setSameSpeed(double speed) {
 
        frontMotor.getClosedLoopController().setSetpoint(speed, ControlType.kVelocity);


    
        SmartDashboard.putNumber(
            "FrontMotorSetpoint", frontMotor.getClosedLoopController().getSetpoint()    
        );

        SmartDashboard.putNumber(
            "FrontMotorError", frontMotor.getClosedLoopController().getSetpoint() - frontMotor.getEncoder().getVelocity()
        );


        //frontMotor.set(speed);
    }

    public void setDiffSpeeds(double front, double back){

        // frontMotor.getClosedLoopController().setSetpoint(back, ControlType.kVelocity);


        frontMotor.set(front);

   
        // SmartDashboard.putNumber(
        //     "FrontMotorSetpoint", frontMotor.getClosedLoopController().getSetpoint()    
        // );


        // SmartDashboard.putNumber(
        //     "FrontMotorError", frontMotor.getClosedLoopController().getSetpoint() - frontMotor.getEncoder().getVelocity()
        // );

    }

    public Command runKicker() {
        return this.runEnd(() -> setDiffSpeeds(0.7, 0.9), () -> setSameSpeed(0));
    }
    
    public Command stopKicker() {
        return this.run(() -> setSameSpeed(0));
    }

    @Override
    public void periodic() {
 
        frontMotor.set(
            SmartDashboard.getNumber("Front KickerSpeed", 0)
        );


            SmartDashboard.putNumber("Front Kicker velocity", frontMotor.getEncoder().getVelocity());
            
            SmartDashboard.putNumber("Front kick applied output", frontMotor.getAppliedOutput());

    }

}
