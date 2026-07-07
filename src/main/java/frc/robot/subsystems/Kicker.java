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
    //private SparkMax backMotor;
    private SparkMaxConfig backMotorConfig;

    private SparkFlex frontMotor;
    private SparkFlexConfig frontMotorConfig;

    public Kicker() {
        //backMotor = new SparkMax(CANID.BackK, MotorType.kBrushless);
        frontMotor = new SparkFlex(CANID.FrontK, MotorType.kBrushless);

        backMotorConfig = new SparkMaxConfig();
        frontMotorConfig = new SparkFlexConfig();
        frontMotorConfig.idleMode(IdleMode.kCoast);
        backMotorConfig.idleMode(IdleMode.kCoast);

        frontMotorConfig.voltageCompensation(11);
        backMotorConfig.voltageCompensation(11);

        backMotorConfig.inverted(true);

//        backMotorConfig.voltageCompensation(11);
//        frontMotorConfig.voltageCompensation(11);

        backMotorConfig.closedLoop
            .pid(0, 0, 0);
        frontMotorConfig.closedLoop
            .pid(0, 0, 0);

        //backMotor.configure(backMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        frontMotor.configure(frontMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  

        SmartDashboard.putNumber("Back KickerSpeed", 0);
        SmartDashboard.putNumber("Front KickerSpeed", 0);
    }

    public void setSameSpeed(double speed) {
        //backMotor.getClosedLoopController().setSetpoint(speed, ControlType.kVelocity);
        frontMotor.getClosedLoopController().setSetpoint(speed, ControlType.kVelocity);


       // SmartDashboard.putNumber(
           // "BackMotorSetpoint", backMotor.getClosedLoopController().getSetpoint()    
        //);
        SmartDashboard.putNumber(
            "FrontMotorSetpoint", frontMotor.getClosedLoopController().getSetpoint()    
        );

       // SmartDashboard.putNumber(
         //   "BackMotorError", backMotor.getClosedLoopController().getSetpoint() - backMotor.getEncoder().getVelocity()
       // );
        SmartDashboard.putNumber(
            "FrontMotorError", frontMotor.getClosedLoopController().getSetpoint() - frontMotor.getEncoder().getVelocity()
        );

        //backMotor.set(speed);
        //frontMotor.set(speed);
    }

    public void setDiffSpeeds(double front, double back){
        // backMotor.getClosedLoopController().setSetpoint(front, ControlType.kVelocity);
        // frontMotor.getClosedLoopController().setSetpoint(back, ControlType.kVelocity);

        //backMotor.set(back);
        frontMotor.set(front);

        // SmartDashboard.putNumber(
        //     "BackMotorSetpoint", backMotor.getClosedLoopController().getSetpoint()    
        // );
        // SmartDashboard.putNumber(
        //     "FrontMotorSetpoint", frontMotor.getClosedLoopController().getSetpoint()    
        // );

        // SmartDashboard.putNumber(
        //     "BackMotorError", backMotor.getClosedLoopController().getSetpoint() - backMotor.getEncoder().getVelocity()
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
       // backMotor.set(
       //      SmartDashboard.getNumber("Back KickerSpeed", 0)
       // );
        frontMotor.set(
            SmartDashboard.getNumber("Front KickerSpeed", 0)
        );


          //  SmartDashboard.putNumber("Back Kicker velocity", backMotor.getEncoder().getVelocity());
            SmartDashboard.putNumber("Front Kicker velocity", frontMotor.getEncoder().getVelocity());
            
            SmartDashboard.putNumber("Front kick applied output", frontMotor.getAppliedOutput());
          //SmartDashboard.putNumber("back kick applied output", backMotor.getAppliedOutput());
    }

}
