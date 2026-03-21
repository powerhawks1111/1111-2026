// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANID;
import frc.robot.Constants.ClimbConst;

public class Climb extends SubsystemBase {

  private SparkMax climbMotor1;
  private SparkMaxConfig climbMotor1Config;

  private SparkMax climbMotor2;
  private SparkMaxConfig climbMotor2Config;

  private SparkMaxConfig climbConfig;

  private double setpoint = 27 ;

  /** Creates a new Climb. */
  // notes for later, gear ratio = 15/1, elvator sits down all match using break mode to keep it down 
  //then when time for climb, a button is pressed and elevator jumps to
  public Climb() {

    SparkMax climbMotor1 = new SparkMax(18, MotorType.kBrushless);
    SparkMax climbMotor2 = new SparkMax(19, MotorType.kBrushless);

    climbMotor1Config = new SparkMaxConfig();
    climbMotor2Config = new SparkMaxConfig();

    SparkMaxConfig climbConfig = new SparkMaxConfig();

    climbConfig = climbConfig.encoder.positionConversionFactor(ClimbConst.positionConversionFactor);
    climbMotor1Config.closedLoop
    .pid(kP, kI, kD);
     climbMotor2.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//one motor needs to be inverted

    climbConfig = climbConfig.encoder.positionConversionFactor(ClimbConst.positionConversionFactor);
    climbMotor2Config.closedLoop
    .pid(0, 0, 0);
     climbMotor2.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  public void brakeMode(){

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("climbMotor1",climbMotor1.climbConfig.get());
  }
}
