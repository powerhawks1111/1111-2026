// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;


public class Spindexer extends SubsystemBase {
  /** Creates a new Spindexer. */
  public Spindexer() {
    final Encoder encoder = new Encoder(null, null);
    final PIDController pidcontrol = new PIDController(0.04, 0, 0);
    final SparkMax sparkmax = new SparkMax(0, null);

    // getDistance();
    // getRate();
    // getStopped();
  }
// Get spindexer to spin at a rate, then also be able to stop
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
