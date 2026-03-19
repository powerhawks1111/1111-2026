// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Encoder.IndexingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Spindexer extends SubsystemBase {
  /** Creates a new Spindexer. */
  public Spindexer() {
    final Encoder encoder = new Encoder(null, null);
    // Creates a PIDController with gains kP, kI, and kD
    final PIDController pidcontrol = new PIDController(0.04, 0, 0);
    final SparkMax sparkmax = new SparkMax(0, null);
    final Spindexer spindexer = new Spindexer();
    final IndexingType indetype;
    final DCMotor dcmotor = new DCMotor(0, 0, 0, 0, 0, 0);
  }

    private double velocityError = 0;
    private double velocitySpeed= 0;

  
// Get spindexer to spin at a rate, then also be able to stop(?)
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public static void rotateToNext{
          // Code to rotate motor
          Spindexer.rotateToNext();
                  Spindexer();
              }
            private static void rotateToNext() {
              // TODO Auto-generated method stub
              throw new UnsupportedOperationException("Unimplemented method 'rotateToNext'");
            }

   
}
    // getDistance();
    // getStopped();