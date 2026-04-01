// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Shooter.Flywheel;
import frc.robot.subsystems.Shooter.Hood;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopShootCommand extends ParallelCommandGroup {
  /** Creates a new StopShootCommand. */
  public StopShootCommand(Flywheel flywheel, Hood hood, Spindexer spindexer, Kicker kicker) {
    //addCommands(flywheel.runFlyWheelWithInput(1000), hood.positionHoodWithInput(0), spindexer.stopSpindexer(), kicker.stopKicker());
  }
}
