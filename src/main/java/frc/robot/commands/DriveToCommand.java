package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveToCommand extends Command {
    private final Drivetrain drivetrain;

    public DriveToCommand(Drivetrain m_drivetrain) {
        drivetrain = m_drivetrain;
        addRequirements(drivetrain);
    }

    @Override 
    public void initialize() {
        
    }

    @Override 
    public void execute() {
        
    }

    @Override 
    public void end(boolean interrupted) {
        
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
