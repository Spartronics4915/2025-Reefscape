package com.spartronics4915.frc2025.commands.autos;

import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FollowLineToReef extends SequentialCommandGroup{
    
    Translation2d approachPoint;
    public FollowLineToReef(SwerveSubsystem swerveSubsystem) {
        
 //       addCommands(new DriveToPoseCommand(null, null, null, 0, 0, swerveSubsystem));
    }
}
