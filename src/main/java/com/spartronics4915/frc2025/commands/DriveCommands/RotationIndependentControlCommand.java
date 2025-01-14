package com.spartronics4915.frc2025.commands.driveCommands;

import java.util.function.Supplier;

import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class RotationIndependentControlCommand extends Command {
    private SwerveSubsystem mSwerve = SwerveSubsystem.getInstance();

    private Supplier<ChassisSpeeds> linearCSSupplier;
    private Supplier<ChassisSpeeds> rotationalCSSupplier;

    public RotationIndependentControlCommand(
        Supplier<ChassisSpeeds> rotationalCSSupplier,
        Supplier<ChassisSpeeds> linearCSSupplier
    ) {
        this.rotationalCSSupplier = rotationalCSSupplier;
        this.linearCSSupplier = linearCSSupplier;

        addRequirements(mSwerve);
    }
    public RotationIndependentControlCommand(
        Supplier<ChassisSpeeds> cSSupplier
    ) {
        this.rotationalCSSupplier = cSSupplier;
        this.linearCSSupplier = cSSupplier;

        addRequirements(mSwerve);
    }

    @Override
    public void execute() {
        
        ChassisSpeeds linearCS = linearCSSupplier.get();
        ChassisSpeeds rotationalCS = rotationalCSSupplier.get();

        mSwerve.drive(new ChassisSpeeds(
            linearCS.vxMetersPerSecond,
            linearCS.vyMetersPerSecond, 
            rotationalCS.omegaRadiansPerSecond
        ));
    }
}
