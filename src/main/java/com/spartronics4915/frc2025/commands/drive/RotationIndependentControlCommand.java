package com.spartronics4915.frc2025.commands.drive;

import java.util.function.Supplier;

import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class RotationIndependentControlCommand extends Command {
    private SwerveSubsystem mSwerve;

    private Supplier<ChassisSpeeds> linearCSSupplier;
    private Supplier<ChassisSpeeds> rotationalCSSupplier;

    public RotationIndependentControlCommand(
        Supplier<ChassisSpeeds> rotationalCSSupplier,
        Supplier<ChassisSpeeds> linearCSSupplier,
        SwerveSubsystem swerve
    ) {
        this.rotationalCSSupplier = rotationalCSSupplier;
        this.linearCSSupplier = linearCSSupplier;
        mSwerve = swerve;

        addRequirements(mSwerve);
    }
    public RotationIndependentControlCommand(
        Supplier<ChassisSpeeds> cSSupplier,
        SwerveSubsystem swerve
    ) {
        this.rotationalCSSupplier = cSSupplier;
        this.linearCSSupplier = cSSupplier;
        mSwerve = swerve;

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
