package com.spartronics4915.frc2025.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static com.spartronics4915.frc2025.commands.drive.ChassisSpeedSuppliers.*;

import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.Constants.OI;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

public class SwerveTeleopCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;
    private boolean useFieldRelative;

    public SwerveTeleopCommand(CommandXboxController driverController, SwerveSubsystem swerveSubsystem) {

        this.swerveSubsystem = swerveSubsystem;

        this.driverController = driverController;

        setFieldRelative(false);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        ChassisSpeeds cs = computeVelocitiesFromController(driverController.getHID(), getFieldRelative(), swerveSubsystem).get();

        swerveSubsystem.drive(cs);
    }


    public void setFieldRelative(boolean fieldRelative) {
        useFieldRelative = fieldRelative;
    }

    public boolean getFieldRelative() {
        return useFieldRelative;
    }
}
