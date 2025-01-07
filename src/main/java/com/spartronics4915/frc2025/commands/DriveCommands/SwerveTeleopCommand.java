package com.spartronics4915.frc2025.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.Constants.OI;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

public class SwerveTeleopCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;
    private boolean useFieldRelative;

    public SwerveTeleopCommand(SwerveSubsystem swerveSubsystem, CommandXboxController driverController) {

        this.swerveSubsystem = swerveSubsystem;
        this.driverController = driverController;

        setFieldRelative(true);
        addRequirements(swerveSubsystem);

    }

    static public ChassisSpeeds computeVelocitiesFromController(XboxController driverController) {

        ChassisSpeeds cs = new ChassisSpeeds();

        // Need to verify that we are using the right axes.
        final double inputxraw = driverController.getLeftY() * -1.0;
        final double inputyraw = driverController.getLeftX() * -1.0;
        final double inputomegaraw;
        if (RobotBase.isSimulation()) {
            inputomegaraw = driverController.getRawAxis(3) * -1.0;
        } else {
            inputomegaraw = driverController.getRightY() * -1.0; // consider changing from angular velocity
            // control to direct angle control
        }

        final double inputx = applyResponseCurve(MathUtil.applyDeadband(inputxraw, OI.kStickDeadband));
        final double inputy = applyResponseCurve(MathUtil.applyDeadband(inputyraw, OI.kStickDeadband));
        final double inputomega = applyResponseCurve(MathUtil.applyDeadband(inputomegaraw, OI.kStickDeadband));

        cs.vxMetersPerSecond = inputx * Drive.kMaxSpeed;
        cs.vyMetersPerSecond = inputy * Drive.kMaxSpeed;
        cs.omegaRadiansPerSecond = inputomega * Drive.kMaxAngularSpeed;

        return cs;
    }

    @Override
    public void execute() {

        ChassisSpeeds cs = computeVelocitiesFromController(driverController.getHID());

        boolean useFieldRelative = getFieldRelative();
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                && useFieldRelative) {
            cs.vxMetersPerSecond = -cs.vxMetersPerSecond;
            cs.vyMetersPerSecond = -cs.vyMetersPerSecond;
        }

        if (useFieldRelative) {
            swerveSubsystem.driveFieldOriented(cs);
        } else {
            swerveSubsystem.drive(cs);
        }
    }

    static private double applyResponseCurve(double x) {
        return Math.signum(x) * Math.pow(x, 2);
    }

    public void setFieldRelative(boolean fieldRelative) {
        useFieldRelative = fieldRelative;
    }

    public boolean getFieldRelative() {
        return useFieldRelative;
    }
}
