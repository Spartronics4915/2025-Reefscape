package com.spartronics4915.frc2025.commands;

import com.spartronics4915.frc2025.commands.VariableAutos.FieldBranch;
import com.spartronics4915.frc2025.commands.VariableAutos.StationSide;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

import static edu.wpi.first.units.Units.Seconds;

import com.spartronics4915.frc2025.Constants.Drive.AutoConstants.DefaultAutos;
import com.spartronics4915.frc2025.Constants.Drive.AutoConstants.StationVisualizationConstants;
import com.spartronics4915.frc2025.commands.VariableAutos.BranchHeight;
import com.spartronics4915.frc2025.commands.VariableAutos.BranchSide;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ComplexAutoChooser extends SubsystemBase {
    private class VariableAutoSegment {
        private SendableChooser<FieldBranch> branchChooser = new SendableChooser<>();
        private SendableChooser<BranchHeight> heightChooser = new SendableChooser<>();

        private int index;
        private static int count = 1;

        protected VariableAutoSegment() {
            index = count++;
            String path = "Variable Autos/Step " + index + "/";
            buildBranchChooser();
            setDefaultBranch();
            setBranchPreview(getFieldBranch());
            buildHeightChooser();
            SmartDashboard.putData(path + "Score on...", branchChooser);
            SmartDashboard.putData(path + "At...", heightChooser);
            SmartDashboard.putNumber(path + "Then wait...", 0.0);
        }

        private void buildBranchChooser() {
            if (index > 1) {
                branchChooser.setDefaultOption("A", FieldBranch.A);
                branchChooser.addOption("B", FieldBranch.B);
                branchChooser.addOption("C", FieldBranch.C);
                branchChooser.addOption("D", FieldBranch.D);
                branchChooser.addOption("E", FieldBranch.E);
                branchChooser.addOption("K", FieldBranch.K);
                branchChooser.addOption("L", FieldBranch.L);
            } else {
                branchChooser.setDefaultOption("E", FieldBranch.E);
            }
            branchChooser.addOption("F", FieldBranch.F);
            branchChooser.addOption("G", FieldBranch.G);
            branchChooser.addOption("H", FieldBranch.H);
            branchChooser.addOption("I", FieldBranch.I);
            branchChooser.addOption("J", FieldBranch.J);

            branchChooser.onChange(this::setBranchPreview);
        }

        private void buildHeightChooser() {
            heightChooser.setDefaultOption("L4", BranchHeight.L4);
            heightChooser.addOption("L3", BranchHeight.L3);
            heightChooser.addOption("L2", BranchHeight.L2);
            heightChooser.addOption("L1", BranchHeight.L1);
        }

        private void setDefaultBranch() {
            boolean useLeft = DriverStation.getLocation().orElse(3) == 1;
            FieldBranch[] defaults = useLeft ? DefaultAutos.kLeft : DefaultAutos.kRight;
            if (defaults.length >= index) {
                FieldBranch defaultBranch = defaults[index - 1];
                branchChooser.setDefaultOption(defaultBranch.name(), defaultBranch);
            }
        }

        protected void setBranchPreview(FieldBranch branch) {
            var reefSide = branch.simpleBranchInfo.reefSide();
            var branchSide = getBranchHeight() == BranchHeight.L1 ? BranchSide.MIDDLE : branch.simpleBranchInfo.branchSide();

            Translation2d branchPose = reefSide.getCurrent().getTranslation().plus(
                new Translation2d(
                    0.2,//branchSide.tagOffset.getY(),
                    branchSide.tagOffset.getX() * 1.5
                ).rotateBy(reefSide.getCurrent().getRotation())
            );

            previewField.getObject("Branch " + index).setPose(new Pose2d(branchPose, reefSide.getCurrent().getRotation()));
        }

        protected FieldBranch getFieldBranch() {
            return branchChooser.getSelected();
        }
        protected BranchHeight getBranchHeight() {
            return heightChooser.getSelected();
        }
        protected Time getDelayAfterScoring() {
            double input = SmartDashboard.getNumber("Variable Autos/Step " + index + "/Then wait...", 0.0);
            input = Math.max(0.0, input);
            return Seconds.of(input);
        }
    }

    private static class SingleRun {
        private static SendableChooser<FieldBranch> branchChooser = new SendableChooser<>();
        private static SendableChooser<BranchHeight> heightChooser = new SendableChooser<>();
        private static boolean initialized = false;

        protected static void initSingleRun() {
            if (initialized) return;
            branchChooser.setDefaultOption("A", FieldBranch.A);
            branchChooser.addOption("B", FieldBranch.B);
            branchChooser.addOption("C", FieldBranch.C);
            branchChooser.addOption("D", FieldBranch.D);
            branchChooser.addOption("E", FieldBranch.E);
            branchChooser.addOption("F", FieldBranch.F);
            branchChooser.addOption("G", FieldBranch.G);
            branchChooser.addOption("H", FieldBranch.H);
            branchChooser.addOption("I", FieldBranch.I);
            branchChooser.addOption("J", FieldBranch.J);
            branchChooser.addOption("K", FieldBranch.K);
            branchChooser.addOption("L", FieldBranch.L);

            heightChooser.setDefaultOption("L4", BranchHeight.L4);
            heightChooser.addOption("L3", BranchHeight.L3);
            heightChooser.addOption("L2", BranchHeight.L2);

            SmartDashboard.putData("Variable Autos/Single Run/Branch", branchChooser);
            SmartDashboard.putData("Variable Autos/Single Run/Height", heightChooser);
            SmartDashboard.putBoolean("Variable Autos/Single Run/Repeat", false);

            initialized = true;
        }

        private static Command getSingleRun(VariableAutos factory, StationSide side) {
            Command run = factory.generateAutoCycle(branchChooser.getSelected(), side, heightChooser.getSelected());
            if (SmartDashboard.getBoolean("Variable Autos/Single Run/Repeat", false)) {
                return Commands.repeatingSequence(run);
            }
            return run;
        }
    }

    private VariableAutoSegment[] segments;
    private VariableAutos factory;
    private SendableChooser<StationSide> stationChooser = new SendableChooser<>();
    private SwerveSubsystem swerve;
    private Field2d previewField;

    public ComplexAutoChooser(VariableAutos factory, int length, SwerveSubsystem swerve) {
        this.factory = factory;
        this.swerve = swerve;
        previewField = new Field2d();
        buildStationChooser();
        setDefaultStation();
        setStationPreview(stationChooser.getSelected());
        SmartDashboard.putData("Variable Autos/Station", stationChooser);
        segments = new VariableAutoSegment[length];
        for (int i = 0; i < length; i++) {
            segments[i] = new VariableAutoSegment();
        }

        SmartDashboard.putData("Preview Field", previewField);

        SingleRun.initSingleRun();
    }

    private void buildStationChooser() {
        stationChooser.setDefaultOption("Left", StationSide.LEFT);
        stationChooser.addOption("Right", StationSide.RIGHT);
        stationChooser.onChange(this::setStationPreview);
    }

    private void setDefaultStation() {
        boolean useLeft = DriverStation.getLocation().orElse(3) == 1;
        if (useLeft) stationChooser.setDefaultOption("Left", StationSide.LEFT);
        else stationChooser.setDefaultOption("Right", StationSide.RIGHT);
    }

    private void setStationPreview(StationSide side) {
        FieldObject2d station = previewField.getObject("Station");
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            station.setPose(side == StationSide.LEFT ? StationVisualizationConstants.kBlueLeft : StationVisualizationConstants.kBlueRight);
        } else {
            station.setPose(side == StationSide.LEFT ? StationVisualizationConstants.kRedLeft : StationVisualizationConstants.kRedRight);
        }
    }

    public void updatePreviewField(Pose2d robotPose) {
        previewField.setRobotPose(robotPose);
        SmartDashboard.putData("Preview Field", previewField);
    }

    public Command getAuto() {
        Command[] commands = new Command[segments.length];
        for (int i = 0; i < segments.length; i++) {
            VariableAutoSegment segment = segments[i];
            if (i == 0) {
                commands[i] = factory.generateStartingAutoCycle(
                    segment.getFieldBranch(),
                    stationChooser.getSelected(),
                    segment.getBranchHeight(),
                    segment.getDelayAfterScoring()
                );
            } else {
                commands[i] = factory.generateAutoCycle(
                    segment.getFieldBranch(),
                    stationChooser.getSelected(),
                    segment.getBranchHeight(),
                    segment.getDelayAfterScoring()
                );
            }
        }
        return Commands.sequence(commands);
    }

    public Command getSingleRun() {
        return SingleRun.getSingleRun(factory, stationChooser.getSelected());
    }

    @Override
    public void periodic() {
        if (DriverStation.isAutonomous()) updatePreviewField(swerve.getPose());
    }
}
