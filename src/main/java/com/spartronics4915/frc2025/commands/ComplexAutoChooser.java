package com.spartronics4915.frc2025.commands;

import com.spartronics4915.frc2025.commands.VariableAutos.FieldBranch;
import com.spartronics4915.frc2025.commands.VariableAutos.StationSide;

import static edu.wpi.first.units.Units.Seconds;

import com.spartronics4915.frc2025.commands.VariableAutos.BranchHeight;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ComplexAutoChooser {
    private class VariableAutoSegment {
        private SendableChooser<FieldBranch> branchChooser = new SendableChooser<>();
        private SendableChooser<BranchHeight> heightChooser = new SendableChooser<>();

        private int index;
        private static int count = 1;

        protected VariableAutoSegment() {
            index = count++;
            String path = "Variable Autos/Step " + index + "/";
            buildBranchChooser();
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
        }

        private void buildHeightChooser() {
            heightChooser.setDefaultOption("L4", BranchHeight.L4);
            heightChooser.addOption("L3", BranchHeight.L3);
            heightChooser.addOption("L2", BranchHeight.L2);
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

    private VariableAutoSegment[] segments;
    private VariableAutos factory;
    private SendableChooser<StationSide> stationChooser = new SendableChooser<>();

    public ComplexAutoChooser(VariableAutos factory, int length) {
        this.factory = factory;
        buildStationChooser();
        SmartDashboard.putData("Variable Autos/Station", stationChooser);
        segments = new VariableAutoSegment[length];
        for (int i = 0; i < length; i++) {
            segments[i] = new VariableAutoSegment();
        }
    }

    private void buildStationChooser() {
        stationChooser.setDefaultOption("Left", StationSide.LEFT);
        stationChooser.addOption("Right", StationSide.RIGHT);
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
}
