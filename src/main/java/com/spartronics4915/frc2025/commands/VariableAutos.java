package com.spartronics4915.frc2025.commands;

import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kPathConstraints;
import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kStartingPathConstraints;
import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kStationApproachSpeed;
import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kStationApproachTimeout;
import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kTriggerDistance;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.spartronics4915.frc2025.RobotContainer;
import com.spartronics4915.frc2025.commands.Autos.AutoPaths;
import com.spartronics4915.frc2025.commands.DynamicsCommandFactory.DynaPreset;
import com.spartronics4915.frc2025.commands.autos.AlignToReef;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.bling2.DriverCommunication.Region;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class VariableAutos {

    public enum BranchHeight{
        L4(DynaPreset.L4),
        L3(DynaPreset.L3),
        L2(DynaPreset.L2),
        ;

        public final DynaPreset preset;

        private BranchHeight(DynaPreset preset) {
            this.preset = preset;
        }


    }

    public enum FieldBranch{
        A(BranchSide.LEFT, ReefSide.ONE),
        B(BranchSide.RIGHT, ReefSide.ONE),
        C(BranchSide.LEFT, ReefSide.TWO),
        D(BranchSide.RIGHT, ReefSide.TWO),
        E(BranchSide.LEFT, ReefSide.THREE),
        F(BranchSide.RIGHT, ReefSide.THREE),
        G(BranchSide.LEFT, ReefSide.FOUR),
        H(BranchSide.RIGHT, ReefSide.FOUR),
        I(BranchSide.LEFT, ReefSide.FIVE),
        J(BranchSide.RIGHT, ReefSide.FIVE),
        K(BranchSide.LEFT, ReefSide.SIX),
        L(BranchSide.RIGHT, ReefSide.SIX);

        public SimpleBranch simpleBranchInfo;

        private FieldBranch(BranchSide branchSide, ReefSide reefSide) {
            this.simpleBranchInfo = new SimpleBranch(branchSide, reefSide);
        }
    }

    public record SimpleBranch(BranchSide branchSide, ReefSide reefSide) {
        public SimpleBranch mirror(){
            //TODO check if mirroring the branchside does work here
            return new SimpleBranch(branchSide.mirror(), reefSide.mirror());
        }
    }

    public enum BranchSide{
        LEFT(new Translation2d(-0.153209, 0.5406845)),
        RIGHT(new Translation2d(0.218062 - 0.0508, 0.5408565)),
        MIDDLE(new Translation2d(0.064853, 0.5408565));

        public Translation2d tagOffset;
        private BranchSide(Translation2d offsets) {
            tagOffset = offsets;
        }

        public BranchSide mirror(){
            switch (this) {
                case LEFT: return RIGHT;
                case MIDDLE: return MIDDLE;
                default: return LEFT;
            }
        }
    }

    public enum ReefSide{
        ONE(18, 7),
        SIX(19, 6),
        FIVE(20, 11),
        FOUR(21, 10),
        THREE(22, 9),
        TWO(17, 8);

        public final Pose2d redTagPose;
        public final Pose2d blueTagPose;

        public Pose2d getCurrent(){
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ?
                blueTagPose : 
                redTagPose;
        }


        public ReefSide mirror(){
            switch (this) {
                case ONE: return ONE;
                case TWO: return SIX;
                case THREE: return FIVE;
                case FOUR: return FOUR;
                case FIVE: return THREE;
                default: return TWO; //SIX case
            }
        }

        private ReefSide(int blue, int red) {
            var layout = RobotContainer.getFieldLayout();


            redTagPose =layout.getTagPose(red).get().toPose2d();
            blueTagPose = layout.getTagPose(blue).get().toPose2d();
        }
    } 

    public enum StationSide{
        LEFT,
        RIGHT;
    }

    public record PathPair(Command approachPath, Command autoAlign, Command returnPath) {}

    private AlignToReef alignmentGenerator;
    private DynamicsCommandFactory dynamics;
    private SwerveSubsystem swerve;

    private final ChassisSpeeds reverseIntoStation;

    public boolean isSwerveCloseToReef() {
        Translation2d currentReef = 
        (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) ?
            new Translation2d(4.5, 4.0) :
            new Translation2d(13, 4.0)
        ;

        return (currentReef.getDistance(swerve.getPose().getTranslation()) < kTriggerDistance);
    }

    public VariableAutos(AlignToReef alignmentGenerator, DynamicsCommandFactory dynamics, SwerveSubsystem swerve) {
        super();
        this.alignmentGenerator = alignmentGenerator;
        this.dynamics = dynamics;
        this.swerve = swerve;

        reverseIntoStation = new ChassisSpeeds(kStationApproachSpeed.unaryMinus().in(MetersPerSecond), 0, 0);
    }

    public Command generateAutoCycle(FieldBranch branch, StationSide side, BranchHeight height) {
        return generateAutoCycle(branch, side, height, Seconds.of(0));
    }

    public Command generateStartingAutoCycle(FieldBranch branch, StationSide side, BranchHeight height) {
        return generateStartingAutoCycle(branch, side, height, Seconds.of(0));
    }

    /**
     * Outputs the entire auto cycle from station to branch with mechanism movement
     */
    public Command generateAutoCycle(FieldBranch branch, StationSide side, BranchHeight height, Time delay) {
        var pathPair = getPathPair(branch, side);
        
        return Commands.sequence(
            Commands.deadline(
                pathPair.approachPath,
                Commands.sequence(
                    Commands.waitUntil(dynamics.intakeSubsystem::detect),
                    dynamics.autoPrescore()
                )
            ),
            Commands.parallel( //this is parallel so it hangs if there isn't coral in the intake
                pathPair.autoAlign,
                Commands.sequence(
                    Commands.waitUntil(() -> dynamics.intakeSubsystem.detect()),
                    Commands.print("moving to height"),
                    dynamics.gotoScore(height.preset)
                )
            ),
            Commands.print("end step"),
            dynamics.waitUntilPreset(height.preset),
            dynamics.score(),
            Commands.print("Stow & return"),
            Commands.parallel(
                dynamics.stow(),
                Commands.sequence(
                    Commands.print("start delay"),
                    Commands.waitTime(delay),
                    Commands.print("end delay"),
                    Commands.waitUntil(() -> dynamics.isSwerveMovable()),
                    Commands.print("returning path"),
                    pathPair.returnPath
                )
            ),
            Commands.print("blocking intake"),
            Commands.deadline(
                dynamics.blockingIntake(),
                Commands.run(() -> swerve.drive(reverseIntoStation)).withTimeout(kStationApproachTimeout)
            )
        ).withName("Auto cycle")
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command generateStartingAutoCycle(FieldBranch branch, StationSide side, BranchHeight height, Time delay) {
        var pathPair = getPathPair(branch, side);
        
        return Commands.sequence(
            Commands.runOnce(() -> {
                alignmentGenerator.changePathConstraints(kStartingPathConstraints); //!!! should this be in command sequence??? -shark
            }),
            Commands.print("auto align"),
            Commands.parallel(
                pathPair.autoAlign,
                Commands.sequence(
                    Commands.print("auto prescoure"),
                    dynamics.autoPrescore(),
                    Commands.print("is swerve close to reef?"),
                    Commands.waitUntil(() -> isSwerveCloseToReef()),
                    Commands.print("moving to height"),
                    dynamics.gotoScore(height.preset)
                )
            ),
            Commands.print("wait until preset"),
            dynamics.waitUntilPreset(height.preset),
            Commands.print("score"),
            dynamics.score(),
            Commands.print("parallel group stow"),
            Commands.parallel(
                dynamics.stow(),
                Commands.sequence(
                    Commands.waitTime(delay),
                    Commands.print("is swerve moveable?"),
                    Commands.waitUntil(() -> dynamics.isSwerveMovable()),
                    Commands.print("returning"),
                    pathPair.returnPath
                )
            ),
            Commands.deadline(
                dynamics.blockingIntake(),
                Commands.run(() -> swerve.drive(reverseIntoStation)).withTimeout(kStationApproachTimeout)
            )
        ).finallyDo(() -> {
            alignmentGenerator.changePathConstraints(kPathConstraints);
        }).withName("Starting Auto cycle")
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public PathPair getPathPair(FieldBranch branch, StationSide side){
        boolean shouldMirror = side == StationSide.RIGHT;

        var branchSide = branch.simpleBranchInfo.branchSide;
        var reefSide = branch.simpleBranchInfo.reefSide;

        return getPathPair(branchSide, reefSide, shouldMirror);
    }

    public PathPair getPathPair(BranchSide fieldBranchSide, ReefSide fieldReefSide, boolean mirror){
        
        var align = alignmentGenerator.generateCommand(fieldReefSide, fieldBranchSide);

        //The reason for the double mirroring here is a bit interesting
        //Basically if we are using the opposite station from what we made the paths on we want to act like the starting positon is mirrored
        //then the path to the coral station is correct, we then mirror it again so the starting position is the same and the path's end point is mirrored
        if (mirror) {
            fieldReefSide = fieldReefSide.mirror();
        }
        
        AutoPaths approachForLeftCS;
        switch (fieldReefSide) {
            case ONE: approachForLeftCS = AutoPaths.CORAL_ONE; break;
            case TWO: approachForLeftCS = AutoPaths.CORAL_TWO; break;
            case THREE: approachForLeftCS = AutoPaths.CORAL_THREE; break;
            case FOUR: approachForLeftCS = AutoPaths.CORAL_FOUR; break;
            case FIVE: approachForLeftCS = AutoPaths.CORAL_FIVE; break;
            default: approachForLeftCS = AutoPaths.CORAL_SIX; break;
        }

        return new PathPair(
            Autos.getAutoPathCommand(approachForLeftCS, mirror), 
            align, 
            Autos.getAutoPathCommand(approachForLeftCS.getReverse(), mirror)
        );
    }

}
