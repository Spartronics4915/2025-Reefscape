package com.spartronics4915.frc2025.subsystems.coral;
import com.spartronics4915.frc2025.Constants.IntakeConstants.IntakeSpeed;
import com.spartronics4915.frc2025.commands.VariableAutos.BranchHeight;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.spartronics4915.frc2025.Constants.DynamicsConstants.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Radians;

import java.util.Set;

public class DynamicsCommandFactory {

    private IntakeSubsystem intakeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;

    private LaserCan funnelLC;
    public Trigger hasScoredTrigger = new Trigger(this::isCoralInArm).negate().debounce(kScoreLaserCanDebounce);

    private DynaPreset lastInputtedPreset = DynaPreset.L4;

    public DynamicsCommandFactory(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.armSubsystem = armSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.funnelLC = new LaserCan(kFunnelLaserCanID);

        var tab = Shuffleboard.getTab("dynamicsLogging");
        tab.addBoolean("armBelowHorizon", this::isArmBelowHorizon);
        tab.addBoolean("armStowed", this::isArmStowed);
        tab.addBoolean("isElevSafeToMove", this::isElevSafeToMove);
        tab.addBoolean("isElevStowed", this::isElevStowed);
        tab.addBoolean("coralInArm", this::isCoralInArm);
        tab.addBoolean("funnelIntake", this::funnelDetect);
        tab.add("CommandScheduler", CommandScheduler.getInstance());




    }

    private record DynamicsSetpoint(double heightMeters, Rotation2d armAngle) {
    }

    public enum DynaPreset{
        LOAD(0.0, Rotation2d.fromDegrees(237.789818)),
        PRESCORE(0.0, Rotation2d.fromDegrees(kSafeArmAngle.in(Degrees))),//114.173111)),
        L1(0.1, Rotation2d.fromDegrees(47.900)),
        L2(0.0, Rotation2d.fromDegrees(47.900)),
        L3(Meters.of(0.23939+0.1524-0.0254).in(Meters), Rotation2d.fromDegrees(58.10311200000001)),
        L4(Meters.of(1.25).in(Meters), Rotation2d.fromDegrees(15));

        private final DynamicsSetpoint setpoint;

        private DynaPreset(double meters, Rotation2d angle) {
            this.setpoint = new DynamicsSetpoint(meters, angle);
        }
    }

    //#region Composite Commands


    //TODO swap out target with actual position

    /**
     * 
     * @return Whether the elevator is safe to move (based on the arm's position)
     */
    private boolean isElevSafeToMove(){
        var currAngle =  armSubsystem.getPosition();
        return currAngle.getCos() < Math.cos(kMoveableArmAngle.in(Radians)); //TODO measure this so it's only if it's above the horizon (for climb)
    }

    private boolean isElevAtSetpoint(double setpoint){
        System.out.println(Math.abs(setpoint - elevatorSubsystem.getPosition()));
        return Math.abs(setpoint - elevatorSubsystem.getPosition()) < 2*kElevatorHeightTolerance;
    }

    private boolean isArmAtSetpoint(Rotation2d angle){
        return armSubsystem.getPosition().minus(angle).getMeasure().isNear(Degrees.of(0), kArmAngleTolerance);
    }

    /**
     * @return Whether the arm is below the horizon and the elevator is too low to allow movement
     */
    private boolean isArmBelowHorizon(){
        return (armSubsystem.getPosition().getDegrees() > 180);
    }

    /**
     * @return Whether the arm is below the horizon and the elevator is too low to allow movement
     */
    private boolean isArmStowed(){
        return isArmBelowHorizon() && isElevStowed();
    }

    private boolean isElevStowed(){
        return  elevatorSubsystem.getPosition() + kElevatorHeightTolerance < kMinSafeElevHeight;
    }

    private boolean isCoralInArm(){
        // return false;
        return intakeSubsystem.detect();
    }

    public boolean funnelDetect(){

        var measurement = funnelLC.getMeasurement();

        if (measurement == null) {
            return false;
        }

        return  measurement.distance_mm < funnelLCTriggerDist.in(Millimeter) || intakeSubsystem.detect(); // the || is here as a way to prevent us stalling at a CS when we are already holding a coral
    }

    private Command makeElevatorSafeToMove(){
        return Commands.sequence(
                Commands.waitUntil(this::isElevSafeToMove),
                elevatorSubsystem.setSetPointCommand(kMinSafeElevHeight),
                Commands.waitUntil(() -> !this.isElevStowed())
            ); 
    }

    /**
     * If the elevator is not in the load position, go to the safe elevator height.
     * Then, move the arm such that it is safe to move (meaning it won't hit the reef).
     */
    private Command makeSystemSafeToMove(boolean forceElevatorMovement, boolean forceArmMovement, boolean isSetpointBelowHorizon){ 

        //note to self, careful about when data gets read here
        return Commands.defer(() -> {

            // it shouldn't realistically be possible for both of these to be true unless in the climb position during teleop
            
            Command makeArmAngleSafe = armSubsystem.setSetpointCommand(new Rotation2d(kSafeArmAngle));

            Command moveElevatorFirstIfRequired = (this.isArmStowed() || forceElevatorMovement) && this.isElevSafeToMove()  ? makeElevatorSafeToMove() : Commands.none();

            Command makeArmSafeIfNeeded = !this.isElevSafeToMove() || forceArmMovement  ? makeArmAngleSafe : Commands.none();

            Command waitUntilElevSafeToMove = Commands.waitUntil(this::isElevSafeToMove).withTimeout(1.0);

            Command moveElevatorIfNeeded = (this.isArmStowed() || forceElevatorMovement) ? makeElevatorSafeToMove() : Commands.none();
            
            ParallelRaceGroup WaitUntilSafeToMove = Commands.waitUntil(() -> {
                // Are the setpoint and current arm angle on the same side of the horizon
                boolean sameSide = !(this.isArmBelowHorizon() ^ isSetpointBelowHorizon);

                // If they are on the same side then the arm is safe to move, if they aren't on the same side then wait until the elevator isn't stowed
                boolean isArmSafeToMove = sameSide || !this.isElevStowed();

                // If the elevator and arm are safe to move then continue
                return this.isElevSafeToMove() && isArmSafeToMove;
            }).withTimeout(1.0);

            // if the elev isn't safe to move (ie it would hook the reef) it should move the arm
            // if the arm is stowed then the elevator should move first, then bring the arm up 
            return Commands.sequence(
                moveElevatorFirstIfRequired,
                makeArmSafeIfNeeded,
                waitUntilElevSafeToMove,
                moveElevatorIfNeeded,
                WaitUntilSafeToMove
            );
        }, Set.of());
    }

    /**
     * Moves the elevator first before moving the arm
     */
    private Command elevatorPriorityMove(DynamicsSetpoint setpoint){
        return Commands.parallel(
            elevatorSubsystem.setSetPointCommand(setpoint.heightMeters),
            Commands.waitUntil(() -> isElevAtSetpoint(setpoint.heightMeters)).withTimeout(2).andThen(
                armSubsystem.setSetpointCommand(setpoint.armAngle)
            )
        );
    }

    /**
     * Moves the arm first before moving the elevator
     */
    private Command armPriorityMove(DynamicsSetpoint setpoint){
        return Commands.parallel(
            armSubsystem.setSetpointCommand(setpoint.armAngle),
            Commands.waitUntil(() -> isArmAtSetpoint(setpoint.armAngle)).withTimeout(2).andThen(
                elevatorSubsystem.setSetPointCommand(setpoint.heightMeters)
            )
        );
    }

    //#endregion

    //#region small Commands

    public Command scoreHeight(DynaPreset scoringPoint){
        return Commands.sequence(
            makeSystemSafeToMove(false, scoringPoint.setpoint.heightMeters < kMinSafeElevHeight, false),
            elevatorPriorityMove(scoringPoint.setpoint)
        );
    }

    public Command loadStow(){
        return Commands.sequence(
            makeSystemSafeToMove(true, false, true),
            armPriorityMove(DynaPreset.LOAD.setpoint) //brings arm to the load angle, then drops the elevator
        );
    }

    public Command prescoreStow(){
        return Commands.sequence(
            makeSystemSafeToMove(false, false, false),
            armPriorityMove(DynaPreset.PRESCORE.setpoint) //using arm Priority allows the arm to goto the right place then move the elevator down to the needed position 
        );
    }


    //#endregion

    public Command stow(){
        return Commands.either(
            prescoreStow(), 
            loadStow(), 
            this::isCoralInArm
        );
    }

    public Command gotoScore(DynaPreset scorePreset){
        return scoreHeight(scorePreset);
    }

    public Command gotoLastInputtedScore() {
        return Commands.defer(() -> gotoScore(lastInputtedPreset), Set.of());
    }

    /**
     * Runs gotoScore() and saves the input so we can automatically go there next time
     */
    public Command operatorScore(DynaPreset preset) {
        return Commands.runOnce(() -> lastInputtedPreset = preset)
                       .andThen(gotoScore(preset));
    }

    public Command score(){
        return Commands.deadline(
            Commands.waitUntil(
                hasScoredTrigger
            ).withTimeout(1.0),
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.OUT)
        ).andThen(intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.NEUTRAL));
    }

    public Command autoScore(DynaPreset scoringLocation){
        return Commands.sequence(
            Commands.waitUntil(() -> 
                isArmAtSetpoint(scoringLocation.setpoint.armAngle) && 
                isElevAtSetpoint(scoringLocation.setpoint.heightMeters)
            ),
            score()
        );
    }

    /**
     * Starts the intake immediately and ends the command once the funnel or manipulator LaserCAN detects coral. This will not stop the intake
     */
    public Command blockingIntake(){
        return Commands.sequence(
            intake(),
            Commands.waitUntil(
                () -> funnelDetect() || isCoralInArm()
            ).withTimeout(3) //TODO remove for comps
        );
    }

    /**
     * Intakes if there is no coral in the manipulator
     * @return a Command that will do the above actions 
     */
    public Command intake(){
        return Commands.either(
            Commands.none(),
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.IN),
            this::isCoralInArm
        );
    }
}