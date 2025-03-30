package com.spartronics4915.frc2025.commands;
import com.spartronics4915.frc2025.Robot;
import com.spartronics4915.frc2025.Constants.IntakeConstants.IntakeSpeed;
import com.spartronics4915.frc2025.commands.VariableAutos.BranchHeight;
import com.spartronics4915.frc2025.subsystems.coral.ArmSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ElevatorSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.IntakeSubsystem;
import com.spartronics4915.frc2025.util.CoralSim;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.spartronics4915.frc2025.Constants.DynamicsConstants.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

public class DynamicsCommandFactory {

    public IntakeSubsystem intakeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;

    private Timer lastScoredTimer = new Timer();

    private LaserCan funnelLC;
    public Trigger hasScoredTrigger = new Trigger(this::isCoralInArm).negate().debounce(kScoreLaserCanDebounce);

    private DynaPreset lastInputtedPreset = DynaPreset.L4;

    public DynamicsCommandFactory(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.armSubsystem = armSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.funnelLC = new LaserCan(kFunnelLaserCanID);

        try {
            funnelLC.setRangingMode(LaserCan.RangingMode.SHORT);
            funnelLC.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4)); // prev numbers that worked(8, 8, 4, 4)
            funnelLC.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
          }

        var tab = Shuffleboard.getTab("dynamicsLogging");
        tab.addBoolean("armBelowHorizon", this::isArmBelowHorizon);
        tab.addBoolean("armStowed", this::isArmStowed);
        tab.addBoolean("isElevSafeToMove", this::isElevSafeToMove);
        tab.addBoolean("isElevStowed", this::isElevStowed);
        tab.addBoolean("coralInArm", this::isCoralInArm);
        tab.addBoolean("funnelIntake", this::funnelDetect);
        tab.addBoolean("swerveSafeToMove", this::isSwerveMovable);
        tab.add("CommandScheduler", CommandScheduler.getInstance());

        lastScoredTimer.start();
        hasScoredTrigger.onTrue(Commands.runOnce(() -> lastScoredTimer.reset()));
    }

    private record DynamicsSetpoint(double heightMeters, Rotation2d armAngle) {
    }

    public enum DynaPreset{
        RESET(0.0, Rotation2d.fromDegrees(270)),
        LOAD(0.0, Rotation2d.fromDegrees(234.4421)),
        PRESCORE(0.2, Rotation2d.fromDegrees(kSafeArmAngle.in(Degrees))),//114.173111)),
        AUTO_PRESCORE(0.4, Rotation2d.fromDegrees(kSafeArmAngle.in(Degrees))),//114.173111)),
        L1(0.0, Rotation2d.fromDegrees(20)),
        L2(0.0, Rotation2d.fromDegrees(47.900)),
        L3(Meters.of(0.23939+0.1524-0.0254).in(Meters), Rotation2d.fromDegrees(58.10311200000001)),
        L4(Meters.of(1.23).in(Meters), Rotation2d.fromDegrees(14.33)),
        CLIMB(0.0, Rotation2d.fromDegrees(270+40)),
        ALGAE_HIGH(0.78 - 0.03, Rotation2d.fromDegrees(90)),
        ALGAE_LOW(0.375 - 0.03, Rotation2d.fromDegrees(90));

        private final DynamicsSetpoint setpoint;

        public Rotation2d getArmAngle() {
            return this.setpoint.armAngle;
        }
        public double getElevatorHeight() {
            return this.setpoint.heightMeters;
        }

        private DynaPreset(double meters, Rotation2d angle) {
            this.setpoint = new DynamicsSetpoint(meters, angle);
        }
    }

    private double getElevHeight(){
        return RobotBase.isSimulation() ? elevatorSubsystem.getDesiredPosition().in(Meters) : elevatorSubsystem.getPosition();
    }

    private Rotation2d getArmRotation(){
        return RobotBase.isSimulation() ? armSubsystem.getTargetPosition() : armSubsystem.getPosition();
    }

    //#region Composite Commands

    /**
     * 
     * @return Whether the elevator is safe to move (based on the arm's position)
     */
    private boolean isElevSafeToMove(){
        var currAngle =  getArmRotation();
        return currAngle.getDegrees() > kMoveableArmAngle.in(Degrees); 
    }

    private boolean isElevAtSetpoint(double setpoint){
        return Math.abs(setpoint - getElevHeight()) < 2*kElevatorHeightTolerance;
    }

    private boolean isArmAtSetpoint(Rotation2d angle){
        return isArmAtSetpoint(angle, kArmAngleTolerance);
    }

    private boolean isArmAtSetpoint(Rotation2d angle, Angle tolerance){
        return getArmRotation().minus(angle).getMeasure().isNear(Degrees.of(0), tolerance);
    }

    /**
     * @return Whether the arm is below the horizon and the elevator is too low to allow movement
     */
    private boolean isArmBelowHorizon(){
        return (getArmRotation().getDegrees() > 180);
    }

    /**
     * @return Whether the arm is below the horizon and the elevator is too low to allow movement
     */
    private boolean isArmStowed(){
        return isArmBelowHorizon() && isElevStowed();
    }

    private boolean isElevStowed(){
        return  getElevHeight() + kElevatorHeightTolerance < kMinSafeElevHeight;
    }

    public boolean isCoralInArm(){
        // return false;
        return intakeSubsystem.detect();
    }

    public boolean isElevatorForceable(){
        if (getElevHeight() <= kMinSafeElevHeight) {
            return true;
        } else {
            return false;
        }
    }

    public boolean funnelDetect(){

        if (RobotBase.isSimulation()) {
            return CoralSim.getFunnelLC() || intakeSubsystem.detect();
        }

        var measurement = funnelLC.getMeasurement();

        if (measurement == null) {
            return false;
        }

        return  measurement.distance_mm < funnelLCTriggerDist.in(Millimeter) || intakeSubsystem.detect(); // the || is here as a way to prevent us stalling at a CS when we are already holding a coral
    }

    public boolean canAutoScore(){
        return (elevatorSubsystem.getPosition() > DynaPreset.L4.getElevatorHeight() - kElevatorHeightTolerance) && 
            intakeSubsystem.branchLC() && 
            isCoralInArm();
    }

    public boolean isSwerveMovable(){
        return (getElevHeight() < kSafeElevHeightForSwerve) && isElevSafeToMove();
    }

    public boolean hasNotJustScored() {
        return lastScoredTimer.hasElapsed(kJustScoredThreshold.in(Seconds));
    }

    private Command makeElevatorSafeToMove(){
        return Commands.sequence(
                Commands.waitUntil(this::isElevSafeToMove),
                elevatorSubsystem.setSetPointCommand(kElevatorSafeHeightSetpoint),
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
            
            Angle safeArmSetpoint = (isSetpointBelowHorizon && kSafeArmAngle.gt(getArmRotation().getMeasure())) ? kReturnArmAngle : kSafeArmAngle;

            Command makeArmAngleSafe = armSubsystem.setSetpointCommand(new Rotation2d(safeArmSetpoint));

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
        }, Set.of())
        .withName("Make System Safe");
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

    private Command elevatorConcurrentMove(DynamicsSetpoint setpoint, Rotation2d concurrentAngle){
        return Commands.sequence(
            armSubsystem.setSetpointCommand(concurrentAngle),
            elevatorPriorityMove(setpoint)
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

    private Command armConcurrentMove(DynamicsSetpoint setpoint, double concurrentHeightMeters){
        return Commands.sequence(
            elevatorSubsystem.setSetPointCommand(concurrentHeightMeters),
            armPriorityMove(setpoint)
        );
    }

    public boolean isAtSetpoint(DynaPreset setpoint) {
        return isElevAtSetpoint(setpoint.setpoint.heightMeters) && isArmAtSetpoint(setpoint.setpoint.armAngle, (DriverStation.isAutonomous()) ? kArmAngleAutoScoringTolerance : kArmAngleTolerance);
    }

    public Command waitUntilPreset(DynaPreset setpoint){
        return Commands.waitUntil(() -> isAtSetpoint(setpoint));
    }

    //#endregion

    //#region small Commands

    public Command scoreHeight(DynaPreset scoringPoint){
        return Commands.sequence(
            makeSystemSafeToMove(false, scoringPoint.setpoint.heightMeters < kMinSafeElevHeight, false),
            elevatorConcurrentMove(scoringPoint.setpoint, new Rotation2d(kSafeArmAngle))
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
    
    public Command autoPrescore(){
        return Commands.sequence(
            makeSystemSafeToMove(true, false, false),
            armPriorityMove(DynaPreset.AUTO_PRESCORE.setpoint) //using arm Priority allows the arm to goto the right place then move the elevator down to the needed position 
        );
    }


    //#endregion

    public Command stow(){
        return Commands.either(
            prescoreStow(), 
            returnLoadStow(), 
            this::isCoralInArm
        )
        .withName("Stow");
    }

    public Command gotoScore(DynaPreset scorePreset){
        return scoreHeight(scorePreset)
        .withName("Goto " + scorePreset);
    }

    public Command gotoLastInputtedScore() {
        return Commands.defer(() -> gotoScore(lastInputtedPreset), Set.of())
        .withName("Goto Last");
    }

    /**
     * Runs gotoScore() and saves the input so we can automatically go there next time
     */
    public Command operatorScore(DynaPreset preset) {
        return Commands.runOnce(() -> lastInputtedPreset = preset)
                       .andThen(gotoScore(preset))
                       .withName("Operator Goto " + preset);
    }

    public Command gotoClimb(){
        return Commands.sequence(
            makeSystemSafeToMove(true, false, true),
            armPriorityMove(DynaPreset.CLIMB.setpoint)
        ).onlyIf(() -> !(isAtSetpoint(DynaPreset.CLIMB)))
        .withName("Goto Climb");
    }

    public Command score(){
        return Commands.deadline(
            Commands.waitUntil(
                hasScoredTrigger
            ).withTimeout(0.5),
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.OUT)
        ).andThen(intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.NEUTRAL))
        .andThen(checkIfScored().onlyIf(DriverStation::isTeleop))
        .withName("Score");
    }

    public Command checkIfScored() {
        return Commands.sequence(
            Commands.waitTime(kCheckIfScoredDelay),
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.IN),
            Commands.race(Commands.waitUntil(this::isCoralInArm), Commands.waitTime(kCheckIfScoredDuration)),
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.NEUTRAL),
            gotoLastInputtedScore().onlyIf(this::isCoralInArm)
        );
    }

    public Command autoScore(){
        return //Commands.deadline(
            // Commands.waitUntil(
            //     hasScoredTrigger
            // ).withTimeout(0.5),
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.OUT);
        // ).withName("Autonomous Score");
    }

    public Command returnLoadStow(){
        return Commands.sequence(
            makeSystemSafeToMove(isElevatorForceable(), false, true),
            armConcurrentMove(DynaPreset.LOAD.setpoint, kMinSafeElevHeight)
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
            ).withTimeout(RobotBase.isSimulation() ? 0.5 : 15)
        )
        .withName("Blocking Intake");
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
        )
        .withName("Intake");
    }

    public Command stopIntake(){
        return intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.NEUTRAL);
    }

    public Command removeAlgaeArm() {
        return armSubsystem.setSetpointCommand(new Rotation2d(kRemoveAlgaeArmAngle))
               .alongWith(intake())
               .onlyIf(() -> !isArmStowed());
    }

    public Command resetDynamics() {
        return Commands.sequence(
            makeSystemSafeToMove(true, false, true),
            armPriorityMove(DynaPreset.RESET.setpoint)
        ).withName("Reset Dynamics");
    }
}