package com.spartronics4915.frc2025.subsystems.coral;
import com.pathplanner.lib.auto.NamedCommands;
import com.spartronics4915.frc2025.RobotContainer;
import com.spartronics4915.frc2025.Constants.IntakeConstants.IntakeSpeed;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.spartronics4915.frc2025.Constants.DynamicsConstants.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class DynamicsCommandFactory {

    private IntakeSubsystem intakeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;

    private LaserCan funnelLC;
    public Trigger hasScoredTrigger = new Trigger(this::isCoralInArm).negate().debounce(kScoreLaserCanDebounce);

    public DynamicsCommandFactory(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.armSubsystem = armSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.funnelLC = new LaserCan(kFunnelLaserCanID);
    }

    private record DynamicsSetpoint(double heightMeters, Rotation2d armAngle) {
    }

    public enum DynaPreset{
        LOAD(0.0, Rotation2d.fromDegrees(237.789818)),
        PRESCORE(0.0, Rotation2d.fromDegrees(114.173111)),
        L2(0.0, Rotation2d.fromDegrees(319.357058)),
        L3(Meters.of(0.23939+0.1524-0.0254).in(Meters), Rotation2d.fromDegrees(58.10311200000001)),
        L4(Meters.of(1.25).in(Meters), Rotation2d.fromDegrees(10));

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
        return currAngle.getCos() < Math.cos(kMoveableArmAngle.in(Radians));
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

    /**
     * If the elevator is not in the load position, go to the safe elevator height.
     * Then, move the arm such that it is safe to move (meaning it won't hit the reef).
     */
    private Command makeSystemSafeToMove(boolean forceMinSafeHeightMove){ 
        //note to self, careful about when data gets read here
        return Commands.either(
            Commands.sequence(
                armSubsystem.setSetpointCommand(new Rotation2d(kSafeArmAngle)),
                Commands.either(
                    Commands.waitUntil(this::isElevSafeToMove).andThen(
                        elevatorSubsystem.setSetPointCommand(kMinSafeElevHeight).andThen(
                        Commands.waitUntil(() -> !this.isElevStowed()) //ensures elevator is at a height so it can move
                    )), 
                    Commands.none(), 
                    () -> {return this.isArmStowed() || forceMinSafeHeightMove;}
                )
            ), 
            Commands.none(),
            () -> {return !this.isElevSafeToMove() || (this.isArmStowed() || forceMinSafeHeightMove);}
        ).andThen(
            Commands.waitUntil(() -> {
                boolean isArmSafeToMove = !(this.isElevStowed() || this.isArmBelowHorizon()); //make sure the elevator isn't stowed or the arm isn't below the horizon
                return this.isElevSafeToMove() && !isArmSafeToMove;
            }).withTimeout(1.0)
        );
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

    public Command scoreHeight(DynamicsSetpoint scoringPoint){
        return Commands.sequence(
            makeSystemSafeToMove(false),
            elevatorPriorityMove(scoringPoint)
        );
    }

    public Command loadStow(){
        return Commands.sequence(
            makeSystemSafeToMove(false),
            armPriorityMove(DynaPreset.LOAD.setpoint) //brings arm to the load angle, then drops the elevator
        );
    }

    public Command prescoreStow(){
        return Commands.sequence(
            makeSystemSafeToMove(false),
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
        return scoreHeight(scorePreset.setpoint);
    }

    public Command score(){
        return Commands.deadline(
            Commands.waitUntil(
                hasScoredTrigger
            ).withTimeout(1.0),
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.OUT)
        ).andThen(intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.NEUTRAL));
    }

    /**
     * Starts the intake immediately and ends the command once the funnel or manipulator LaserCAN detects coral. This will not stop the intake
     */
    public Command blockingIntake(){
        return Commands.sequence(
            intake(),
            Commands.waitUntil(
                () -> funnelDetect() || isCoralInArm()
            ).withTimeout(1.5)
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