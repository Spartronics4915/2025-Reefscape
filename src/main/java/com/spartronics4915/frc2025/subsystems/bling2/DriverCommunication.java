package com.spartronics4915.frc2025.subsystems.bling2;

import com.spartronics4915.frc2025.Constants.OrientTowardsNearestPOIConstants;
import com.spartronics4915.frc2025.commands.DynamicsCommandFactory;
import com.spartronics4915.frc2025.commands.autos.AlignToReef;

import static com.spartronics4915.frc2025.commands.DynamicsCommandFactory.DynaPreset.*;
import static edu.wpi.first.units.Units.Meters;

import com.spartronics4915.frc2025.Robot;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ArmSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ElevatorSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.IntakeSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.LimelightVisionSubsystem;
import com.spartronics4915.frc2025.util.RumbleFeedbackHandler.RumbleController;
import com.spartronics4915.frc2025.util.RumbleFeedbackHandler.RumblePresets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static com.spartronics4915.frc2025.Constants.BlingConstants.*;

public class DriverCommunication extends BlingSegment {
    private SwerveSubsystem swerve;
    private LimelightVisionSubsystem vision;
    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private DynamicsCommandFactory dynamics;

    private RumbleController[] controllers;
    private double rumbleTime = 0;

    private int alertFrames = 0;

    private BlingSegment current = OFF;

    private static BlingSegment autoSegment = RAINBOW;

    public static enum Region {
        REEF(
            new Translation2d[] {new Translation2d(5, 4), new Translation2d(4, 4), new Translation2d(4.5, 4.5), new Translation2d(4.5, 3.5)}, 
            new Translation2d[] {new Translation2d(13.5, 4), new Translation2d(12.5, 4), new Translation2d(13, 4.5), new Translation2d(13, 4.5)}
        ),
        PROCESSOR(
            new Translation2d[] {new Translation2d(6, .5)}, 
            new Translation2d[] {new Translation2d(11.5, 7.5)}
        ),
        BARGE(
            new Translation2d[] {
                new Translation2d(8.8, 7.2),
                new Translation2d(8.8, 6.1),
                new Translation2d(8.8, 5)
            }, 
            new Translation2d[] {
                new Translation2d(8.8, 3),
                new Translation2d(8.8, 1.9),
                new Translation2d(8.8, .8)
            }
        ),
        CORAL_STATION(
            new Translation2d[] {new Translation2d(.5, 7.5), new Translation2d(.5, .5)}, 
            new Translation2d[] {new Translation2d(17, 7.5), new Translation2d(17, .5)}
        );

        final Translation2d[] redPositions;        
        final Translation2d[] bluePositions;

        private Region(Translation2d[] blue, Translation2d[] red) {
            this.bluePositions = blue;
            this.redPositions = red;
        }
    }

    /**
     * @param length Length of the segment
     * @param subsystems In no particular order, SwerveSubsystem, LimelightVisionSubsystem, IntakeSubsystem, ArmSubsystem, ElevatorSubsystem, DynamicsCommandFactory
     */
    public DriverCommunication(int length, Object... subsystems) {
        this.ledLength = length;
        for (Object subsystem : subsystems) {
            if (subsystem instanceof SwerveSubsystem) this.swerve = (SwerveSubsystem) subsystem;
            if (subsystem instanceof LimelightVisionSubsystem) this.vision = (LimelightVisionSubsystem) subsystem;
            if (subsystem instanceof IntakeSubsystem) this.intake = (IntakeSubsystem) subsystem;
            if (subsystem instanceof ArmSubsystem) this.arm = (ArmSubsystem) subsystem;
            if (subsystem instanceof ElevatorSubsystem) this.elevator = (ElevatorSubsystem) subsystem;
            if (subsystem instanceof DynamicsCommandFactory) this.dynamics = (DynamicsCommandFactory) subsystem;
        }
        dynamics.hasScoredTrigger.onTrue(Commands.runOnce(() -> {
            rumbleTime = 10;
            rumble(RumblePresets.SOFT);
        }));
    }

    public void setRumbleControllers(RumbleController... controllers) {
        this.controllers = controllers;
    }

    public static Region getClosestRegion(SwerveSubsystem swerve) {
        Region closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (Region reg : Region.values()) {
            for (Translation2d position : (DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get().equals(Alliance.Blue) ? reg.bluePositions : reg.redPositions) : reg.redPositions)) {
                if (swerve.getPose().getTranslation().getDistance(position) < closestDistance) {
                    closest = reg;
                    closestDistance = swerve.getPose().getTranslation().getDistance(position);
                }
            }
        }
        return closest;
    }

    @Override
    protected void updateLights() {
        if (!Robot.AUTO_TIMER.hasElapsed(0.01) && vision != null) { // Match has started
            current = vision.isInitialPoseSet() ? SHOW_SPARTRONICS47 : WARN; 
        } else if (DriverStation.isAutonomous()) {
            current = autoSegment;
        } else if (vision != null && vision.newMegaTag1Reading()) {
            current = CYAN;
            alertFrames = 10;
        } else if (alertFrames > 0 && alertFrames % 2 == 0) {
            current = CYAN;
            rumble(RumblePresets.LEFT_WEAK);
        }
        else {
            Region closest = getClosestRegion(this.swerve);
            double elevHeight = elevator.getPosition();
            Rotation2d armRotation = arm.getPosition();
            switch (closest) {
                case PROCESSOR: // Extension of Reef zone
                case REEF:
                    Pose2d closestAprilTag = AlignToReef.getClosestReefAprilTag(swerve.getPose());
                    int index = AlignToReef.allReefTagPoses.indexOf(closestAprilTag);
                    switch(index) {
                        case 1:
                        case 7:
                            current = WHITE;
                            break;
                        case 0:
                        case 2:
                        case 8:
                        case 6:
                            current = ORANGE;
                            break;
                        case 3:
                        case 5:
                        case 9:
                        case 11:
                            current = GREEN;
                            break;
                        case 4:
                        case 10:
                            current = PURPLE;
                            break;
                        default:
                            current = OFF;
                            break;
                    }

                    break;
                case CORAL_STATION:
                    boolean subsystemsInCorrectSpot = Math.abs(armRotation.minus(LOAD.getArmAngle()).getDegrees()) < ARM_THRESHOLD // If arm in correct spot
                                    && Math.abs(elevHeight - LOAD.getElevatorHeight()) < ELEVATOR_THRESHOLD; // And elevator in correct spot
                    boolean robotInRightSpot = true; // TODO: Figure out robot in right spot

                    if (dynamics.funnelDetect()) {
                        current = RAINBOW;
                        rumble(RumblePresets.STRONG);
                        rumbleTime = 5;
                    } else {
                        if (subsystemsInCorrectSpot && robotInRightSpot) {
                            current = GOOD;
                        }
                        else if (subsystemsInCorrectSpot) current = PURPLE; // Robot needs to move
                        else if (robotInRightSpot) current = WARN; // Mechanisms need to move
                        else current = OFF;
                    }

                    break;
                case BARGE:
                    if (DriverStation.getLocation().isEmpty()) {
                        current = OFF;
                        break;
                    }
                    int location = DriverStation.getLocation().getAsInt() - 1;
                    double distance;
                    boolean isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);

                    if (isBlue) distance = OrientTowardsNearestPOIConstants.BARGE_BLUE_CAGE_POSITIONS[location].minus(swerve.getPose().getTranslation()).getMeasureY().in(Meters);
                    else  distance = OrientTowardsNearestPOIConstants.BARGE_RED_CAGE_POSITIONS[location].minus(swerve.getPose().getTranslation()).getMeasureY().in(Meters);
                
                    // If showing red, too far right, go further left.
                    // If showing blue, too far left, go further right.

                    if (distance <= -BARGE_ALIGNMMENT_THRESHOLD) current = isBlue ? BLUE : RED;
                    else if (distance >= BARGE_ALIGNMMENT_THRESHOLD) current = isBlue ? RED : BLUE;
                    else current = RAINBOW; // It's climbin' time.

                    break;
                default:
                    current = OFF;
            }
        }

        current.incrementFrame(FRAME_WAIT);
        current.buffer = this.buffer;
        current.updateLights();

        rumbleTime--;
        alertFrames--;
        if (rumbleTime <= 0) rumble(RumblePresets.OFF);
    }

    private void rumble(RumblePresets feedback) {
        for (RumbleController controller : controllers) {
            controller.setFeedback(feedback.rumble);
        }
    }

    public static Command setAutoSegmentCommand(BlingSegment segment) {
        return Commands.runOnce(() -> autoSegment = segment);
    }

}
