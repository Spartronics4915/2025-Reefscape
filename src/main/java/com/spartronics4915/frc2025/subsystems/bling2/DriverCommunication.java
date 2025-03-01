package com.spartronics4915.frc2025.subsystems.bling2;

import com.spartronics4915.frc2025.Constants.BlingConstants;
import com.spartronics4915.frc2025.commands.DynamicsCommandFactory;
import com.spartronics4915.frc2025.commands.DynamicsCommandFactory.DynaPreset;
import com.spartronics4915.frc2025.commands.autos.AlignToReef;

import static com.spartronics4915.frc2025.commands.DynamicsCommandFactory.DynaPreset.*;

import static edu.wpi.first.units.Units.Meters;

import com.spartronics4915.frc2025.Robot;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ArmSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ElevatorSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.IntakeSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.LimelightVisionSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriverCommunication extends BlingSegment {
    private SwerveSubsystem swerve;
    private LimelightVisionSubsystem vision;
    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private DynamicsCommandFactory dynamics;

    private BlingSegment current = BlingConstants.OFF;

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

    public DriverCommunication(int length, Object... subsystems) {
        this.ledLength = length;
        for (Object subsystem : subsystems) {
            if (subsystem instanceof SwerveSubsystem) this.swerve = (SwerveSubsystem) subsystem;
            if (subsystem instanceof LimelightVisionSubsystem) this.vision = (LimelightVisionSubsystem) subsystem;
            if (subsystem instanceof IntakeSubsystem) this.intake = (IntakeSubsystem) subsystem;
            if (subsystem instanceof ArmSubsystem) this.arm = (ArmSubsystem) subsystem;
            if (subsystem instanceof ElevatorSubsystem) this.elevator = (ElevatorSubsystem) subsystem;
            if (subsystem instanceof LimelightVisionSubsystem) this.vision = (LimelightVisionSubsystem) subsystem;
            if (subsystem instanceof DynamicsCommandFactory) this.dynamics = (DynamicsCommandFactory) subsystem;
        }
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
            current = vision.isInitialPoseSet() ? BlingConstants.SHOW_SPARTRONICS42 : BlingConstants.WARN;
        } else {
            Region closest = getClosestRegion(this.swerve);
            double elevHeight = elevator.getPosition();
            Rotation2d armRotation = arm.getPosition();
            switch (closest) {
                case PROCESSOR: // Extension of Reef zone
                case REEF:
                    Pose2d closestAprilTag = AlignToReef.getClosestReefAprilTag(swerve.getPose());
                    int index = AlignToReef.allReefTagPoses.indexOf(closestAprilTag);
                    System.out.println(index);
                    switch(index) {
                        case 1:
                        case 7:
                            current = BlingConstants.RED;
                            break;
                        case 0:
                        case 2:
                        case 8:
                        case 6:
                            current = BlingConstants.GREEN;
                            break;
                        case 3:
                        case 5:
                        case 9:
                        case 11:
                            current = BlingConstants.BLUE;
                            break;
                        case 4:
                        case 10:
                            current = BlingConstants.PURPLE;
                            break;
                        default:
                            current = BlingConstants.OFF;
                            break;
                    }

                    break;
                case CORAL_STATION:
                    boolean subsystemsInCorrectSpot = Math.abs(armRotation.minus(LOAD.getArmAngle()).getDegrees()) < BlingConstants.ARM_THRESHOLD // If arm in correct spot
                                    && Math.abs(elevHeight - LOAD.getElevatorHeight()) < BlingConstants.ELEVATOR_THRESHOLD; // And elevator in correct spot
                    boolean robotInRightSpot = true; // TODO: Figure out robot in right spot

                    if (dynamics.funnelDetect()) current = BlingConstants.RAINBOW;
                    else {
                        if (subsystemsInCorrectSpot && robotInRightSpot) current = BlingConstants.GOOD;
                        else if (subsystemsInCorrectSpot) current = BlingConstants.PURPLE; // Robot needs to move
                        else if (robotInRightSpot) current = BlingConstants.WARN; // Mechanisms need to move
                        else current = BlingConstants.OFF;
                    }

                    break;
                case BARGE:
                    current = BlingConstants.RAINBOW;
                    break;
                default:
                    current = BlingConstants.OFF;
            }
        }
        current.incrementFrame(BlingConstants.FRAME_WAIT);
        current.buffer = this.buffer;
        current.updateLights();
    }

}
