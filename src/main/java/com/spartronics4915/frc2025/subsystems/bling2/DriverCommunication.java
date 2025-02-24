package com.spartronics4915.frc2025.subsystems.bling2;

import com.spartronics4915.frc2025.Constants.BlingConstants;
import com.spartronics4915.frc2025.Robot;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ArmSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ElevatorSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.IntakeSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.LimelightVisionSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverCommunication extends BlingSegment {
    private SwerveSubsystem swerve;
    private LimelightVisionSubsystem vision;
    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    
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

    public DriverCommunication(int length, SubsystemBase... subsystems) {
        this.ledLength = length;
        for (SubsystemBase subsystem : subsystems) {
            if (subsystem instanceof SwerveSubsystem) this.swerve = (SwerveSubsystem) subsystem;
            if (subsystem instanceof LimelightVisionSubsystem) this.vision = (LimelightVisionSubsystem) subsystem;
            if (subsystem instanceof IntakeSubsystem) this.intake = (IntakeSubsystem) subsystem;
            if (subsystem instanceof ArmSubsystem) this.arm = (ArmSubsystem) subsystem;
            if (subsystem instanceof ElevatorSubsystem) this.elevator = (ElevatorSubsystem) subsystem;
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
            switch (closest) {
                case PROCESSOR: // Extension of Reef zone
                case REEF:
                    /* Couple Ideas:
                        Coral Level Indication
                            Blue for l4
                            Green for l3
                            Yellow for l2
                            I don’t think we are doing l1?
                        Red if not ready to score?
                        Can see limelight (Red/Green)
                        Ready to score (red/green)
                     */

                    current = BlingConstants.GOOD;
                    break;
                case CORAL_STATION:
                    // Purple when ready to move on
                    // Green when waiting for coral
                    // Yellow when waiting for robot to move
                    // Red if in wrong position
                case BARGE:
                    // Fun lights
                default:
                    current = BlingConstants.OFF;
            }
        }
        current.incrementFrame();
        current.buffer = this.buffer;
        current.updateLights();
    }

}
