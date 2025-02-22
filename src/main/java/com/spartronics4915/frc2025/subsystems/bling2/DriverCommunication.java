package com.spartronics4915.frc2025.subsystems.bling2;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import java.util.function.Supplier;

import com.spartronics4915.frc2025.Constants.BlingConstants;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.LimelightVisionSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.VisionDeviceSubystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public class DriverCommunication extends BlingSegment {
    private SwerveSubsystem swerve;
    private VisionDeviceSubystem vision;
    
    private BlingSegment current;

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

    public DriverCommunication(int length, SwerveSubsystem swerve, VisionDeviceSubystem vision) {
        this.ledLength = length;
        this.swerve = swerve;
        this.vision = vision;
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
        Region closest = getClosestRegion(this.swerve);
        switch (closest) {
            case REEF:
                current = BlingConstants.GOOD;
                break;
            case CORAL_STATION:
            case BARGE:
            case PROCESSOR:
            default:
                current = BlingConstants.OFF;
        }
        System.out.println(closest);
        current.incrementFrame();
        current.buffer = this.buffer;
        current.updateLights();
    }

}
