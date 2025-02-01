package com.spartronics4915.frc2025.subsystems.bling2;

import java.util.function.Supplier;

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
    private static SwerveSubsystem swerve;
    private static VisionDeviceSubystem vision;
    
    public enum Region {
        REEF(
            new Translation2d[] {new Translation2d(5, 4), new Translation2d(4, 4), new Translation2d(4.5, 4.5), new Translation2d(4.5, 3.5)}, 
            new Translation2d[] {new Translation2d(13.5, 4), new Translation2d(12.5, 4), new Translation2d(13, 4.5), new Translation2d(13, 4.5)}, 
            () -> {
                if (RobotBase.isSimulation())
                    return LEDPattern.solid(Color.kYellow);
                if (((LimelightVisionSubsystem) vision).canSeeTags())
                    return LEDPattern.solid(Color.kGreen);
                else
                    return LEDPattern.solid(Color.kRed);
            }
        ),
        PROCESSOR(
            new Translation2d[] {new Translation2d(6, .5)}, 
            new Translation2d[] {new Translation2d(11.5, 7.5)}, 
            null
        ),
        BARGE(
            new Translation2d[] {new Translation2d(9, 6)}, 
            new Translation2d[] {new Translation2d(9, 2)}, 
            null
        ),
        CORAL_STATION(
            new Translation2d[] {new Translation2d(.5, 7.5), new Translation2d(.5, .5)}, 
            new Translation2d[] {new Translation2d(17, 7.5), new Translation2d(17, .5)}, 
        null);

        final Translation2d[] redPositions;        
        final Translation2d[] bluePositions;
        final Supplier<LEDPattern> pattern;

        private Region(Translation2d[] blue, Translation2d[] red, Supplier<LEDPattern> pattern) {
            this.bluePositions = blue;
            this.redPositions = red;
            this.pattern = pattern;
        }
    }

    public DriverCommunication(int length, SwerveSubsystem swerve, VisionDeviceSubystem vision) {
        this.ledLength = length;
        this.swerve = swerve;
        this.vision = vision;
    }

    @Override
    protected void updateLights() {
        Region closest = null;
        double closestDistance = Double.MAX_VALUE;

        for (Region reg : Region.values()) {
            for (Translation2d position : (DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get().equals(Alliance.Blue) ? reg.bluePositions : reg.redPositions) : reg.redPositions)) {
                if (this.swerve.getPose().getTranslation().getDistance(position) < closestDistance) {
                    closest = reg;
                    closestDistance = this.swerve.getPose().getTranslation().getDistance(position);
                }
            }
        }
    
        if (closest.pattern != null)
            closest.pattern.get().applyTo(buffer);
        else 
            LEDPattern.kOff.applyTo(buffer);

        //System.out.println((DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get().name() : "No Alliance") + ", " + closest);
    }

}
