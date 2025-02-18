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
    private static SwerveSubsystem swerve;
    private static VisionDeviceSubystem vision;
    
    public static enum Region {
        REEF(
            new Translation2d[] {new Translation2d(5, 4), new Translation2d(4, 4), new Translation2d(4.5, 4.5), new Translation2d(4.5, 3.5)}, 
            new Translation2d[] {new Translation2d(13.5, 4), new Translation2d(12.5, 4), new Translation2d(13, 4.5), new Translation2d(13, 4.5)}, 
            () -> {
                if (RobotBase.isSimulation())
                    return LEDPattern.solid(Color.kYellow).atBrightness(Percent.of(BlingConstants.BLING_BRIGHTNESS));
                if (((LimelightVisionSubsystem) vision).canSeeTags())
                    return LEDPattern.solid(Color.kGreen).atBrightness(Percent.of(BlingConstants.BLING_BRIGHTNESS));
                else
                    return LEDPattern.solid(Color.kRed).atBrightness(Percent.of(BlingConstants.BLING_BRIGHTNESS));
            }
        ),
        PROCESSOR(
            new Translation2d[] {new Translation2d(6, .5)}, 
            new Translation2d[] {new Translation2d(11.5, 7.5)}, 
            () -> {
                return LEDPattern.rainbow(255, 255).atBrightness(Percent.of(BlingConstants.BLING_BRIGHTNESS)).scrollAtAbsoluteSpeed(MetersPerSecond.of(40), Meters.of(1));
            }
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
            }, 
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
        if (closest.pattern != null)
            closest.pattern.get().applyTo(buffer);
        else 
            LEDPattern.kOff.applyTo(buffer);

        //System.out.println((DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get().name() : "No Alliance") + ", " + closest);
    }

}
