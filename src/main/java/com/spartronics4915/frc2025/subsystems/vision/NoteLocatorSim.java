package com.spartronics4915.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
// import java.util.Random;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

public class NoteLocatorSim implements TargetDetectorInterface {

    // private final Random random = new Random();

    SwerveSubsystem swerveDrive;
    final static ArrayList<Translation2d> noteLocations = new ArrayList<>(List.of(new Translation2d(2.9, 7),
            new Translation2d(2.9, 5.5), new Translation2d(2.9, 4.1)));

    public NoteLocatorSim(SwerveSubsystem swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    public Optional<Detection> getClosestVisibleTarget() {

        double minDist = 1e6;
        Pose2d currPose = swerveDrive.getPose();
        Translation2d currPosition = currPose.getTranslation();

        final double MAX_DEGREES = 27;
        final double ROBOT_HEIGHT = 0.3; // Meters
        // final double VERT_TOP_VISIBILITY_THRESH = -20; // Degrees
        final double VERT_BOT_VISIBILITY_THRESH = -40; // Degrees
        Optional<Detection> bestNote = Optional.empty();

        for (Translation2d currNoteLoc : noteLocations) {
            Translation2d botNoteVec = currNoteLoc.minus(currPosition);
            Rotation2d botNoteAngle = botNoteVec.getAngle();
            Rotation2d viewCenterNoteAngle = currPose.getRotation().minus(botNoteAngle);
            Rotation2d vertAngle = new Rotation2d(botNoteVec.getNorm(), -ROBOT_HEIGHT);
            double dist = botNoteVec.getNorm();

            if (Math.abs(viewCenterNoteAngle.getDegrees()) > MAX_DEGREES) {
                continue;
            }

            double vertAngleDegrees = vertAngle.getDegrees();
            if ((vertAngleDegrees < VERT_BOT_VISIBILITY_THRESH)) {
                continue;
            }

            if (dist < minDist) {

                bestNote = Optional.of(new Detection(viewCenterNoteAngle.getDegrees(), vertAngle.getDegrees(), dist));
                minDist = dist;
            }
            // System.out.println("view center: " + viewCenterNoteAngle.getDegrees() + " " +
            // "vert: "
            // + vertAngle.getDegrees() + " " + currPosition + " " + currNoteLoc);

        }

        // if (bestNote.isPresent()) {
        //     double prob = random.nextDouble();

        //     if (prob < 0.25) {
        //         System.out.println("Dropped out");
        //         bestNote = Optional.empty();
        //     }
        // }

        return bestNote;
    }

    // Assuming that we don't care about Ty, just return a constant value.

    public double getTy() {
        return 10;

    }

    // Finds the angle from the center of the closest note. Will filter over 27
    // degrees
    // TODO: This still needs to be completed.

    public OptionalDouble getTx() {
        // final double MAX_ANGLE_DEGREES = 27;

        return OptionalDouble.of(10);

    }

}
