package com.spartronics4915.frc2025.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import com.spartronics4915.frc2025.RobotContainer;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.IntakeSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoralSim {
    

    private static boolean funnelLCstate = false;
    private static boolean intakeLCstate = true;
    private static double timeUntilIntake = 0.0;
    private static Timer funnelToIntakeTimer = new Timer();


    public static boolean getFunnelLC(){
        return funnelLCstate;
    }

    public static boolean getIntakeLC(){
        return intakeLCstate;
    }

    public static void setup(SwerveSubsystem swerve, IntakeSubsystem intake){

        final ArrayList<Pose2d> stationPoses = new ArrayList<>();
        var field = RobotContainer.getFieldLayout();

        stationPoses.addAll(Arrays.stream(
            AprilTagRegion.kStation.both()
        ).mapToObj((i) -> {
            return field.getTagPose(i).get().toPose2d();
        }).toList());

        var withinZoneTrigger = new Trigger(() -> {
            Pose2d pose = swerve.getPose();

            var nearest = pose.nearest(stationPoses);

            return pose.getTranslation().getDistance(nearest.getTranslation()) < 1.0;
        }).debounce(0.3);

        var isIntaking = new Trigger(() -> {
            return intake.setpoint < -0.1;
        }).debounce(0.0);

        var isOuttaking = new Trigger(() -> {
            return intake.setpoint > 0.1;
        }).debounce(0.0);

        isOuttaking.debounce(0.0).onTrue(Commands.runOnce(() -> {
            intakeLCstate = false;
            System.out.println("outtaking");
        }));

        withinZoneTrigger.debounce(0.3).onTrue(Commands.runOnce(() -> {
            System.out.println("inZone, funnel = true");
            funnelLCstate = true;
        }));



        isIntaking.and(() -> getFunnelLC()).onTrue(Commands.runOnce(() -> {
            System.out.println("intaking");
            timeUntilIntake = new Random().nextGaussian(0.2, 0.2);
            funnelToIntakeTimer.restart();
        }));

        new Trigger(() -> {
            return funnelToIntakeTimer.hasElapsed(timeUntilIntake);
        }).and(() -> getFunnelLC()).and(isIntaking).onTrue(Commands.runOnce(() -> {
            System.out.println("intaked");
            funnelLCstate = false;
            intakeLCstate = true;
        }));

    }

}
