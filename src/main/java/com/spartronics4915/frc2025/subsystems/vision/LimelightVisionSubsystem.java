package com.spartronics4915.frc2025.subsystems.vision;

import static edu.wpi.first.units.Units.Milliseconds;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.logging.Logger;

import com.spartronics4915.frc2025.Constants.VisionConstants;
import com.spartronics4915.frc2025.Constants.VisionConstants.PoseEstimationMethod;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2025.util.Structures.LimelightConstants;
import com.spartronics4915.frc2025.util.Structures.VisionMeasurement;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVisionSubsystem extends SubsystemBase implements VisionDeviceSubystem, ModeSwitchInterface {
    private final ArrayList<LimelightDevice> limelights;
    private static boolean mt1Override = false;
    private static boolean discardMeasurements = false;

    private LimelightDevice reefLL;
    private LimelightDevice alignLL;
    private LimelightDevice stationLL;

    private final SwerveSubsystem swerveSubsystem;
    private final StructArrayPublisher<Pose3d> visionTargetPublisher;
    private final AprilTagFieldLayout fieldLayout;

    private boolean initalPoseSet = false;

    private long lastMegaTag1Reading;
    private boolean isMegaTag1ReadingNew = false;

    public LimelightVisionSubsystem(SwerveSubsystem swerveSubsystem, AprilTagFieldLayout fieldLayout) {
        limelights = new ArrayList<>();
        for (LimelightConstants config : VisionConstants.kLimelights) {
            LimelightDevice limelight = new LimelightDevice(config);
            limelights.add(limelight);
            boolean diagnosticsNeeded = VisionConstants.kVisionDiagnostics;
            switch (config.role()) {
                case REEF:
                    reefLL = limelight;
                    System.out.println("Setting reef limelight to " + config.id());
                    break;
                case ALIGN:
                    alignLL = limelight;
                    System.out.println("Setting align limelight to " + config.id());
                    break;
                case STATION:
                    stationLL = limelight;
                    System.out.println("Setting station limelight to " + config.id());
                    break;
                default:
                    diagnosticsNeeded = false;
                    System.out.println("Not setting " + config.id() + " to anything");
            }
            if (diagnosticsNeeded) {
                SmartDashboard.putNumber("VisionDiagnostics/limelight-" + config.name() + "/stddev", -1);
                SmartDashboard.putNumber("VisionDiagnostics/limelight-" + config.name() + "/count", -1);
                SmartDashboard.putNumber("VisionDiagnostics/limelight-" + config.name() + "/distance", -1);
                SmartDashboard.putNumber("VisionDiagnostics/limelight-" + config.name() + "/speed", -1);
                SmartDashboard.putString("VisionDiagnostics/limelight-" + config.name() + "/method", "");
                SmartDashboard.putData("VisionDiagnostics/limelight-" + config.name() + "/pose", new Field2d());
            }

            SmartDashboard.putBoolean("Initial Pose Set?", false);
            SmartDashboard.putBoolean("VisionDiagnostics/Want New MT1 Reading?", false);

            lastMegaTag1Reading = System.currentTimeMillis();
        }

        this.fieldLayout = fieldLayout;
        this.swerveSubsystem = swerveSubsystem;

        if (VisionConstants.kVisionDiagnostics) {
            NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
            StructArrayTopic<Pose3d> visionTargetTopic = networkTableInstance.getStructArrayTopic(
                "VisionDiagnostics/vision targets", Pose3d.struct);
            visionTargetPublisher = visionTargetTopic.publish();
            // Shuffleboard.getTab("logging").addString("VisionDiagnostics/vision target ids", () -> this.getVisibleTagIDs().toString());
        } else visionTargetPublisher = null;

        updateTagFilters();
    }

    private void updateTagFilters() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        limelights.forEach(limelight -> {
            limelight.setTagFilter(alliance);
        });
    }

    public ArrayList<VisionMeasurement> getVisionMeasurements() {
        ArrayList<VisionMeasurement> measurements = new ArrayList<>();
        limelights.forEach((limelight) -> {
            Optional<VisionMeasurement> measurement = limelight.getVisionMeasurement(swerveSubsystem);
            if (measurement.isPresent()) {
                measurements.add(measurement.get());
            }
        });
        return measurements;
    }

    public boolean canSeeTags() {
        return reefLL.getTv() || stationLL.getTv();
    }

    public ArrayList<Integer> getVisibleTagIDs() {
        Set<Integer> set = new HashSet<>();
        limelights.forEach((limelight) -> {
            ArrayList<Integer> results = limelight.getVisibleTags();
            set.addAll(results);
        });
        return new ArrayList<Integer>(set);
    }

    public ArrayList<Pose3d> getVisibleTagPoses() {
        ArrayList<Integer> visibleTagIDs = getVisibleTagIDs();
        ArrayList<Pose3d> visibleTagPoses = new ArrayList<>();
        visibleTagIDs.forEach((Integer tagID) -> {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);
            if (tagPose.isEmpty()) {
                Logger.getLogger(this.getClass().getName())
                        .warning("Tag ID " + tagID + " not found in field layout ");
            } else {
                visibleTagPoses.add(tagPose.get());
            }
        });
        return visibleTagPoses;
    }

    @Override
    public void periodic() {
        Long currentTime = System.currentTimeMillis();
        Time sinceLastReading = Milliseconds.of(currentTime - lastMegaTag1Reading);
        boolean wantNewMegaTag1Reading = sinceLastReading.gte(VisionConstants.newMegaTag1ReadingThreshold);
        SmartDashboard.putBoolean("VisionDiagnostics/Want New MT1 Reading?", wantNewMegaTag1Reading);

        getVisionMeasurements().forEach((measurement) -> {
            if (!initalPoseSet) {
                initalPoseSet = true;
                SmartDashboard.putBoolean("Initial Pose Set?", true);
            }
            if (measurement.diagMethod().equals(PoseEstimationMethod.MEGATAG_1)) {
                if (wantNewMegaTag1Reading) isMegaTag1ReadingNew = true;
                lastMegaTag1Reading = currentTime;
            }
            if (!discardMeasurements) swerveSubsystem.addVisionMeasurement(measurement.pose(), measurement.timestamp(), measurement.stdDevs());
            if (VisionConstants.kVisionDiagnostics) {
                SmartDashboard.putNumber("VisionDiagnostics/" + measurement.diagName() + "/stddev", measurement.stdDevs().get(0, 0));
                SmartDashboard.putNumber("VisionDiagnostics/" + measurement.diagName() + "/count", measurement.diagTagCount());
                SmartDashboard.putNumber("VisionDiagnostics/" + measurement.diagName() + "/distance", measurement.diagTagDistance());
                SmartDashboard.putNumber("VisionDiagnostics/" + measurement.diagName() + "/speed", measurement.diagRobotSpeed());
                SmartDashboard.putString("VisionDiagnostics/" + measurement.diagName() + "/method", measurement.diagMethod().toString());
                ((Field2d) SmartDashboard.getData("VisionDiagnostics/" + measurement.diagName() + "/pose")).setRobotPose(measurement.pose());
            }
        });

        if (VisionConstants.kVisionDiagnostics) visionTargetPublisher.set(getVisibleTagPoses().toArray(new Pose3d[0]));
    }

    @Override
    public void onModeSwitch() {
        updateTagFilters();
    }

    public static boolean getMegaTag1Override() {
        return mt1Override;
    }

    public static void setMegaTag1Override(boolean b) {
        mt1Override = b;
    }

    public static void setDiscardMeasurements(boolean b) {
        discardMeasurements = b;
    }

    public boolean isInitialPoseSet() {
        return initalPoseSet;
    }

    public boolean newMegaTag1Reading() {
        if (isMegaTag1ReadingNew) {
            isMegaTag1ReadingNew = false;
            return true;
        }
        return false;
    }

    public Optional<Pose2d> getBotPose2dFromReefCamera() {
        return reefLL.getPose2d();
    }

    public LimelightDevice getReefLimelight() {
        return reefLL;
    }

    public LimelightDevice getAlignLimelight() {
        return alignLL;
    }

    public LimelightDevice getStationLimelight() {
        return stationLL;
    }

}
