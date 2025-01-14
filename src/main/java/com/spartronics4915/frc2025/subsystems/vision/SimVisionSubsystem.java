package com.spartronics4915.frc2025.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class SimVisionSubsystem extends SubsystemBase implements VisionSubystem {

    private final SwerveDrive swerveDrive;
    private final VisionSystemSim visionSim;
    private final PhotonCamera camera;

    public SimVisionSubsystem(SwerveSubsystem swerveSubsystem) {
        swerveDrive = swerveSubsystem.getInternalSwerve();
        visionSim = new VisionSystemSim("main");

        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        visionSim.addAprilTags(tagLayout);
        SimCameraProperties cameraProp = new SimCameraProperties();
        // The PhotonCamera used in the real robot code.
        camera = new PhotonCamera("cameraName");

        // The simulation of this camera. Its values used in real robot code will be
        // updated.
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);

        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
        // pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z =
        // 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        // Add this camera to the vision system simulation with the given
        // robot-to-camera transform.
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    public ArrayList<Integer> getVisibleTagIDs() {

        var res = camera.getAllUnreadResults();
        ArrayList<Integer> output = new ArrayList<Integer>();
        if (res.size() > 0) {
            PhotonPipelineResult first_PipelineResult = res.get(0);

            List<PhotonTrackedTarget> targetList = first_PipelineResult.getTargets();
            for (var i : targetList) {
                output.add(i.fiducialId);
            }
        }

        return output;
    }

    @Override
    public void simulationPeriodic() {

        var poseQuery = swerveDrive.getSimulationDriveTrainPose();
        Pose2d simPose = poseQuery.get();

        visionSim.update(simPose);

    }

}
