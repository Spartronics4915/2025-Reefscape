package com.spartronics4915.frc2025.subsystems.vision;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.spartronics4915.frc2025.RobotContainer;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class SimVisionSubsystem extends SubsystemBase implements VisionDeviceSubystem {

    private final SwerveDrive swerveDrive;
    private final VisionSystemSim visionSim;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private Optional<EstimatedRobotPose> reefPoseEst;
    private final PhotonPoseEstimator reefCameraEstimator;

    private ArrayList<Integer> fiducialIds;

    public SimVisionSubsystem(SwerveSubsystem swerveSubsystem) {
        swerveDrive = swerveSubsystem.getInternalSwerve();
        visionSim = new VisionSystemSim("main");

        AprilTagFieldLayout tagLayout = RobotContainer.getFieldLayout();
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
        Translation3d robotToCameraTrl = new Translation3d(-0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        reefCameraEstimator = photonPoseEstimator;

        // Add this camera to the vision system simulation with the given
        // robot-to-camera transform.
        visionSim.addCamera(cameraSim, robotToCamera);
        cameraSim.enableDrawWireframe(false);

        reefPoseEst = Optional.empty();

    }

    public ArrayList<Integer> getVisibleTagIDs() {

        return fiducialIds;
    }


    public Optional<Pose2d> getBotPose2dFromReefCamera() {

        if(reefPoseEst.isEmpty()) {
            return Optional.empty();
        }

        EstimatedRobotPose estimatedPose = reefPoseEst.get();

        Pose2d result = estimatedPose.estimatedPose.toPose2d();

        return Optional.of(result);

    }
       
    @Override
    public void simulationPeriodic() {

        var poseQuery = swerveDrive.getSimulationDriveTrainPose();
        Pose2d simPose = poseQuery.get();

        visionSim.update(simPose);

        var res = camera.getAllUnreadResults();
        fiducialIds = new ArrayList<Integer>();

        if (res.size() > 0) {

            reefPoseEst = Optional.empty();
            for (var change : res) {
                reefPoseEst = reefCameraEstimator.update(change);
            }
            PhotonPipelineResult first_PipelineResult = res.get(0);

            List<PhotonTrackedTarget> targetList = first_PipelineResult.getTargets();
            for (var i : targetList) {
                if (i.area > 0.1) {}
                fiducialIds.add(i.fiducialId);
            }
        }

    }

}
