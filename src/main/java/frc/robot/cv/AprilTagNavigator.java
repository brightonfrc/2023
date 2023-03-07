package frc.robot.cv;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public class AprilTagNavigator {
    PhotonCamera camera;
    PhotonPoseEstimator poseEstimator;
    
    public AprilTagNavigator(PhotonCamera camera) throws IOException { // TODO: Test
        // Init camera
        this.camera = camera;

        // Initialise pose estimator
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, Constants.Measurements.k_robotToCam);
        // TODO: Add reference pose
    }

    // /**
    //  * Return the ID of the AprilTag seen, else return 0
    //  * @return ID of the AprilTag seen, if none seen 0
    //  */
    // public int getAprilTagID() {
    //     PhotonPipelineResult result = camera.getLatestResult();
    //     if(!result.hasTargets()) {
    //         return 0;
    //     }
    //     PhotonTrackedTarget target = result.getBestTarget();
    //     return target.getFiducialId();
    // }

    /**
     * Return the Transform to the AprilTag seen, else return null
     * @return Transform to the AprilTag seen, if none seen null
     */
    public PhotonTrackedTarget getAprilTag() {
        PhotonPipelineResult result = camera.getLatestResult();
        if(!result.hasTargets()) {
            return null;
        }
        return result.getBestTarget();
    }

    public Optional<EstimatedRobotPose> getRobotPose() {
        return poseEstimator.update();
    }
}
