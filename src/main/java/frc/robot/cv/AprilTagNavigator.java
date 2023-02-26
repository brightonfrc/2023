package frc.robot.cv;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagNavigator {
    PhotonCamera camera;
    
    public AprilTagNavigator(PhotonCamera camera) {
        this.camera = camera;
    }

    /**
     * Return the ID of the AprilTag seen, else return 0
     * @return ID of the AprilTag seen, if none seen 0
     */
    public int getAprilTagID() {
        PhotonPipelineResult result = camera.getLatestResult();
        if(!result.hasTargets()) {
            return 0;
        }
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getFiducialId();
    }

    /**
     * Return the Transform to the AprilTag seen, else return null
     * @return Transform to the AprilTag seen, if none seen null
     */
    public Transform3d getAprilTagCameraToTarget() {
        PhotonPipelineResult result = camera.getLatestResult();
        if(!result.hasTargets()) {
            return null;
        }
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getBestCameraToTarget();
    }
}
