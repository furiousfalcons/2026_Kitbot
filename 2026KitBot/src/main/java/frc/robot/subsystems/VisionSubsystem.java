
package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionSubsystem extends SubsystemBase {

    PhotonCamera frontCamera = new PhotonCamera(VisionConstants.USB_CAMERA1_NAME); // Declare the name of the camera
                                                                                   // used in the pipeline
    // PhotonCamera backCamera = new PhotonCamera(VisionConstants.USB_CAMERA2_NAME);

    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.3683, 0.0, 0.5144),
            new Rotation3d(0, 0, 0)); // Set position of camera relative to robot, meters and radians

    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

    // PhotonCamera camera = new PhotonCamera(VisionConstants.USB_CAMERA1_NAME);
    PhotonPipelineResult result = frontCamera.getLatestResult();
    Optional<EstimatedRobotPose> visionEst;

    @Override
    public void periodic() {
        // System.out.println(kTagLayout);
        // System.out.println(AprilTagFields.kDefaultField);
        try {
            PhotonPipelineResult result = frontCamera.getLatestResult();
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);

            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);

                // System.out.println(visionEst.get().estimatedPose);

            }

        }

        catch (Exception E) {
            System.out.println("no apriltags");
        }
    }

    /*
     * public Pair<Pose3d, Double> getVisionMeasurement(){
     * 
     * Pair<Pose3d, Double> result = new Pair(visionEst.get().estimatedPose,
     * visionEst.get().timestampSeconds);
     * return result;
     * }
     */

    public Pose2d getAutoPose() {
        if (visionEst != null) {
            return visionEst.get().estimatedPose.toPose2d();
        }
        return null;

    }

}