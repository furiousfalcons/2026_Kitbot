
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
//     int t = 0;
//     PhotonCamera frontCamera = new PhotonCamera(VisionConstants.USB_CAMERA1_NAME); // Declare the name of the camera
//                                                                                    // used in the pipeline
//     PhotonCamera backCamera = new PhotonCamera(VisionConstants.USB_CAMERA2_NAME);

//     public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(-0.318, -0.14, 0.356), new Rotation3d(0,-14, 180));   // Set position of camera relative to robot, meters and radians

//     public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

//     PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

//     PhotonPipelineResult resultBack = backCamera.getLatestResult();
//     PhotonPipelineResult resultFront = frontCamera.getLatestResult();
//     Optional<EstimatedRobotPose> visionEst;

//     @Override
//     public void periodic() {
//         // System.out.println(kTagLayout);
//         // System.out.println(AprilTagFields.kDefaultField);
//         try {
//             t += 1;
//             PhotonPipelineResult resultFront = frontCamera.getLatestResult();
//             resultBack = backCamera.getLatestResult();
//              SmartDashboard.putNumber("chimpazini bananini got u", resultFront.getTimestampSeconds());
//              SmartDashboard.putNumber("time: ", t);
//             visionEst = photonEstimator.estimateCoprocMultiTagPose(resultBack);

//             if (visionEst.isEmpty() || true) {
//                 visionEst = photonEstimator.estimateLowestAmbiguityPose(resultBack);

//                 // System.out.println(visionEst.get().estimatedPose);

//             }

//         }

//         catch (Exception E) {
//             System.out.println("no apriltags");
//         }
//     }

//     /*
//      * public Pair<Pose3d, Double> getVisionMeasurement(){
//      * 
//      * Pair<Pose3d, Double> result = new Pair(visionEst.get().estimatedPose,
//      * visionEst.get().timestampSeconds);
//      * return result;
//      * }
//      */

//     public Pose2d getAutoPose(){
//         if (visionEst.isEmpty()){
//             return new Pose2d();

//         }
//         return visionEst.get().estimatedPose.toPose2d();

//     }

//     public double getAngleToAlign(){

//         if (resultFront.hasTargets()){
//             return resultFront.getBestTarget().getYaw();
//         }
//         else return 0;
//     }


    

}