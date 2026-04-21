package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystem.MecanumSub;

public class PoseManager {
    Limelight3A limelight;
    MecanumSub mecanumSub;

    //These are the drive localization variables, ODOMETRY PODS and mecanum wheel encoders
    private double poseXInches;
    private double poseYInches;
    private double headingDeg;
    public PoseManager(MecanumSub mecanumSub, HardwareMap hardwareMap){
        this.mecanumSub = mecanumSub;

        limelight = hardwareMap.get(limelight.getClass(), Constants.HardWareConstants.OdometryHardwareConstants.Limelight3a);

        headingDeg = mecanumSub.getRobotYawRadians();
        poseXInches = mecanumSub.getPoseXMecanumIn();
        poseYInches = mecanumSub.getPoseYMecanumIn();

        limelight.pipelineSwitch(Constants.PoseManagerConstants.limelightPipeline);
        limelight.setPollRateHz(Constants.PoseManagerConstants.limlightPollRateHZ);
        limelight.start();
    }
    public void update() {
        poseXInches = mecanumSub.getPoseXMecanumIn();
        poseYInches = mecanumSub.getPoseYMecanumIn();
        headingDeg = Math.toDegrees(mecanumSub.getRobotYawRadians());

        LimelightHelpers.FusionResult fused =
                LimelightHelpers.fuseWithOdometryInches(
                        limelight,
                        poseXInches,
                        poseYInches,
                        headingDeg,
                        Constants.PoseManagerConstants.maxStalenessMs,
                        Constants.PoseManagerConstants.minTagCount,
                        Constants.PoseManagerConstants.maxSingleTagDistanceMeters,
                        Constants.PoseManagerConstants.maxVisionOdomErrorInches,
                        Constants.PoseManagerConstants.visionWeightXY,
                        Constants.PoseManagerConstants.visionWeightHeading
                );

        if (fused.visionAccepted) {
            poseXInches = fused.fusedPose.x;
            poseYInches = fused.fusedPose.y;
            headingDeg = fused.fusedPose.headingDeg;

            mecanumSub.setPoseInches(poseXInches, poseYInches);
        }
    }

    public double getXInches() {
        return poseXInches;
    }

    public double getYInches() {
        return poseYInches;
    }

    public double getHeadingDeg() {
        return headingDeg;
    }

    public void stopLimelight() {
        limelight.stop();
    }

}
