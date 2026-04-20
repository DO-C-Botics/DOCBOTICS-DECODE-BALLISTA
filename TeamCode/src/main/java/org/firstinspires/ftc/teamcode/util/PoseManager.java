package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystem.MecanumSub;

public class PoseManager {
    Limelight3A limelight;
    MecanumSub mecanumSub;

    //These are the drive localization variables, ODOMETRY PODS and mecanum wheel encoders
    private double poseXInches;
    private double poseYInches;
    private double headingDeg;
    public PoseManager(MecanumSub mecanumSub){
        this.mecanumSub = mecanumSub;

        headingDeg = mecanumSub.getRobotYawRadians();
        poseXInches = mecanumSub.getPoseXMecanumIn();
        poseYInches = mecanumSub.getPoseYMecanumIn();

        limelight.pipelineSwitch(Constants.PoseManagerConstants.limelightPipeline);
        limelight.setPollRateHz(Constants.PoseManagerConstants.limlightPollRateHZ);
        limelight.start();
    }
}
