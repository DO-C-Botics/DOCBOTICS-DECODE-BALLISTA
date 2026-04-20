package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightHelpers {

    public static class PoseEstimate {
        public boolean valid;
        public Pose3D pose3d;
        public double xMeters;
        public double yMeters;
        public double zMeters;
        public double rollDeg;
        public double pitchDeg;
        public double yawDeg;
        public int tagCount;
        public long stalenessMs;
        public LLResult rawResult;
        public List<LLResultTypes.FiducialResult> fiducials;

        public PoseEstimate() {
            valid = false;
        }
    }

    public static class Pose2dEstimate {
        public double x;
        public double y;
        public double headingDeg;

        public Pose2dEstimate() { }

        public Pose2dEstimate(double x, double y, double headingDeg) {
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
        }

        public Pose2D toPose2D(DistanceUnit distanceUnit) {
            return new Pose2D(distanceUnit, x, y, AngleUnit.DEGREES, headingDeg);
        }

        @Override
        public String toString() {
            return String.format("(%.3f, %.3f, %.1f°)", x, y, headingDeg);
        }
    }

    public static class FusionResult {
        public boolean visionAccepted;
        public PoseEstimate visionEstimate;
        public Pose2dEstimate odomPose;
        public Pose2dEstimate fusedPose;
        public double xyError;
        public double headingErrorDeg;
    }

    public static PoseEstimate getBotPoseEstimateMT1(Limelight3A limelight) {
        LLResult result = limelight.getLatestResult();
        return fromResult(result, false);
    }

    public static PoseEstimate getBotPoseEstimateMT2(Limelight3A limelight, double robotYawDeg) {
        limelight.updateRobotOrientation(robotYawDeg);
        LLResult result = limelight.getLatestResult();
        return fromResult(result, true);
    }

    private static PoseEstimate fromResult(LLResult result, boolean useMT2) {
        PoseEstimate estimate = new PoseEstimate();
        estimate.rawResult = result;

        if (result == null || !result.isValid()) {
            return estimate;
        }

        Pose3D pose = useMT2 ? result.getBotpose_MT2() : result.getBotpose();
        if (pose == null) {
            return estimate;
        }

        estimate.valid = true;
        estimate.pose3d = pose;

        estimate.xMeters = pose.getPosition().x;
        estimate.yMeters = pose.getPosition().y;
        estimate.zMeters = pose.getPosition().z;

        estimate.rollDeg = pose.getOrientation().getRoll(AngleUnit.DEGREES);
        estimate.pitchDeg = pose.getOrientation().getPitch(AngleUnit.DEGREES);
        estimate.yawDeg = pose.getOrientation().getYaw(AngleUnit.DEGREES);

        estimate.fiducials = result.getFiducialResults();
        estimate.tagCount = estimate.fiducials != null ? estimate.fiducials.size() : 0;
        estimate.stalenessMs = result.getStaleness();

        return estimate;
    }

    public static boolean isUsable(PoseEstimate estimate, long maxStalenessMs, int minTagCount) {
        return estimate != null
                && estimate.valid
                && estimate.stalenessMs <= maxStalenessMs
                && estimate.tagCount >= minTagCount;
    }

    public static boolean shouldReject(
            PoseEstimate estimate,
            long maxStalenessMs,
            int minTagCount,
            double maxSingleTagDistanceMeters,
            double maxVisionOdomErrorMeters,
            double odomX,
            double odomY
    ) {
        if (!isUsable(estimate, maxStalenessMs, minTagCount)) {
            return true;
        }

        double odomError = Math.hypot(estimate.xMeters - odomX, estimate.yMeters - odomY);
        if (odomError > maxVisionOdomErrorMeters) {
            return true;
        }

        if (estimate.tagCount == 1 && estimate.fiducials != null && !estimate.fiducials.isEmpty()) {
            LLResultTypes.FiducialResult fid = estimate.fiducials.get(0);
            if (fid == null) {
                return true;
            }

            Pose3D cameraPoseTargetSpace = fid.getCameraPoseTargetSpace();
            if (cameraPoseTargetSpace != null) {
                double dx = cameraPoseTargetSpace.getPosition().x;
                double dy = cameraPoseTargetSpace.getPosition().y;
                double dz = cameraPoseTargetSpace.getPosition().z;
                double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

                if (dist > maxSingleTagDistanceMeters) {
                    return true;
                }
            }
        }

        return false;
    }

    public static FusionResult fuseWithOdometryMeters(
            Limelight3A limelight,
            double odomX,
            double odomY,
            double odomHeadingDeg,
            long maxStalenessMs,
            int minTagCount,
            double maxSingleTagDistanceMeters,
            double maxVisionOdomErrorMeters,
            double visionWeightXY,
            double visionWeightHeading
    ) {
        FusionResult out = new FusionResult();
        out.odomPose = new Pose2dEstimate(odomX, odomY, odomHeadingDeg);

        PoseEstimate vision = getBotPoseEstimateMT2(limelight, odomHeadingDeg);
        out.visionEstimate = vision;

        if (shouldReject(
                vision,
                maxStalenessMs,
                minTagCount,
                maxSingleTagDistanceMeters,
                maxVisionOdomErrorMeters,
                odomX,
                odomY
        )) {
            out.visionAccepted = false;
            out.fusedPose = new Pose2dEstimate(odomX, odomY, odomHeadingDeg);
            out.xyError = 0.0;
            out.headingErrorDeg = 0.0;
            return out;
        }

        out.visionAccepted = true;

        double headingError = angleWrapDeg(vision.yawDeg - odomHeadingDeg);
        double fusedX = lerp(odomX, vision.xMeters, clamp01(visionWeightXY));
        double fusedY = lerp(odomY, vision.yMeters, clamp01(visionWeightXY));
        double fusedHeading = angleWrapDeg(
                odomHeadingDeg + clamp01(visionWeightHeading) * headingError
        );

        out.xyError = Math.hypot(vision.xMeters - odomX, vision.yMeters - odomY);
        out.headingErrorDeg = headingError;
        out.fusedPose = new Pose2dEstimate(fusedX, fusedY, fusedHeading);

        return out;
    }

    public static FusionResult fuseWithOdometryInches(
            Limelight3A limelight,
            double odomXInches,
            double odomYInches,
            double odomHeadingDeg,
            long maxStalenessMs,
            int minTagCount,
            double maxSingleTagDistanceMeters,
            double maxVisionOdomErrorInches,
            double visionWeightXY,
            double visionWeightHeading
    ) {
        FusionResult result = fuseWithOdometryMeters(
                limelight,
                inchesToMeters(odomXInches),
                inchesToMeters(odomYInches),
                odomHeadingDeg,
                maxStalenessMs,
                minTagCount,
                maxSingleTagDistanceMeters,
                inchesToMeters(maxVisionOdomErrorInches),
                visionWeightXY,
                visionWeightHeading
        );

        result.odomPose.x = metersToInches(result.odomPose.x);
        result.odomPose.y = metersToInches(result.odomPose.y);
        result.fusedPose.x = metersToInches(result.fusedPose.x);
        result.fusedPose.y = metersToInches(result.fusedPose.y);
        result.xyError = metersToInches(result.xyError);

        return result;
    }

    public static double metersToInches(double meters) {
        return meters * 39.3701;
    }

    public static double inchesToMeters(double inches) {
        return inches / 39.3701;
    }

    public static String poseToString(PoseEstimate estimate) {
        if (estimate == null || !estimate.valid) {
            return "INVALID";
        }
        return String.format("(%.3f m, %.3f m, %.1f°)", estimate.xMeters, estimate.yMeters, estimate.yawDeg);
    }

    public static String fusionToString(FusionResult result) {
        if (result == null) {
            return "NULL";
        }
        return "visionAccepted=" + result.visionAccepted
                + " odom=" + result.odomPose
                + " fused=" + result.fusedPose
                + String.format(" xyError=%.3f headingError=%.1f°", result.xyError, result.headingErrorDeg);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double clamp01(double x) {
        return Math.max(0.0, Math.min(1.0, x));
    }

    private static double angleWrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }
}