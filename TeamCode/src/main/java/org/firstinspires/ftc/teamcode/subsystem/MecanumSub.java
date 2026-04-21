package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.config.MatchConfig;

public class MecanumSub extends SubsystemBase {

    public final DcMotor frontLeft, frontRight, backLeft, backRight;

    private double fwdPower, strPower, rotPower;

    public final IMU imu;
    public boolean isFieldCentric = Constants.MecanumConstants.isFieldCentric;
    private final double strafeCorrection = Constants.MecanumConstants.strafeCorrection;

    private TelemetryManager panelsTelemetry;
    private Telemetry telemetry;

    private int flPos = 0;
    private int frPos = 0;
    private int blPos = 0;
    private int brPos = 0;

    private double poseXInches = 0.0;
    private double poseYInches = 0.0;
    private double lastHeadingRad = 0.0;

    public MecanumSub(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        frontLeft  = hardwareMap.get(DcMotor.class, Constants.HardWareConstants.MecanumHardWareConstants.frontLeftMotor);
        frontRight = hardwareMap.get(DcMotor.class, Constants.HardWareConstants.MecanumHardWareConstants.frontRightMotor);
        backLeft   = hardwareMap.get(DcMotor.class, Constants.HardWareConstants.MecanumHardWareConstants.backLeftMotor);
        backRight  = hardwareMap.get(DcMotor.class, Constants.HardWareConstants.MecanumHardWareConstants.backRightMotor);

        imu = hardwareMap.get(IMU.class, Constants.HardWareConstants.MecanumHardWareConstants.IMU);

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, // TODO
                        RevHubOrientationOnRobot.UsbFacingDirection.UP      // TODO
                )
        );

        imu.initialize(parameters);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        resetOdometry();

    }

    // Panels only, no Driver Station mirror
    public MecanumSub(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public void setFieldCentric(boolean fieldCentric) {
        isFieldCentric = fieldCentric;
    }

    public void resetOdometry() {
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        lastHeadingRad = -getRobotYawRadians();

        poseXInches = 0.0;
        poseYInches = 0.0;
    }

    public void updateOdometry() {
        int fl = frontLeft.getCurrentPosition();
        int fr = frontRight.getCurrentPosition();
        int bl = backLeft.getCurrentPosition();
        int br = backRight.getCurrentPosition();

        int df = fl - flPos;   // forward (FL, BR)
        int dr = fr - frPos;   // rotate (FR, BL)

        flPos = fl;
        frPos = fr;
        blPos = bl;
        brPos = br;

        double ticksPerInch = Constants.MecanumConstants.ticksPerInch; // TUNE THIS to your robot (ticks per inch)
        double dForward = (df + br) / 2.0 / ticksPerInch;   // average forward wheel motion
        double dRotation = (dr - bl) / 2.0 / ticksPerInch;  // rough turn

        double headingRad = -getRobotYawRadians();
        double dTheta = headingRad - lastHeadingRad;

        double worldDX = 0.0;
        double worldDY = 0.0;

        if (Math.abs(dTheta) < 1e-5) {
            worldDX = dForward * Math.cos(headingRad);
            worldDY = dForward * Math.sin(headingRad);
        } else {
            double radius = dForward / dTheta;
            double centerX = -radius * Math.sin(headingRad);
            double centerY =  radius * Math.cos(headingRad);
            worldDX = centerX - centerX * Math.cos(dTheta) + centerY * Math.sin(dTheta);
            worldDY = centerY - centerY * Math.cos(dTheta) - centerX * Math.sin(dTheta);
        }

        poseXInches += worldDX;
        poseYInches += worldDY;
        lastHeadingRad = headingRad;
    }

    public double getRobotYawRadians(){
        if(MatchConfig.alliance == MatchConfig.AllianceColor.RED){
            return Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)  + 3.14159);
        } else {
            return Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }
    }

    public double getPoseXMecanumIn(){
        return poseXInches;
    }
    public double getPoseYMecanumIn(){
        return poseYInches;
    }

    public void drive(double forward, double strafe, double rotation) {
        forward  = applyDeadzone(forward,  Constants.MecanumConstants.thresHold);
        strafe   = applyDeadzone(strafe,   Constants.MecanumConstants.thresHold);
        rotation = applyDeadzone(rotation, Constants.MecanumConstants.thresHold);

        fwdPower = forward;
        strPower = strafe;
        rotPower = rotation;

        if (isFieldCentric) {

            double headingRad = -getRobotYawRadians();

            double tempForward = forward * Math.cos(headingRad) - strafe * Math.sin(headingRad);
            double tempStrafe  = forward * Math.sin(headingRad) + strafe * Math.cos(headingRad);

            forward = tempForward;
            strafe  = tempStrafe;
        }

        strafe *= strafeCorrection;

        double fl = forward + strafe + rotation;
        double fr = forward - strafe + rotation;
        double bl = forward - strafe - rotation;
        double br = forward + strafe - rotation;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;  fr /= max;  bl /= max;  br /= max;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private double applyDeadzone(double value, double threshold) {
        return Math.abs(value) > threshold ? value : 0;
    }

    public DcMotor getFl() { return frontLeft; }
    public DcMotor getFr() { return frontRight; }
    public DcMotor getRl() { return backLeft; }
    public DcMotor getRr() { return backRight; }

    public double getFwdPower()  { return fwdPower; }
    public double getStrPower()  { return strPower; }
    public double getRotPower()  { return rotPower; }

    public void setPoseInches(double xInches, double yInches) {
        poseXInches = xInches;
        poseYInches = yInches;
    }

    @Override
    public void periodic() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        panelsTelemetry.debug(
                "=== Drive Inputs ===",
                "Forward:  " + fwdPower,
                "Strafe:   " + strPower,
                "Rotation: " + rotPower,

                "=== Motor Powers ===",
                "FL: " + frontLeft.getPower(),
                "FR: " + frontRight.getPower(),
                "BL: " + backLeft.getPower(),
                "BR: " + backRight.getPower(),

                "=== IMU ===",
                "Yaw:   " + orientation.getYaw(AngleUnit.DEGREES)   + " deg",
                "Pitch: " + orientation.getPitch(AngleUnit.DEGREES) + " deg",
                "Roll:  " + orientation.getRoll(AngleUnit.DEGREES)  + " deg",

                "=== Mode ===",
                "Field Centric: " + isFieldCentric
        );

        if (telemetry != null) {
            panelsTelemetry.update(telemetry);
        } else {
            panelsTelemetry.update();
        }
    }
}