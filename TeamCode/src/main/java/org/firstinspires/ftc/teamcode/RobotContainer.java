package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.MecanumDriveCMD;
import org.firstinspires.ftc.teamcode.config.MatchConfig;
import org.firstinspires.ftc.teamcode.subsystem.MecanumSub;

@TeleOp(name = "TeleOpMode")
public class RobotContainer extends CommandOpMode {
    private MecanumSub mecanumSub;
    private GamepadEx driverJoystick;

    @Override
    public void initialize(){
        mecanumSub = new MecanumSub(
                hardwareMap
        );

        driverJoystick = new GamepadEx(gamepad1);


        configureAlliance();
        setDefaultCommands();

    }

    /**
     * Apply a joystick deadband so tiny inputs don’t move the motors.
     *
     * @param value     joystick value
     * @param threshold minimum absolute value to count as input
     * @return filtered value
     */
    private double applyDeadband(double value, double threshold) {
        return (Math.abs(value) > threshold) ? value : 0.0;
    }

    public void configureAlliance(){
        ButtonReader redSelect =
                new ButtonReader(driverJoystick, GamepadKeys.Button.B);
        ButtonReader blueSelect =
                new ButtonReader(driverJoystick, GamepadKeys.Button.X);

        while(opModeInInit()){
            redSelect.readValue();
            blueSelect.readValue();

            if(redSelect.wasJustPressed()){
                MatchConfig.alliance = MatchConfig.AllianceColor.RED;
            } else if (blueSelect.wasJustPressed()){
                MatchConfig.alliance = MatchConfig.AllianceColor.BLUE;
            }
        }
    }

    public void setDefaultCommands(){
        mecanumSub.setDefaultCommand(
                new MecanumDriveCMD(
                        mecanumSub,
                        () -> applyDeadband(driverJoystick.getLeftY(), Constants.OIConstants.MecanumOIConstants.joystickDeadband),  // Forward/back
                        () -> applyDeadband(driverJoystick.getLeftX(), Constants.OIConstants.MecanumOIConstants.joystickDeadband),  // Strafe
                        () -> applyDeadband(driverJoystick.getRightX(), Constants.OIConstants.MecanumOIConstants.joystickDeadband), // Rotate
                        () -> driverJoystick.getButton(GamepadKeys.Button.START)        // Reset gyro
                )
        );

    }
}
