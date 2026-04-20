package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystem.MecanumSub;

import java.util.function.Supplier;

public class MecanumDriveCMD extends CommandBase {


    private final MecanumSub driveSub;
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> rSupplier;
    private final Supplier<Boolean> resetSupplier;

    public MecanumDriveCMD(
            MecanumSub driveSub,
            Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> rSupplier,
            Supplier<Boolean> resetSupplier
    ) {

        this.driveSub = driveSub;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        this.resetSupplier = resetSupplier;
        addRequirements(driveSub);
    }

    @Override
    public void execute() {

            double forward = -ySupplier.get();
            double strafe = -xSupplier.get();
            double rotation = -rSupplier.get();

            driveSub.drive(forward, strafe, rotation);

            boolean reset = resetSupplier.get();

        if(reset){
            driveSub.resetOdometry();
        }
    }

}