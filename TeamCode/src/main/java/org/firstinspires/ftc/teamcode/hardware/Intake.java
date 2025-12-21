package org.firstinspires.ftc.teamcode.hardware;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;

public class Intake extends Subsystem {


    public MotorEx intakeRight;
    public MotorEx intakeLeft;

    public  MotorGroup intakeMotors;


    private Intake() {

    }

    public void initialize() {

        intakeRight = new MotorEx("intakeRight");
        intakeLeft = new MotorEx("intakeLeft");
        intakeLeft.reverse();

        intakeMotors  = new MotorGroup(intakeRight, intakeLeft);

    }

    public static Intake INSTANCE = new Intake();

    public InstantCommand setPowerToIntake(double i) {
        return new InstantCommand(()-> {
            intakeMotors.setPower(i);
        });
    }

    public InstantCommand eat() {
        return new InstantCommand(()-> {
            if (intakeRight.getPower() != 0) {
                intakeMotors.setPower(0);
            } else {
                intakeMotors.setPower(-1);

            }
        });
    }

    public InstantCommand spit() {
        return new InstantCommand(()-> {
            if (intakeRight.getPower() != 0) {
                intakeMotors.setPower(0);
            } else {
                intakeMotors.setPower(1);

            }
        });
    }

    public Command loadBall( double loadDelaySecond ) {
        return new SequentialGroup(
                new InstantCommand(() -> intakeMotors.setPower(-1) ),
                new Delay(loadDelaySecond),
                new InstantCommand(() ->  intakeMotors.setPower(0) )

        );

    }

    public double getMotorsVelocity() {
        return intakeRight.getPower();
    }




}
