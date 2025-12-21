package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

@Config
public class Transfer extends Subsystem {
    public Servo liftRight;
    public Servo liftLeft;

    public Servo protector;


    public  static double liftRightSpeed = -0.9;

    public  static double liftLeftSpeed = 0.9;

    public  double isThisGettingCalled = 0;

    public  static double protectorPosition1 = 0.63;
    public  static double protectorPosition2 = 0.97;

    public  static  double protectorDelaySeconds = 0.4;

    public static double shooterVelocityTolerance = 30;   // ticks/sec,


    public static double rotatorStep = 0.01;

    public  boolean transferedEnabled = false;


    private  Transfer() {}

    public static Transfer INSTANCE = new Transfer();




    public void initialize() {
        liftRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "liftRight");
        liftLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "liftLeft");
        protector = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "Protector");

        liftRight.setPosition(0.5);
        liftLeft.setPosition(0.5);
        protector.setPosition(protectorPosition1);

        transferedEnabled = false;
        goToDefault();

    }

    public InstantCommand goToDefault() {
        return new InstantCommand(()-> {
            liftRight.setPosition(0.5);
            liftLeft.setPosition(0.5);
            protector.setPosition(protectorPosition1);
        });
    }

    public Command transferBall() {
        // protector moves, wait, then kicker moves
        if (!transferedEnabled) {
            transferedEnabled = true;
            return new SequentialGroup(
                    new InstantCommand(() -> protector.setPosition(protectorPosition2)),
                    new Delay(protectorDelaySeconds)


                    // I want to make the servos continousally move until manually instructed not to

            );
        } else {
            transferedEnabled = false;
            return new SequentialGroup(
                   goToDefault()

                    // I want to make the servo stop moving

            );
        }
    }

    public Command updateWheelSpeed() {


        if (transferedEnabled) {
            if (  Math.abs( ( Outtake.INSTANCE.getMotorCurrentLeftVelocity() + Outtake.INSTANCE.getMotorCurrentRightVelocity() )/2 - Outtake.motorVelocityTarget) <= shooterVelocityTolerance &&  liftRight.getPosition() != 0.5) {
               return new SequentialGroup(
                        new InstantCommand(() -> liftRight.setPosition(0.5)),
                        new InstantCommand(() -> liftLeft.setPosition(0.5)),
                        new InstantCommand( ()-> Intake.INSTANCE.intakeMotors.setPower(0))
               );

            } else if ( Math.abs( (Outtake.INSTANCE.getMotorCurrentRightVelocity() + Outtake.INSTANCE.getMotorCurrentLeftVelocity())/2  -  Outtake.motorVelocityTarget)  >= shooterVelocityTolerance &&  liftRight.getPosition() != liftRightSpeed)  {
               return new SequentialGroup(
                        new InstantCommand(() -> liftRight.setPosition(liftRightSpeed)),
                        new InstantCommand(() -> liftLeft.setPosition(liftLeftSpeed)),
                       new InstantCommand(()->  Intake.INSTANCE.intakeMotors.setPower(1))

               );
            }
        }




        return new NullCommand();

    }








}
