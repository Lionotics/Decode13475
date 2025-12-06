package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;


import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToVelocity;

@Config
public class Outtake extends Subsystem {

    public  static double motorPower = 0.5;

    public  static double motorVelocityStep = 1;

    public  static  double kP = 0.06;
    public  static  double kI = 0.00;
    public  static  double kD = 0.002;



    public  static double motorVelocityTargetLower = 770;
    public  static double motorVelocityTargetHigher = 875;

    public  static double motorVelocityCurrent = motorVelocityTargetHigher;

    public  static  boolean motorIsOnHigher = true;





    private final PIDFController outtakeVelocityController = new PIDFController(
            kP,  // kP
            kI,   // kI
            kD    // kD
    );


    private  Outtake() {}

    public static Outtake INSTANCE = new Outtake();

    private MotorEx motorOuttakeRight;
    private MotorEx motorOuttakeLeft;

    private  MotorGroup outtakeGroup;




    public void initialize() {
        motorOuttakeRight = new MotorEx("outtakeRight");
        motorOuttakeLeft = new MotorEx("outtakeLeft");

       // motorOuttakeLeft.reverse();
        motorOuttakeRight.reverse();

        outtakeGroup = new MotorGroup(motorOuttakeLeft, motorOuttakeRight);
    }

    public InstantCommand setPowerToMotorS(double i) {
        return new InstantCommand(()-> {
            //motorOuttakeRight.setPower(i*motorPower);
            outtakeGroup.setPower(i*motorPower);


        });
    }

    public double targetVelocityToActualVelocity(double targetVelocity) {
        return  -targetVelocity;
        //return  0.0418 * targetVelocity + 5;
    }

    public Command startMotor() {
        double targetTemp  = targetVelocityToActualVelocity(motorVelocityCurrent); // ignore direction


        return new RunToVelocity(
                outtakeGroup,
                targetTemp,
                outtakeVelocityController,
                this
        );

    }

    public Command stopMotor() {
        return new InstantCommand(() -> {
            outtakeGroup.setPower(0);
        });
    }

    public Command raiseMotorVelocity() {
        return new InstantCommand(() -> {
            motorVelocityCurrent += motorVelocityStep;
        });
    }

    public Command lowerMotorVelocity() {
        return new InstantCommand(() -> {
            motorVelocityCurrent -= motorVelocityStep;
        });
    }


    public Command MotorVelocityToHigher() {
        return new InstantCommand(() -> {
            if (!motorIsOnHigher) {
                motorVelocityTargetLower = motorVelocityCurrent;
                motorVelocityCurrent = motorVelocityTargetHigher;
                motorIsOnHigher = true;
            }
        });
    };

    public Command MotorVelocityToLower() {
        return new InstantCommand(() -> {
            if (motorIsOnHigher) {
                motorVelocityTargetHigher = motorVelocityCurrent;
                motorVelocityCurrent = motorVelocityTargetLower;
                motorIsOnHigher = false;
            }
        });
    };


    public  double getMotorCurrentLeftVelocity() {
        return motorOuttakeLeft.getVelocity();
    }

    public  double getMotorCurrentRightVelocity() {
        return motorOuttakeRight.getVelocity();
    }


}
