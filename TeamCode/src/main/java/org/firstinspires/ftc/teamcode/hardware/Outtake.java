package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;


import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToVelocity;

@Config
public class Outtake extends Subsystem {

    public  static double motorPower = 0.5;

    public  static double motorVelocityStep = 1;

    public  static  double kPright = 0.01;
    public  static  double kIright = 0.00;
    public  static  double kDright = 0.000;


    public  static  double kPleft = 0.01;
    public  static  double kIleft = 0.00;
    public  static  double kDleft = 0.000;


    public  static double motorVelocityTargetLower = 770;
    public  static double motorVelocityTargetHigher = 875;

    public  static double motorVelocityTarget = motorVelocityTargetHigher;

    public  static  boolean motorIsOnHigher = true;

    public  enum  DISTANCE_OR_MOTOR_POWER {DISTANCE, MOTOR_POWER};

    public double distanceToMotorVelocity(double distance) {
        return  0.173 * distance * distance - 7.09 * distance + 807.56;
    }



    private final PIDFController leftController = new PIDFController(
            kPleft,  // kP
            kIleft,   // kI
            kDleft    // kD
    );

    private final PIDFController rightContoller = new PIDFController(
            kPright,  // kP
            kIright,   // kI
            kDright    // kD
    );


    private  Outtake() {}

    public static Outtake INSTANCE = new Outtake();

    private MotorEx motorOuttakeRight;
    private MotorEx motorOuttakeLeft;


    public  static  double velocityAdderRightMotor = 40;

    public  static  double velocityAdderLeftMotor = 40;




    public void initialize() {
        motorOuttakeRight = new MotorEx("outtakeRight");
        motorOuttakeLeft = new MotorEx("outtakeLeft");

        motorOuttakeLeft.reverse();
      //  motorOuttakeRight.reverse();

        motorIsOnHigher = true;
    }



    public double targetVelocityToActualVelocityRightMotor(double targetVelocity) {
        return  -targetVelocity- velocityAdderRightMotor;
    }

    public double targetVelocityToActualVelocityLeftMotor(double targetVelocity) {
        return  -targetVelocity- velocityAdderLeftMotor;
    }

    public Command startMotor(DISTANCE_OR_MOTOR_POWER type, double value ) {
        double targetTempRight;
        double targetTempLeft;

        if (type == DISTANCE_OR_MOTOR_POWER.MOTOR_POWER) {
            motorVelocityTarget = value;
            targetTempRight  = targetVelocityToActualVelocityRightMotor(value); // ignore direction
            targetTempLeft  = targetVelocityToActualVelocityLeftMotor(value); // ignore direction
        } else {
            double velocityRaw  = distanceToMotorVelocity(value);
            motorVelocityTarget = velocityRaw;
            targetTempRight  = targetVelocityToActualVelocityRightMotor(velocityRaw); // ignore direction
            targetTempLeft  = targetVelocityToActualVelocityLeftMotor(velocityRaw); // ignore direction

        }




        return new ParallelGroup(
                new RunToVelocity(motorOuttakeRight,  targetTempRight, rightContoller),
                new RunToVelocity(motorOuttakeLeft, targetTempLeft, leftController)
        );



    }

    public Command stopMotor() {


        return new ParallelGroup(
                new RunToVelocity(motorOuttakeRight,  0, rightContoller),
                new RunToVelocity(motorOuttakeLeft, 0, leftController)
        );
    }

    public Command raiseMotorVelocity() {
        return new InstantCommand(() -> {
            motorVelocityTarget += motorVelocityStep;
        });
    }

    public Command lowerMotorVelocity() {
        return new InstantCommand(() -> {
            motorVelocityTarget -= motorVelocityStep;
        });
    }


    public Command MotorVelocityToHigher() {
        return new InstantCommand(() -> {
            if (true || !motorIsOnHigher) {
              //  motorVelocityTargetLower = motorVelocityCurrent;
                motorVelocityTarget = motorVelocityTargetHigher;
                motorIsOnHigher = true;
            }
        });
    };

    public Command MotorVelocityToLower() {
        return new InstantCommand(() -> {
            if (true || motorIsOnHigher) {
               // motorVelocityTargetHigher = motorVelocityCurrent;
                motorVelocityTarget = motorVelocityTargetLower;
                motorIsOnHigher = false;
            }
        });
    };


    public  double getMotorCurrentLeftVelocity() {
        return -motorOuttakeLeft.getVelocity();
    }

    public  double getMotorCurrentRightVelocity() {
        return -motorOuttakeRight.getVelocity();
    }

    public  double getOuttakeGroupVelocity() {
        return ( (getMotorCurrentLeftVelocity()+getMotorCurrentRightVelocity()) /2 );
        //return  getMotorCurrentLeftVelocity();
    }


}
