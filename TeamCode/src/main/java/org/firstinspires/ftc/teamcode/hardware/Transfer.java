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
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Transfer extends Subsystem {
    public Servo liftRight;
    public Servo liftLeft;

    public Servo protectorRight;
    public Servo protectorLeft;

    public  static double liftRightSpeed = -0.9;

    public  static double liftLeftSpeed = 0.9;

    public  double isThisGettingCalled = 0;

    public  static double protectorRightPosition1 = 0.63;
    public  static double protectorRightPosition2 = 0.97;

    public  static double protectorLeftPosition1 = 0.68;
    public  static double protectorLeftPosition2 = 0.01;

    public  static  double protectorDelaySeconds = 0.4;

    public static double shooterVelocityTolerance = 30;   // ticks/sec,


    public static double rotatorStep = 0.01;

    public  boolean transferedEnabled = false;


    private  Transfer() {}

    public static Transfer INSTANCE = new Transfer();

    public int ballsFired = 0;

    public boolean previouslyFired = false;

    public  enum  TRANSFER_MODE { START_SHOOTING, STOP_SHOOTING,  AUTO_MODE };



    private final ElapsedTime shotTimer = new ElapsedTime();
    private boolean isFiring = false;
    public static double MIN_SHOT_INTERVAL_S = 0.30;
    public static double POS_EPS = 0.02;




    public void initialize() {
        liftRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "liftRight");
        liftLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "liftLeft");
        protectorRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "protectorRight");
        protectorLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "protectorLeft");


        liftRight.setPosition(0.5);
        liftLeft.setPosition(0.5);
        protectorRight.setPosition(protectorRightPosition1);
        protectorLeft.setPosition(protectorLeftPosition1);

        previouslyFired = false;
        ballsFired = 0;
        transferedEnabled = false;
        goToDefault();


    }

    public InstantCommand goToDefault() {
        return new InstantCommand(()-> {
            liftRight.setPosition(0.5);
            liftLeft.setPosition(0.5);
            protectorRight.setPosition(protectorRightPosition1);
            protectorLeft.setPosition(protectorLeftPosition1);
            Intake.INSTANCE.intakeMotors.setPower(0);
            Outtake.INSTANCE.stopMotor().invoke();
        });
    }

    public Command transferBall(TRANSFER_MODE transferMode  ) {
        // protector moves, wait, then kicker moves
        if (transferMode == TRANSFER_MODE.START_SHOOTING) {
            transferedEnabled = false;
        } else if (transferMode == TRANSFER_MODE.STOP_SHOOTING ) {
            transferedEnabled = true;
        }


        if (!transferedEnabled) {


            transferedEnabled = true;
            return new SequentialGroup(
                    new InstantCommand(() -> protectorRight.setPosition(protectorRightPosition2)),
                    new InstantCommand(() -> protectorLeft.setPosition(protectorLeftPosition2)),

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

    private boolean near(double a, double b) { return Math.abs(a-b) < POS_EPS; }

    public void updateWheelSpeedTick() {
        if (!transferedEnabled) return;

        double v = Math.abs(Outtake.INSTANCE.getOuttakeGroupVelocity());
        double target = Outtake.motorVelocityTarget;
        boolean atSpeed = Math.abs(v - target) <= shooterVelocityTolerance;

        boolean liftRetracted = near(liftRight.getPosition(), 0.5);
        boolean liftFired     = near(liftLeft.getPosition(), liftLeftSpeed); // or check both L/R

        // Decide whether we *should* be firing
        if (atSpeed && liftRetracted && !isFiring) {
            // start a firing stroke
            liftRight.setPosition(liftRightSpeed);
            liftLeft.setPosition(liftLeftSpeed);
            Intake.INSTANCE.intakeMotors.setPower(-1);
            isFiring = true;
        }

        // End of stroke → count exactly once, debounced
        if (isFiring && liftFired) {
            // retract (you can also delay here if needed)
            liftRight.setPosition(0.5);
            liftLeft.setPosition(0.5);
            Intake.INSTANCE.intakeMotors.setPower(0);

            if (shotTimer.seconds() >= MIN_SHOT_INTERVAL_S) {
                ballsFired++;
                shotTimer.reset();
            }
            isFiring = false;
        }
    }

    public InstantCommand rotateUp() {
        return new InstantCommand(()-> {
            protectorLeft.setPosition( protectorLeft.getPosition() + rotatorStep );
        });
    }

    public InstantCommand rotateDown() {
        return new InstantCommand(()-> {
            protectorLeft.setPosition( protectorLeft.getPosition() - rotatorStep );
        });
    }








}
