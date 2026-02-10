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

    public  static double liftRightSpeed = 0.1;

    public  static double liftLeftSpeed = 0.9;

    public  static double protectorRightPosition1 = 0.63;
    public  static double protectorRightPosition2 = 0.97;

    public  static double protectorLeftPosition1 = 0.68;
    public  static double protectorLeftPosition2 = 0.01;

    public  static  double protectorDelaySeconds = 0.4;

    public static double shooterVelocityTolerance = 30;   // ticks/sec,

    public static  double shootingDelaySeconds = 1.25;


    public static double rotatorStep = 0.01;

    public  boolean transferedEnabled = false;


    private  Transfer() {}

    public static Transfer INSTANCE = new Transfer();

    public int ballsFired = 0;

    public boolean previouslyFired = false;

    public  enum  TRANSFER_MODE { START_SHOOTING, STOP_SHOOTING,  AUTO_MODE };



    private boolean isFiring = false;

    private  boolean previousallyIncreamtedBallsFired = false;

    public static double POS_EPS = 0.02;


    private final ElapsedTime atSpeedTimer = new ElapsedTime();
    private boolean speedTimerRunning = false;

    private boolean shotLatched = false; // blocks repeat shots while delayPassed stays true






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
        isFiring = false;
        previousallyIncreamtedBallsFired = false;


    }

    public InstantCommand goToDefault() {
        return new InstantCommand(()-> {
            liftRight.setPosition(0.5);
            liftLeft.setPosition(0.5);
            protectorRight.setPosition(protectorRightPosition1);
            protectorLeft.setPosition(protectorLeftPosition1);
            Intake.INSTANCE.intakeMotors.setPower(0);
            Outtake.INSTANCE.stopMotor().invoke();
            ballsFired = 0;
            isFiring = false;
            previousallyIncreamtedBallsFired = false;
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
        // If transfer isn't enabled, reset all shot state and exit
        if (!transferedEnabled) {
            speedTimerRunning = false;
            isFiring = false;
            previousallyIncreamtedBallsFired = false;
            shotLatched = false;
            return;
        }

        // --- Flywheel atSpeed + "stable for X seconds" logic (unchanged conceptually) ---
        double v = Math.abs(Outtake.INSTANCE.getOuttakeGroupVelocity());
        double target = Outtake.motorVelocityTarget;
        boolean atSpeed = Math.abs(v - target) <= shooterVelocityTolerance;

        // Start/reset timer when we FIRST reach atSpeed, and reset if we fall out of atSpeed
        if (atSpeed) {
            if (!speedTimerRunning) {
                atSpeedTimer.reset();
                speedTimerRunning = true;
            }
        } else {
            speedTimerRunning = false;
        }

        boolean delayPassed = speedTimerRunning && atSpeedTimer.seconds() >= shootingDelaySeconds;

        // "Retracted" is safe to check (it's your commanded resting position)
        boolean liftRetracted = near(liftRight.getPosition(), 0.5) && near(liftLeft.getPosition(), 0.5);

        // --- Re-arm logic ---
        // Once we're NOT in the "ready-to-shoot" window anymore (delayPassed false),
        // we allow another shot later (after the wheel recovers and delayPassed becomes true again).
        if (!delayPassed && !isFiring) {
            shotLatched = false;
            previousallyIncreamtedBallsFired = false;
        }

        // --- Start shot (exactly once per "ready" window) ---
        if (delayPassed && liftRetracted && !isFiring && !shotLatched) {
            // Command the kick
            liftRight.setPosition(liftRightSpeed);
            liftLeft.setPosition(liftLeftSpeed);
            Intake.INSTANCE.intakeMotors.setPower(-1);

            isFiring = true;
            // Don't increment here; wait for evidence the shot actually happened.
        }

        // --- Detect shot completion WITHOUT servo position ---
        // A real shot almost always causes a brief flywheel speed dip -> atSpeed becomes false.
        // Use that dip as the "shot happened" event.
        if (isFiring && !atSpeed) {
            // Increment exactly once
            if (!previousallyIncreamtedBallsFired) {
                ballsFired += 1;
                previousallyIncreamtedBallsFired = true;
            }

            // Retract
            liftRight.setPosition(0.5);
            liftLeft.setPosition(0.5);
            Intake.INSTANCE.intakeMotors.setPower(0);

            isFiring = false;

            // Latch so we cannot re-fire again while delayPassed is still true (if it ever stays true)
            shotLatched = true;
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
