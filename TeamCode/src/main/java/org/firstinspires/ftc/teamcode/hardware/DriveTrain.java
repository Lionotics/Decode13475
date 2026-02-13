package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.jetbrains.annotations.NotNull;

import java.util.Collections;
import java.util.Set;


@Config
public class DriveTrain extends Subsystem {
    public static final DriveTrain INSTANCE = new DriveTrain();
    private DriveTrain() {
    }
    public static double maxSpeed = 1;
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private MotorEx[] motors;
    private IMU imu;

    public GoBildaPinpointDriver odometry;

    // --- Tag world estimate (odometry frame) ---
    public static boolean haveTagEstimate = false;
    public static double blueTagX_in = 0.0;
    public static double blueTagY_in = 0.0;

    // If your camera is not at robot center, put offsets here (inches).
// +X = forward, +Y = left (matches what you described).
    public static double camOffsetX_in = 0.0;
    public static double camOffsetY_in = 0.0;

    // How aggressively we trust new measurements (0..1). Higher = updates faster.
    public static double tagEstimateAlpha = 0.25;

    // Turning behavior when tag is NOT visible
    public static double kP_noTag = 0.05;     // power per degree
    public static double maxTurn_noTag = 0.6;
    public static double minTurn_noTag = 0.15;

    // Turning behavior when tag IS visible (fine alignment)
    public static double kP_inTag = 0.05;
    public static double maxTurn_inTag = 0.15;
    public static double minTurn_inTag = 0.2;

    public static double searchPowerBeforeTag = 0.35;



    public  boolean drivingFieldCentricNoTurnIsActivated = false;


    int GOAL_TAG_ID = 0;

    // Tune these:

    public static double desiredTilt = -4;
    public static double deadbandDeg = 2.0;
    public static long timeoutMs = 25000;

    public static double turnPowerValue = 0.33;

    // tiny “mutable holders” for lambdas
    final long[] startTime = new long[1];
    final boolean[] sawTag = new boolean[1];
    final double[] lastErrorDeg = new double[1];

    private AprilTagDetection d;




    public void initialize() {
       // frontLeft = new MotorEx("frontLeft");
       // frontRight = new MotorEx("frontRight");
      //  backLeft = new MotorEx("backLeft");
      //  backRight = new MotorEx("backRight");

        frontLeft = new MotorEx("backLeft");
        frontRight = new MotorEx("backRight");
        backLeft = new MotorEx("frontLeft");
        backRight = new MotorEx("frontRight");



        odometry = OpModeData.INSTANCE.getHardwareMap().get(GoBildaPinpointDriver.class, "odometryForward");


        frontLeft.reverse();
        backLeft.reverse();

        motors = new MotorEx[]{frontLeft, frontRight, backLeft, backRight};

        initIMU(OpModeData.INSTANCE.getHardwareMap());

        Webcam.INSTANCE.setSoleTagID(GOAL_TAG_ID);

    }

    public void periodic() {
        d = Webcam.INSTANCE.bestGoalDetection;

        // Always keep odometry-based estimate fresh when tag is visible
        if (d != null && d.ftcPose != null) {
            updateBlueTagEstimate(d);
        }
    }



    public void initIMU(HardwareMap hwMap) {
        // Retrieve the IMU from the hardware map
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();


        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometry.resetPosAndIMU();

    }

    public Command Drive(GamepadEx gamepad, boolean robotOriented) {
        // Please work
        // The normal NextFTC drive command
        MecanumDriverControlled inner =
                new MecanumDriverControlled(motors, gamepad, robotOriented, imu);

        // Wrap it so it REQUIRES (claims) this drivetrain subsystem
        return new Command() {

            @NotNull
            @Override
            public Set<Subsystem> getSubsystems() {
                return Collections.singleton(DriveTrain.this);
            }

            @Override
            public boolean isDone() {
                return false;
            }

            @Override
            public void start() {
                drivingFieldCentricNoTurnIsActivated = false;
                inner.start();
            }

            @Override
            public void update() {
                inner.update();
            }

            @Override
            public void stop(boolean interrupted) {
                inner.stop(interrupted);
            }
        };
    }

    private static double wrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }


    public void setGoalID(int id) {
        GOAL_TAG_ID = id;
    }


    public Command setPowerToWheelsOnceAtATime() {
        int delaySeconds = 2;

        return new SequentialGroup(
                new SetPower(frontRight, 1, this),
                new Delay(delaySeconds),
                new SetPower(frontRight, 0, this),

                new SetPower(frontLeft, 1, this),
                new Delay(delaySeconds),
                new SetPower(frontLeft, 0, this),

                new SetPower(backRight, 1, this),
                new Delay(delaySeconds),
                new SetPower(backRight, 0, this),

                new SetPower(backLeft, 1, this),
                new Delay(delaySeconds),
                new SetPower(backLeft, 0, this)
        );
    }

    public void setTurnPower(double turn) {
        turn = Range.clip(turn, -1.0, 1.0);

        // Turn in place: left side forward, right side backward (signs may be flipped if you prefer)
        frontLeft.setPower(turn);
        backLeft.setPower(turn);
        frontRight.setPower(-turn);
        backRight.setPower(-turn);
    }




    private void updateBlueTagEstimate(AprilTagDetection d) {
        if (d == null || d.ftcPose == null || odometry == null) return;

        Pose2D pose = odometry.getPosition();

        double rx = pose.getX(DistanceUnit.INCH);
        double ry = pose.getY(DistanceUnit.INCH);
        double hDeg = pose.getHeading(AngleUnit.DEGREES);
        //double hDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // we use Imu for heading since using pose caused some problem with MechnamControlled


        // Camera position in world frame (approx)
        double hRad = Math.toRadians(hDeg);
        double camX = rx + camOffsetX_in * Math.cos(hRad) - camOffsetY_in * Math.sin(hRad);
        double camY = ry + camOffsetX_in * Math.sin(hRad) + camOffsetY_in * Math.cos(hRad);

        // Direction from camera to tag in world frame
        double dirDeg = hDeg + d.ftcPose.bearing;   // bearing positive = camera must turn left/right :contentReference[oaicite:5]{index=5}
        double dirRad = Math.toRadians(dirDeg);

        double measTagX = camX + d.ftcPose.range * Math.cos(dirRad);
        double measTagY = camY + d.ftcPose.range * Math.sin(dirRad);

        if (!haveTagEstimate) {
            blueTagX_in = measTagX;
            blueTagY_in = measTagY;
            haveTagEstimate = true;
        } else {
            // EMA filter to smooth noise
            blueTagX_in = (1 - tagEstimateAlpha) * blueTagX_in + tagEstimateAlpha * measTagX;
            blueTagY_in = (1 - tagEstimateAlpha) * blueTagY_in + tagEstimateAlpha * measTagY;
        }
    }

    private static double signedMinPower(double power, double minAbs) {
        if (Math.abs(power) < minAbs) return Math.copySign(minAbs, power);
        return power;
    }

    public void stopDrive() {
        setTurnPower(0.0);
    }


    public Command faceBlueGoal = new LambdaCommand()
            .setSubsystems(this)
            .setStart(() -> {
                startTime[0] = System.currentTimeMillis();
                sawTag[0] = false;
                lastErrorDeg[0] = 999;
            })
            .setUpdate(() -> {

                // If tag is visible: fine align using BEARING
                if (d != null && d.ftcPose != null) {
                    sawTag[0] = true;

                    double errorDeg = wrapDeg(desiredTilt - d.ftcPose.bearing);
                    lastErrorDeg[0] = errorDeg;

                    if (Math.abs(errorDeg) < deadbandDeg) {
                        DriveTrain.INSTANCE.setTurnPower(0.0);
                        return;
                    }

                    double turn = Range.clip(kP_inTag * errorDeg, -maxTurn_inTag, maxTurn_inTag);
                    turn = signedMinPower(turn, minTurn_inTag);
                    DriveTrain.INSTANCE.setTurnPower(turn);
                    return;
                }

                // Tag NOT visible: use odometry to "pre-aim" toward where we think the tag is
                if (haveTagEstimate && odometry != null) {
                    Pose2D pose = odometry.getPosition();
                    double rx = pose.getX(DistanceUnit.INCH);
                    double ry = pose.getY(DistanceUnit.INCH);
                    double hDeg = pose.getHeading(AngleUnit.DEGREES);
                    //double hDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


                    double dx = blueTagX_in - rx;
                    double dy = blueTagY_in - ry;

                    // heading to face the tag (in your x-forward, y-left frame)
                    double dirToTagDeg = Math.toDegrees(Math.atan2(dy, dx));

                    // We want camera bearing to be desiredTilt, so:
                    // bearing ≈ dirToTag - heading  =>  headingTarget ≈ dirToTag - desiredTilt
                    double headingTargetDeg = wrapDeg( dirToTagDeg - desiredTilt);

                    double errorDeg = wrapDeg(headingTargetDeg - hDeg);
                    errorDeg = -1 * errorDeg;
                    lastErrorDeg[0] = 999; // only allow the "facebluegoaL" to be done if robot sees the april tag

                    if (Math.abs(errorDeg) < deadbandDeg) {
                        DriveTrain.INSTANCE.setTurnPower(0.0);
                        return;
                    }

                    double turn = Range.clip(kP_noTag * errorDeg, -maxTurn_noTag, maxTurn_noTag);
                    turn = signedMinPower(turn, minTurn_noTag);
                    DriveTrain.INSTANCE.setTurnPower(turn);
                    return;
                }

                // If we have NO estimate yet: do a gentle alternating search (prevents "always clockwise")
                DriveTrain.INSTANCE.setTurnPower(searchPowerBeforeTag);
            })
            .setIsDone(() -> {
                long elapsed = System.currentTimeMillis() - startTime[0];
                if (elapsed > timeoutMs) {
                    return true;
                }
                if (!sawTag[0]) {
                    return false;
                } // keep hunting until timeout
                return Math.abs(lastErrorDeg[0]) < deadbandDeg;
            })
            .setStop(interrupted -> {
                DriveTrain.INSTANCE.stopDrive();
                // resume normal teleop drive when done (or interrupted)
            });



}