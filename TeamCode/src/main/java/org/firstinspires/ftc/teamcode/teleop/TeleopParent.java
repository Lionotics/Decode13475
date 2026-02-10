package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.ForcedParallelCommand;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;

import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Webcam;

@Config
public class TeleopParent extends NextFTCOpMode {

    public final int BLUE_TAG_ID = 20;
    public final int RED_TAG_ID = 24;

    private boolean autoScoreCancelled = false;

    public Command driverControlled;

    public TeleopParent() {
        super( Outtake.INSTANCE, DriveTrain.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, Webcam.INSTANCE);
    }

    private GamepadEx gp1;

    @Override
    public void onStartButtonPressed() {
        autoScoreCancelled = false;

        FtcDashboard.getInstance().startCameraStream(Webcam.INSTANCE.getVisionPortal(), 30);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverControlled = DriveTrain.INSTANCE.Drive(gamepadManager.getGamepad1(), false);
        driverControlled.invoke();




        GamepadEx gp1 = gamepadManager.getGamepad1();

        gp1.getX().setPressedCommand(() -> Intake.INSTANCE.eat());

        gp1.getY().setPressedCommand(() -> Transfer.INSTANCE.transferBall(Transfer.TRANSFER_MODE.AUTO_MODE));



        gp1.getRightBumper().setPressedCommand(() -> Intake.INSTANCE.spit());



        gp1.getDpadLeft().setPressedCommand( ()->Outtake.INSTANCE.MotorVelocityToLower() );

        gp1.getDpadRight().setPressedCommand( ()->Outtake.INSTANCE.MotorVelocityToHigher() );


        gp1.getA().setPressedCommand(() -> Outtake.INSTANCE.startMotor(Outtake.DISTANCE_OR_MOTOR_POWER.DISTANCE,getWebCamDistance() ));


        gp1.getB().setPressedCommand(


                () ->
                        new SequentialGroup(
                                new InstantCommand( ()-> autoScoreCancelled = true ),

                                Transfer.INSTANCE.transferBall(Transfer.TRANSFER_MODE.STOP_SHOOTING)
                        )
        );


       /* gp1.getRightTrigger().setPressedCommand(value -> {
            // value is the analog trigger value (0.0–1.0); you can ignore it
            Outtake.INSTANCE.startMotor();   // side effect
            return new NullCommand();        // schedules a do-nothing command
        });

        gp1.getLeftTrigger().setPressedCommand(value -> {
            // value is the analog trigger value (0.0–1.0); you can ignore it
            Outtake.INSTANCE.stopMotor();   // side effect
            return new NullCommand();        // schedules a do-nothing command
        }); */



        gp1.getLeftBumper().setPressedCommand(

                () -> new SequentialGroup(
                        new InstantCommand( ()->autoScoreCancelled = false ),
                        autoScore()
                ) );

        gp1.getDpadUp().setHeldCommand( ()-> Outtake.INSTANCE.raiseMotorVelocity() );
        gp1.getDpadDown().setHeldCommand( ()-> Outtake.INSTANCE.lowerMotorVelocity() );


    }

    @Override
    public void onUpdate() {
        Transfer.INSTANCE.updateWheelSpeedTick();
        telemetry.addData("Motor Outtake Left Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentLeftVelocity());
        telemetry.addData("Motor Outtake Right Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentRightVelocity());
        telemetry.addData("Motor Outtake Target Velocity: ",  Outtake.motorVelocityTarget);
        telemetry.addData("Motor Velocity Is Higher (true if Higher, false if Lower): ",  Outtake.motorIsOnHigher);
        telemetry.addData("Is transfer enabled: ", Transfer.INSTANCE.transferedEnabled );
        telemetry.addData("Balls Fired: ", Transfer.INSTANCE.ballsFired );


        if ( DriveTrain.INSTANCE.odometry != null) {
            DriveTrain.INSTANCE.odometry.update(); // read sensors and update internal pose

            Pose2D pose2d = DriveTrain.INSTANCE.odometry.getPosition();


            double xInches = pose2d.getX(DistanceUnit.INCH);
            double yInches = pose2d.getY(DistanceUnit.INCH);
            double headingDeg = pose2d.getHeading(AngleUnit.DEGREES); // LATER NEED TO COMBINE BOTH ODOS TO GET DEGREE

            telemetry.addData("Odo X (in)", xInches);
            telemetry.addData("Odo Y (in)", yInches);
            telemetry.addData("Odo Heading (deg)", headingDeg);
        }

        Webcam.INSTANCE.addTelemetry(telemetry);


        telemetry.update();


    }



    @Override
    public  void onStop() {
        Webcam.INSTANCE.close();
        FtcDashboard.getInstance().stopCameraStream();
    }


    public Command autoScore() {
        return new SequentialGroup(
                DriveTrain.INSTANCE.faceBlueGoal,

                // This step runs ONLY after faceBlueGoal is finished
                new Command() {
                    private Command afterFace;

                    @Override
                    public void start() {
                        Transfer.INSTANCE.ballsFired = 0;
                        double webCamDistance = getWebCamDistance();



                       // double targetTempRaw = Outtake.INSTANCE.distanceToVelocity(webCamDistance);

                        afterFace = new SequentialGroup(
                                Transfer.INSTANCE.transferBall(Transfer.TRANSFER_MODE.START_SHOOTING),

                             new ForcedParallelCommand(Outtake.INSTANCE.startMotor(Outtake.DISTANCE_OR_MOTOR_POWER.DISTANCE,getWebCamDistance()))
                        );

                        afterFace.invoke();
                    }

                    @Override
                    public void update() { }

                    @Override
                    public boolean isDone() {
                        return (afterFace != null &&  Transfer.INSTANCE.ballsFired >= 3) || autoScoreCancelled;
                    }

                    @Override
                    public void stop(boolean interrupted) {
                        if (autoScoreCancelled) {
                            interrupted = true;
                        }

                        if (interrupted) {
                            Transfer.INSTANCE.transferBall(Transfer.TRANSFER_MODE.STOP_SHOOTING);
                            Outtake.INSTANCE.stopMotor().invoke();
                        }
                    }
                },
                Transfer.INSTANCE.transferBall(Transfer.TRANSFER_MODE.STOP_SHOOTING) ,
                 new InstantCommand(() -> driverControlled.invoke())
        );
    }

    public  double getWebCamDistance() {
        double webCamDistance;
        if (Webcam.INSTANCE.seesTag()) {
            webCamDistance = Webcam.INSTANCE.getRange();
        } else if (DriveTrain.haveTagEstimate && DriveTrain.INSTANCE.odometry != null) {
            // Make sure pose is fresh right now (not just onUpdate)
            DriveTrain.INSTANCE.odometry.update();

            Pose2D pose = DriveTrain.INSTANCE.odometry.getPosition();
            double rx = pose.getX(DistanceUnit.INCH);
            double ry = pose.getY(DistanceUnit.INCH);

            double dx = DriveTrain.blueTagX_in - rx;
            double dy = DriveTrain.blueTagY_in - ry;

            webCamDistance = Math.hypot(dx, dy); // inches
        } else {
            webCamDistance = 30.0; // last-resort fallback if you *never* saw the tag yet
        }
        return  webCamDistance;
    }


}
