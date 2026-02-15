package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;

import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.autoCommands;
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

        gp1.getX().setPressedCommand(
                () -> new SequentialGroup(
                        Intake.INSTANCE.eat(),

                        new InstantCommand( ()->
                        {
                            if (Intake.INSTANCE.intakeMotors.getPower() != 0) {
                                Transfer.INSTANCE.liftRight.setPosition(Transfer.liftRightSpeedIntake);
                                Transfer.INSTANCE.liftLeft.setPosition(Transfer.liftLeftSpeedIntake);
                            } else {
                                Transfer.INSTANCE.liftLeft.setPosition(0.5);
                                Transfer.INSTANCE.liftRight.setPosition(0.5);
                            }
                        }

                                )
                        )
        );

        gp1.getY().setPressedCommand(
                () -> new SequentialGroup(
                        new InstantCommand( ()->autoScoreCancelled = false ),
                    //    DriveTrain.INSTANCE.faceBlueGoal,         // keep aiming here
                        new InstantCommand(() -> driverControlled.invoke()),

                        autoCommands.autoScoreCore(),
                        Transfer.INSTANCE.transferBall(Transfer.TRANSFER_MODE.STOP_SHOOTING)//,

                        // autoCommands.autoScore(),

                )

        );



        gp1.getRightBumper().setPressedCommand(
                () -> new SequentialGroup( Intake.INSTANCE.spit(),
                        new InstantCommand( ()->Transfer.INSTANCE.liftLeft.setPosition(0.5)),
                        new InstantCommand( ()->Transfer.INSTANCE.liftRight.setPosition(0.5))
                )
        );



        gp1.getDpadLeft().setPressedCommand( ()->Outtake.INSTANCE.MotorVelocityToLower() );

        gp1.getDpadRight().setPressedCommand( ()->Outtake.INSTANCE.MotorVelocityToHigher() );


        gp1.getA().setPressedCommand(() -> Outtake.INSTANCE.startMotor(Outtake.DISTANCE_OR_MOTOR_POWER.DISTANCE, autoCommands.getWebCamDistance() ));


        gp1.getB().setPressedCommand(


                () ->
                        new SequentialGroup(
                                new InstantCommand( ()-> autoScoreCancelled = true ),
                                Transfer.INSTANCE.transferBall(Transfer.TRANSFER_MODE.STOP_SHOOTING),
                                new InstantCommand(() -> driverControlled.invoke())
                        )
        );

        gp1.getLeftBumper().setPressedCommand(

                () -> new SequentialGroup(
                        new InstantCommand( ()->autoScoreCancelled = false ),
                        DriveTrain.INSTANCE.faceBlueGoal,         // keep aiming here
                        new InstantCommand(() -> driverControlled.invoke()),

                        autoCommands.autoScoreCore(),
                        Transfer.INSTANCE.transferBall(Transfer.TRANSFER_MODE.STOP_SHOOTING)//,

                       // autoCommands.autoScore(),

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

        telemetry.addData("Robot Sees Tag? ", DriveTrain.INSTANCE.TagStatus);


        Webcam.INSTANCE.addTelemetry(telemetry);




        telemetry.update();


    }



    @Override
    public  void onStop() {
        Webcam.INSTANCE.close();
        FtcDashboard.getInstance().stopCameraStream();
    }




}
