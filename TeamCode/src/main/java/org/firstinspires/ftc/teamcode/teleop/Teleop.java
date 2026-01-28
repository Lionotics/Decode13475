package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
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
@TeleOp(name = "13475Teleop", group = "Teleop")
public class Teleop extends NextFTCOpMode {

    public Command driverControlled;

    public Teleop() {
        super( Outtake.INSTANCE, DriveTrain.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, Webcam.INSTANCE);
    }

    private GamepadEx gp1;

    @Override
    public void onStartButtonPressed() {
        FtcDashboard.getInstance().startCameraStream(Webcam.INSTANCE.getVisionPortal(), 30);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverControlled = DriveTrain.INSTANCE.Drive(gamepadManager.getGamepad1(), false);
        driverControlled.invoke();




        GamepadEx gp1 = gamepadManager.getGamepad1();

        gp1.getX().setPressedCommand(() -> Intake.INSTANCE.eat());


        gp1.getRightBumper().setPressedCommand(() -> Intake.INSTANCE.spit());



        gp1.getDpadLeft().setPressedCommand( ()->DriveTrain.INSTANCE.setPowerToWheelsOnceAtATime() );

        gp1.getDpadRight().setPressedCommand( ()->Outtake.INSTANCE.MotorVelocityToHigher() );


        gp1.getA().setPressedCommand(() -> Outtake.INSTANCE.startMotor());


        gp1.getB().setPressedCommand(() -> Outtake.INSTANCE.stopMotor());


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



        gp1.getLeftBumper().setPressedCommand(() -> Transfer.INSTANCE.transferBall());

        //gp1.getDpadUp().setHeldCommand( ()-> Outtake.INSTANCE.raiseMotorVelocity() );
        //gp1.getDpadDown().setHeldCommand( ()-> Outtake.INSTANCE.lowerMotorVelocity() );
        gp1.getDpadUp().setHeldCommand( ()-> Transfer.INSTANCE.rotateUp() );
        gp1.getDpadDown().setHeldCommand( ()-> Transfer.INSTANCE.rotateDown() );


    }

    @Override
    public void onUpdate() {
        Transfer.INSTANCE.updateWheelSpeed().invoke();
        telemetry.addData("Motor Outtake Left Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentLeftVelocity());
        telemetry.addData("Motor Outtake Right Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentRightVelocity());
        telemetry.addData("Motor Outtake Target Velocity: ",  Outtake.motorVelocityTarget);
        telemetry.addData("Motor Velocity Is Higher (true if Higher, false if Lower): ",  Outtake.motorIsOnHigher);
        telemetry.addData("Is transfer enabled: ", Transfer.INSTANCE.transferedEnabled );
        telemetry.addData("Is this getting called: ", Transfer.INSTANCE.isThisGettingCalled );

        if (DriveTrain.INSTANCE.odometryForward != null && DriveTrain.INSTANCE.odometrySideways != null) {
            DriveTrain.INSTANCE.odometryForward.update(); // read sensors and update internal pose
            DriveTrain.INSTANCE.odometrySideways.update();

            Pose2D poseX2d = DriveTrain.INSTANCE.odometryForward.getPosition();
            Pose2D poseY2d = DriveTrain.INSTANCE.odometrySideways.getPosition();


            double xInches = poseX2d.getX(DistanceUnit.INCH);
            double yInches = poseY2d.getY(DistanceUnit.INCH);
            double headingDeg = poseX2d.getHeading(AngleUnit.DEGREES); // LATER NEED TO COMBINE BOTH ODOS TO GET DEGREE

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


}
