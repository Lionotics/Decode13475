package org.firstinspires.ftc.teamcode.hardware;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.zyxOrientation;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class DriveTrain extends Subsystem {
    public static final DriveTrain INSTANCE = new DriveTrain();
    private DriveTrain() {
    }
    public static double maxSpeed = 1;
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private MotorEx[] motors;
    private IMU imu;

    public GoBildaPinpointDriver odometryForward;
    public GoBildaPinpointDriver odometrySideways;




    public void initialize() {
       // frontLeft = new MotorEx("frontLeft");
       // frontRight = new MotorEx("frontRight");
       // backLeft = new MotorEx("backLeft");
      //  backRight = new MotorEx("backRight");

        frontLeft = new MotorEx("backLeft");
        frontRight = new MotorEx("backRight");
        backLeft = new MotorEx("frontLeft");
        backRight = new MotorEx("frontRight");


        odometryForward = OpModeData.INSTANCE.getHardwareMap().get(GoBildaPinpointDriver.class, "odometryForward");
        odometrySideways = OpModeData.INSTANCE.getHardwareMap().get(GoBildaPinpointDriver.class, "odometrySideways");


       // frontLeft.reverse();
       // backLeft.reverse();

        motors = new MotorEx[]{frontLeft, frontRight, backLeft, backRight};

        initIMU(OpModeData.INSTANCE.getHardwareMap());
    }



    public void initIMU(HardwareMap hwMap) {
        // Retrieve the IMU from the hardware map
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();


        odometryForward.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometryForward.resetPosAndIMU();

        odometrySideways.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometrySideways.resetPosAndIMU();
    }

    public Command Drive(GamepadEx gamepad, boolean robotOreinted) {
        return new MecanumDriverControlled(motors, gamepad, robotOreinted, imu);
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
}