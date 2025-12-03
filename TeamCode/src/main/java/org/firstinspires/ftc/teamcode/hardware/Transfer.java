package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

@Config
public class Transfer extends Subsystem {
    public Servo liftRight;
    public Servo liftLeft;

    public Servo protector;

    public  static double liftRightPosition1 = 0;

    public  static double liftRightPosition2 = 1;

    public  static double liftLeftPosition1 = 1;
    public  static double liftLeftPosition2 = 0;


    public  static double protectorPosition1 = 0.63;
    public  static double protectorPosition2 = 0.97;

    public  static  double protectorDelaySeconds = 0.4;

    public  static  double liftDelaySeconds = 0.4;

    public static double rotatorStep = 0.01;


    private  Transfer() {}

    public static Transfer INSTANCE = new Transfer();




    public void initialize() {
        liftRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "liftRight");
        liftLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "liftLeft");
        protector = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "Protector");

        liftRight.setPosition(liftRightPosition1);
        liftLeft.setPosition(liftLeftPosition1);
        protector.setPosition(protectorPosition1);
    }

    public InstantCommand goToDefaultPosition() {
        return new InstantCommand(()-> {
            liftRight.setPosition(liftRightPosition1);
            liftLeft.setPosition(liftLeftPosition1);
            protector.setPosition(protectorPosition1);
        });
    }

    public Command transferBall() {
        // protector moves, wait, then kicker moves
        return new SequentialGroup(
                new InstantCommand(() -> protector.setPosition(protectorPosition2)),
                new Delay(protectorDelaySeconds),
                new InstantCommand(() -> liftRight.setPosition(liftRightPosition2)),
                new InstantCommand(() -> liftLeft.setPosition(liftLeftPosition2)),

                new Delay(liftDelaySeconds),
                goToDefaultPosition()
        );
    }

    public InstantCommand rotateUp() {
        return new InstantCommand(()-> {
            protector.setPosition( protector.getPosition() + rotatorStep );
        });
    }

    public InstantCommand rotateDown() {
        return new InstantCommand(()-> {
            protector.setPosition( protector.getPosition() - rotatorStep );
        });
    }



}
