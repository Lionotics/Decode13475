package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

@TeleOp(name = "13475RedTeleop", group = "Teleop")
public class RedTeleop extends  TeleopParent{
    public RedTeleop() {
        super();
        DriveTrain.INSTANCE.setGoalID(RED_TAG_ID);

    }

    @Override
    public void onStartButtonPressed() {
        super.onStartButtonPressed();
    }

    @Override
    public  void onUpdate() {
        super.onUpdate();
    }

    @Override    public  void onStop() {
        super.onStop();
    }



}
