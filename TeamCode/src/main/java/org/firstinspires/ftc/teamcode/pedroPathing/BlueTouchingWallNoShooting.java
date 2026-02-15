package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.autoCommands;
import org.firstinspires.ftc.teamcode.hardware.Transfer;


@Config
@Autonomous(name = "BlueTouchingWallNoShootingAuto", group = "Autonomous")
public class BlueTouchingWallNoShooting extends AutoParent{
    // This auto's custom paths
    private PathChain pathToWall;


    @Override
    public  void onInit() {
        super.onInit();
    }

    @Override
    protected Pose getStartPose() {
        return new Pose(56, 8, Math.toRadians(90));
    }

    @Override
    protected void buildPaths() {
        // Your existing Path1/Path2 logic moved here
        pathToWall = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 8.000),
                        new Pose(56.0, 30)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

    }

    @Override
    protected int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathToWall);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.turnTo(180);
                    pathState = 2;
                }
                break;
        }
        return pathState;
    }

    @Override
    protected void addSubclassTelemetry() {
        // optional extra debugging
    }
}
