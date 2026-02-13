package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.autoCommands;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Transfer;


@Config
@Autonomous(name = "BlueTouchingGoalAuto", group = "Autonomous")
public class BlueTouchingGoal extends AutoParent{
    // This auto's custom paths
    private PathChain pathToWall;
    private PathChain pathBack;

    public static double angleToFaceGoal = 325;

    @Override
    public  void onInit() {
        DriveTrain.INSTANCE.setGoalID(BLUE_TAG_ID);
    }

    @Override
    protected Pose getStartPose() {
        return new Pose(24, 131, Math.toRadians(angleToFaceGoal));
    }

    @Override
    protected void buildPaths() {
        // Your existing Path1/Path2 logic moved here
        pathToWall = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(24, 131),
                        new Pose(56, 85)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToFaceGoal), Math.toRadians(angleToFaceGoal))
                .build();

        pathBack = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56, 85),
                        new Pose(56, 135)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();
    }

    @Override
    protected int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathToWall);
                pathState = 2;
                break;


            case 2: {



                if (!follower.isBusy()) {
                    scoreCmd = autoCommands.autoScoreNoAim();
                    scoreCmd.invoke();
                    pathState = 3;
                }
                break;
            }

            case 3:
                if (scoreCmd != null && scoreCmd.isDone()) {
                    follower.turnTo(Math.toRadians(270));
                    scoreCmd = null;
                    turnStartMs = System.currentTimeMillis();
                    pathState = 4;
                } else {
                    // keep your existing behavior
                    Transfer.INSTANCE.updateWheelSpeedTick();
                }
                break;

            case 4:
                boolean timedOut = (System.currentTimeMillis() - turnStartMs) > turnMillisecondsWait;
                if (timedOut) {
                    follower.breakFollowing(); // lets the FSM advance even if Pedro still says “busy”
                    follower.followPath(pathBack);
                    pathState = 5;
                }
                break;

            default:
                // done
                break;
        }

        return pathState;
    }

    @Override
    protected void addSubclassTelemetry() {
        // optional extra debugging
    }
}
