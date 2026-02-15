package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.autoCommands;
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
        super.onInit();
    }

    @Override
    protected Pose getStartPose() {
        return new Pose(21, 123, Math.toRadians(angleToFaceGoal));
    }

    @Override
    protected void buildPaths() {
        // Your existing Path1/Path2 logic moved here
        pathToWall = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20, 121),
                        new Pose(35, 111)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToFaceGoal), Math.toRadians(angleToFaceGoal))
                .build();

        pathBack = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(35, 111),
                        new Pose(45, 65)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                    follower.turnTo(Math.toRadians(180));
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
