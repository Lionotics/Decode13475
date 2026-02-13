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
@Autonomous(name = "RedTouchingWallAuto", group = "Autonomous")
public class RedTouchingWall extends AutoParent {

    // This auto's custom paths
    private PathChain pathToWall;
    private PathChain pathBack;

    public static double angleToFaceGoal = 225;

    @Override
    public  void onInit() {
        DriveTrain.INSTANCE.setGoalID(RED_TAG_ID);
    }


    @Override
    protected Pose getStartPose() {
        return new Pose(88, 8, Math.toRadians(90));
    }

    @Override
    protected void buildPaths() {
        // Your existing Path1/Path2 logic moved here
        pathToWall = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88.000, 8.000),
                        new Pose(88, 85)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        pathBack = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88, 85),
                        new Pose(88, 10)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(90))
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
                    target = AngleUnit.normalizeRadians(Math.toRadians(angleToFaceGoal));
                    follower.turnTo(target);
                    turnStartMs = System.currentTimeMillis();
                    pathState = 2;
                }
                break;

            case 2: {


                boolean timedOut = (System.currentTimeMillis() - turnStartMs) > 2000;

                if (timedOut) {
                    follower.breakFollowing(); // lets the FSM advance even if Pedro still says “busy”
                    reachedCaseTwo = true;

                    scoreCmd = autoCommands.autoScoreNoAim();
                    scoreCmd.invoke();

                    pathState = 3;
                }
                break;
            }



            case 3:
                if (scoreCmd != null && scoreCmd.isDone()) {
                    follower.turnTo(Math.toRadians(90));
                    scoreCmd = null;
                    pathState = 4;
                    turnStartMs = System.currentTimeMillis();


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
