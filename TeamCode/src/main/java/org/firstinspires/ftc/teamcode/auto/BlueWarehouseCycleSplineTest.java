package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.followers.RamseteFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.roadrunnerext.RamseteConstants;
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequence;

@Autonomous(name="Blue warehouse cycle spline test Roadrunner Path", group="Roadrunner Paths")
public class BlueWarehouseCycleSplineTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot drive = new Robot(this);

        // On start

        waitForStart();
        if(isStopRequested()) return;










        Pose2d startPose = new Pose2d(-11, 62.5, Math.toRadians(90));
        ElapsedTime timer = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .back(15)
                .build();


        TrajectorySequence Trajectory2 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .turn(Math.toRadians(-50))
                .build();

        TrajectorySequence Trajectory3 = drive.trajectorySequenceBuilder(Trajectory1.end())
                .setReversed(false)
                .splineTo(new Vector2d(30, 66), Math.toRadians(0))
                .forward(24)
                .build();

        TrajectorySequence Trajectory4 = drive.trajectorySequenceBuilder(Trajectory3.end())
                .back(24)
                .setReversed(true)
                .splineTo(new Vector2d(-20, 48), Math.toRadians(200)) // reversed
                .build();


        /*
        TrajectorySequence Trajectory5 = drive.trajectorySequenceBuilder(Trajectory4.end())

                .build();

        TrajectorySequence Trajectory6 = drive.trajectorySequenceBuilder(Trajectory5.end())
                .forward(15)
                .build();

         */



        // Run trajectory 1
        drive.followTrajectorySequence((Trajectory1));
        drive.followTrajectorySequence((Trajectory2));
        drive.followTrajectorySequence((Trajectory3));
        drive.followTrajectorySequence((Trajectory4));
        drive.followTrajectorySequence((Trajectory3));
        drive.followTrajectorySequence((Trajectory4));
        drive.followTrajectorySequence((Trajectory3));
        drive.followTrajectorySequence((Trajectory4));

    }
}

