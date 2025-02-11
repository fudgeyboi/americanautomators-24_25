package org.firstinspires.ftc.teamcode.TeamOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "HALF_AUTO_LEFT", group = "Autonomous")
public class AutoCoyote2 extends LinearOpMode {

    private boolean prevup = false;
    private boolean prevdown = false;
    private double t = 0;


    public void runOpMode() {


        while (!gamepad1.y && !opModeIsActive() && !isStopRequested()) {

            if (gamepad1.dpad_down && !prevdown && (t > 0)) {
                t += -1;
            }

            if (gamepad1.dpad_up && !prevup) {
                t += 1;
            }

            prevup = gamepad1.dpad_up;
            prevdown = gamepad1.dpad_down;
            telemetry.addData("Timer", t);
            telemetry.update();
        }


        Pose2d initialPose = new Pose2d(24, 59, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        DependencyOp.Claw claw = new DependencyOp.Claw(hardwareMap);
        DependencyOp.Arm arm = new DependencyOp.Arm(hardwareMap);
        DependencyOp.Worm worm = new DependencyOp.Worm(hardwareMap);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0, 54), Math.toRadians(180));

        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, 26));

        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(45, 40), Math.toRadians(-60));

        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(51, 47, Math.toRadians(45)), Math.toRadians(45));

        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(52, 39, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder traj6 = traj5.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(51, 47, Math.toRadians(45)), Math.toRadians(90));

        TrajectoryActionBuilder traj7 = traj6.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24, 48, Math.toRadians(-90)), Math.toRadians(180));

        Action Traj1;
        Action Traj2;
        Action Traj3;
        Action Traj4;
        Action Traj5;
        Action Traj6;
        Action Traj7;

        Traj1 = traj1.build();
        Traj2 = traj2.build();
        Traj3 = traj3.build();
        Traj4 = traj4.build();
        Traj5 = traj5.build();
        Traj6 = traj6.build();
        Traj7 = traj7.build();


        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(t),
                        claw.closeClaw(),
                        new ParallelAction(
                                Traj1,
                                worm.wormUp(),
                                arm.armUp()
                        ),
                        new SleepAction(0.1),
                        Traj2,
                        arm.armDown(),
                        claw.openClaw(),
                        new ParallelAction(
                                Traj3,
                                worm.wormDown(),
                                arm.armGrabSample()
                        ),
                        new SleepAction(0.1),
                        claw.closeClaw(),
                        new SleepAction(0.2),
                        worm.wormUp(),
                        new SleepAction(0.1),
                        arm.armDropSample(),
                        Traj4,
                        claw.openClaw(),
                        new SleepAction(0.2),
                        Traj5,
                        new ParallelAction(
                                arm.armGrabSample(),
                                worm.wormDown()
                        ),
                        new SleepAction(0.1),
                        claw.closeClaw(),
                        new SleepAction(0.2),
                        worm.wormUp(),
                        new SleepAction(0.1),
                        arm.armDropSample(),
                        Traj6,
                        claw.openClaw(),
                        new SleepAction(0.2),
                        Traj7,
                        arm.armDown()
                )
        );
    }
}