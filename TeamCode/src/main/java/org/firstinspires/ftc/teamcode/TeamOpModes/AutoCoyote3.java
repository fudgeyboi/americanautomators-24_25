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
@Autonomous(name = "HALF_AUTO_RIGHT_TEST", group = "Autonomous")
public class AutoCoyote3 extends LinearOpMode {

    private boolean prevup = false;
    private boolean prevdown = false;
    private double t = 0;


    public void runOpMode() {




        Pose2d initialPose = new Pose2d(-24, 59, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        DependencyOp.Claw claw = new DependencyOp.Claw(hardwareMap);
        DependencyOp.Arm arm = new DependencyOp.Arm(hardwareMap);
        DependencyOp.Worm worm = new DependencyOp.Worm(hardwareMap);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0, 54), Math.toRadians(0));

        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, 26));

        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-36, 24, Math.toRadians(90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-36,12), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-48,12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-48,54), Math.toRadians(90));

        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-32, 51, Math.toRadians(135)), Math.toRadians(45));

        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(0, 54), Math.toRadians(0));

        TrajectoryActionBuilder traj6 = traj5.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, 26));

        TrajectoryActionBuilder traj7 = traj6.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-46, 24, Math.toRadians(90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-46,12), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-58,12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-58,54), Math.toRadians(90));

        Action Traj1 = traj1.build();
        Action Traj2 = traj2.build();
        Action Traj3 = traj3.build();
        Action Traj4 = traj4.build();
        Action Traj5 = traj5.build();
        Action Traj6 = traj6.build();
        Action Traj7 = traj7.build();


        while (!opModeIsActive() && !isStopRequested()) {

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
                        arm.armGrabSample(),
                        claw.openClaw(),
                        Traj3,
                        Traj4,
                        claw.closeClaw(),
                        Traj5,
                        new ParallelAction(
                                Traj6,
                                worm.wormUp(),
                                arm.armUp()
                        ),
                        new SleepAction(0.1),
                        Traj7,
                        arm.armGrabSample(),
                        claw.openClaw()
                )
        );
    }
}