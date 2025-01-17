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
@Autonomous(name = "HALF_AUTO_LEFT_TEST", group = "Autonomous")
public class AutoCoyote3 extends LinearOpMode {

    private boolean prevup = false;
    private boolean prevdown = false;
    private double t = 0;


    public void runOpMode() {


        while (!gamepad1.y && !opModeIsActive()) {

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


        Pose2d initialPose = new Pose2d(24, 63, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        DependencyOp.Claw claw = new DependencyOp.Claw(hardwareMap);
        DependencyOp.Arm arm = new DependencyOp.Arm(hardwareMap);
        DependencyOp.Worm worm = new DependencyOp.Worm(hardwareMap);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0,54), Math.toRadians(0));

        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, 29));

        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(42.5,47), Math.toRadians(-90));

        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48,48), Math.toRadians(45));

        Action Traj1;
        Action Traj2;
        Action Traj3;
        Action Traj4;

        Traj1 = traj1.build();
        Traj2 = traj2.build();
        Traj3 = traj3.build();
        Traj4 = traj4.build();


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
                        Traj2,
                        arm.armDown(),
                        claw.openClaw(),
                        new ParallelAction(
                                Traj3,
                                worm.wormDownSample(),
                                arm.armGrabSample()
                        ),
                        claw.closeClaw(),
                        new SleepAction(1),
                        worm.wormUp(),
                        arm.armDropSample(),
                        Traj4
                )
        );
    }
}