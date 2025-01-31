package org.firstinspires.ftc.teamcode.TeamOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "HALF_AUTO_RIGHT", group = "Autonomous")
public class AutoCoyote extends LinearOpMode {

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


        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");


        Pose2d initialPose = new Pose2d(-24, 62, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        DependencyOp.Claw claw = new DependencyOp.Claw(hardwareMap);
        DependencyOp.Arm arm = new DependencyOp.Arm(hardwareMap);
        DependencyOp.Worm worm = new DependencyOp.Worm(hardwareMap);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0,54), Math.toRadians(0))
                .strafeTo(new Vector2d(0, 30));

        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-36, 12), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-46, 12), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(-46, 56), Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-46, 12), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-56, 12), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(-56, 54), Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-56, 12), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-65, 12), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(-65, 54), Math.toRadians(90));


        Action Traj1;
        Action Traj2;


        Traj1 = traj1.build();
        Traj2 = traj2.build();


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
                        arm.armDown(),
                        new ParallelAction(
                                Traj2,
                                worm.wormDown()
                        )
                )
        );
    }
}