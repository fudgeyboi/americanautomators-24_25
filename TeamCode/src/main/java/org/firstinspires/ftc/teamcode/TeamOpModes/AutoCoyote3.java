package org.firstinspires.ftc.teamcode.TeamOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "HALF_AUTO_LEFT", group = "Autonomous")
public class AutoCoyote3 extends LinearOpMode {
    public void runOpMode() {


        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "lift");


        Pose2d initialPose = new Pose2d(24, 61, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        DependencyOp.Claw claw = new DependencyOp.Claw(hardwareMap);
        DependencyOp.Arm arm = new DependencyOp.Arm(hardwareMap);
        DependencyOp.Worm worm = new DependencyOp.Worm(hardwareMap);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0,54), Math.toRadians(0));

        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, 30));

        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(36, 36), Math.toRadians(-90))
                .strafeTo(new Vector2d(36, 12))
                .splineToConstantHeading(new Vector2d(46, 12), Math.toRadians(90))
                .strafeTo(new Vector2d(46, 54))
                .strafeTo(new Vector2d(46, 12))
                .splineToConstantHeading(new Vector2d(56, 12), Math.toRadians(90))
                .strafeTo(new Vector2d(56, 54))
                .strafeTo(new Vector2d(56, 12))
                .splineToConstantHeading(new Vector2d(62, 12), Math.toRadians(90))
                .strafeTo(new Vector2d(62, 54));

        Action Traj1;
        Action Traj2;
        Action Traj3;

        Traj1 = traj1.build();
        Traj2 = traj2.build();
        Traj3 = traj3.build();


        Actions.runBlocking(claw.closeClaw());


        lift.setPower(0.7);


        while (true && !isStopRequested()) {
            if (lift.getCurrent(CurrentUnit.AMPS) > 3) {
                lift.setPower(0);
                break;
            }
        }


        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                Traj1,
                                worm.wormUp(),
                                arm.armUp()
                        ),
                        Traj2,
                        arm.armDown(),
                        new ParallelAction(
                                Traj3,
                                worm.wormDown()
                        )
                )
        );
    }
}