package org.firstinspires.ftc.teamcode.TeamOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    private DcMotorEx worm;
    private DcMotorEx arm;
    private DcMotor lift;
    private Servo claw;
    private Servo sweeper;
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(24, 48, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        worm = hardwareMap.get(DcMotorEx.class, "worm");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");
        claw = hardwareMap.get(Servo.class, "claw");
        sweeper = hardwareMap.get(Servo.class, "sweeper");
        DependencyOp.Worm Worm = new DependencyOp.Worm(hardwareMap);
        DependencyOp.Arm Arm = new DependencyOp.Arm(hardwareMap);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        worm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.back || gamepad2.back) {
                requestOpModeStop();
            }
            drive.updatePoseEstimate();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            ((gamepad1.left_stick_y * java.lang.Math.sin(drive.pose.heading.toDouble()))
                                    - (gamepad1.left_stick_x * java.lang.Math.cos(drive.pose.heading.toDouble()))) / 1.5,
                            ((gamepad1.left_stick_y * java.lang.Math.cos(drive.pose.heading.toDouble()))
                                    + (gamepad1.left_stick_x * java.lang.Math.sin(drive.pose.heading.toDouble()))) / 1.5
                    ),
                    -gamepad1.right_stick_x / 2
            ));
            if (gamepad2.a && (worm.getCurrentPosition() < 5000)) {
                worm.setPower(1);
            } else if (gamepad2.b) {
                worm.setPower(-1);
            } else {
                worm.setPower(0);
            }
            if (gamepad2.y) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            }
            if (gamepad2.x) {
                worm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                worm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                worm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            }
            if (worm.getCurrentPosition() > 3000) {
                if (arm.getCurrentPosition() < -3900) {
                    arm.setPower(java.lang.Math.max(0, gamepad2.left_stick_y));
                } else {
                    arm.setPower(gamepad2.left_stick_y);
                }
            } else {
                if (arm.getCurrentPosition() < -2170) {
                    arm.setPower(java.lang.Math.max(0, gamepad2.left_stick_y));
                } else {
                    arm.setPower(gamepad2.left_stick_y);
                }
            }
            sweeper.setPosition(gamepad1.right_trigger);
            if (gamepad2.dpad_left) {
                Actions.runBlocking(Worm.wormDown());
            }
            if (gamepad2.dpad_right) {
                Actions.runBlocking(Worm.wormUp());
            }
            if (gamepad2.dpad_up) {
                Actions.runBlocking(Arm.armUp());
            }
            if (gamepad2.dpad_down) {
                Actions.runBlocking(Arm.armDown());
            }
            if (gamepad1.y) {
                TrajectoryActionBuilder traj1 = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(51, 47), 45);

                Action Traj1;
                Traj1 = traj1.build();
                Actions.runBlocking(Traj1);
            }
            claw.setPosition(java.lang.Math.max(gamepad2.right_trigger / 5, gamepad2.left_trigger / 5));
            lift.setPower(gamepad2.right_stick_y);
            double armcurrent = arm.getCurrent(CurrentUnit.AMPS);
            double wormcurrent = worm.getCurrent(CurrentUnit.AMPS);
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.addData("Worm Current Draw: ", wormcurrent);
            telemetry.addData("Arm Current Draw: ", armcurrent);
            telemetry.addData("Worm Pos", worm.getCurrentPosition());
            telemetry.addData("Arm Pos", arm.getCurrentPosition());
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
    }
}
