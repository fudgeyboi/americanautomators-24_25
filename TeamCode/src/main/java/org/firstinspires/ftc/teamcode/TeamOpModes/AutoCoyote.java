package org.firstinspires.ftc.teamcode.teleops;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class AutoCoyote extends LinearOpMode {
    public class Arm {
        private DcMotorEx arm;

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorEx.Direction.FORWARD);
        }

        public class ArmUp implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(1);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 4370) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }
        public Action armUp() {
            return new ArmUp();
        }
        public class ArmDown implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(-0.3);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 50) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }
        public Action armDown() {
            return new ArmDown();
        }
    }
    public class Claw {
        private Servo claw;
        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
            claw.setDirection(Servo.Direction.FORWARD);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);
                return false;
            }
            public Action openClaw() {
                return new OpenClaw();
            }
        }
    }
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-24, 60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-48, 48), Math.toRadians(-90));
        TrajectoryActionBuilder traj2 = traj1.fresh()
                .strafeToSplineHeading(new Vector2d(48,48), Math.toRadians(45));
        TrajectoryActionBuilder traj3 = traj2.fresh()
                .strafeToSplineHeading(new Vector2d(48,36), Math.toRadians(-90));
    }
}