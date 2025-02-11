package org.firstinspires.ftc.teamcode.TeamOpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DependencyOp {
    public static class Worm {
        private DcMotorEx worm;

        public Worm(HardwareMap hardwareMap) {
            worm = hardwareMap.get(DcMotorEx.class, "worm");
            worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            worm.setDirection(DcMotorEx.Direction.FORWARD);
        }
        public class WormUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    worm.setPower(0.8);
                    initialized = true;
                }
                double pos = worm.getCurrentPosition();
                packet.put("wormpos", pos);
                new SleepAction(0.01);
                if (pos < 3800) {
                    return true;
                } else {
                    worm.setPower(0);
                    new SleepAction(0.2);
                    return false;
                }
            }
        }
        public Action wormUp() {
            return new WormUp();
        }
        public class WormDown implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    worm.setPower(-0.8);
                    initialized = true;
                }

                double pos = worm.getCurrentPosition();
                packet.put("wormpos", pos);
                new SleepAction(0.1);
                if (pos > 25) {
                    return true;
                } else {
                    worm.setPower(0);
                    new SleepAction(0.2);
                    return false;
                }
            }
        }
        public Action wormDown() {
            return new WormDown();
        }
    }

    public static class Arm {
        private DcMotorEx arm;

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorEx.Direction.FORWARD);
        }

        public class ArmUp implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(-1);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos > -1900) {
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
                    arm.setPower(1);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos < -650) {
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

        public class ArmGrabSample implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                double pos = arm.getCurrentPosition();

                if (!initialized) {
                    arm.setPower(-java.lang.Math.signum(pos + 1075));
                    initialized = true;
                }


                packet.put("armPos", pos);


                if ((pos > -1150) || (pos < -1200)) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }
        public Action armGrabSample() {
            return new ArmGrabSample();
        }

        public class ArmDropSample implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(-1);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos > -3900) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }
        public Action armDropSample() {
            return new ArmDropSample();
        }
    }
    public static class Claw {
        private Servo claw;
        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
            claw.setDirection(Servo.Direction.FORWARD);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0.25);
                return false;
            }
        }
        public Action openClaw() {
                return new OpenClaw();
        }
    }
}
