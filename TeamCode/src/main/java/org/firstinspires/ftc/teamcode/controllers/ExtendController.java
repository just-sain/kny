package org.firstinspires.ftc.teamcode.controllers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ExtendController {
    // motors
    private DcMotorEx motor;
    // k of pid
    private double integralSum = 0;
    public static double kP = 0.006, kI = 0.0000003, kD = 0.00000005;
    // needed
    public static double target = Position.HOME.getPos();
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    // is manual or pid mode
    private boolean isManMode = false;

    // arm positions
    public enum Position {
        HOME(0),
        PASS(100),
        MIDDLE(900),
        LONG(1500);

        Position(int pos) {
            this.position = pos;
        }

        private int position;

        public int getPos() {
            return position;
        }
    }

    // initialize
    public void initialize(HardwareMap hardwareMap) {
        // getting from hardware map
        motor = hardwareMap.get(DcMotorEx.class, "extend");
        // setting direction
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        // setting zero power behavior to brake
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // stop and reset encoder on the left arm motor
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // running two arm motors without encoder
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        target = Position.HOME.getPos();
    }

    private double CustomPIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * kP) + (derivative * kD) + (integralSum * kI);
    }

    // manual mode
    public void setPower(double power) {
        motor.setPower(power);

        isManMode = true;
    }

    // set target to current position
    public void setTargetToCurrentPos() {
        target = motor.getCurrentPosition();
        isManMode = false;
    }

    // mode handles
    public boolean getIsManMode() {
        return isManMode;
    }

    // set target pos
    public void setTargetPosition(Position position) {
        target = position.getPos();
    }

    // set custom target pos
    public void setTargetCustomPosition(int position) {
        if (position + 50 > Position.LONG.getPos()) {
            target = Position.LONG.getPos();
        } else if (position - 50 < Position.HOME.getPos()) {
            target = Position.HOME.getPos();
        } else {
            target = position;
        }
    }

    // get current position
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    // reset encoders
    public void resetEncoders() {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // running two arm motors without encoder
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        target = Position.HOME.getPos();
    }

    // periodic
    public void periodic() {
        isManMode = false;

        double power = CustomPIDControl(target, motor.getCurrentPosition());
        motor.setPower(power);
    }

    // actions
    public Action moveToPositionAction(Position position) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    target = position.getPos();
                    initialized = true;
                }

                double power = CustomPIDControl(target, motor.getCurrentPosition());
                motor.setPower(power);

                double currentPosition = motor.getCurrentPosition();
                packet.put("Arm Position", currentPosition);
                packet.put("Arm Target", target);

                // Return true if the arm has not yet reached the target
                return Math.abs(target - currentPosition) > 10;
            }
        };
    }

    // logs for debugging
    public void showLogs(Telemetry telemetry) {
        telemetry.addData("extend Power", motor.getPower());
        telemetry.addData("extend Position", motor.getCurrentPosition());
        telemetry.addData("extend Target", target);
    }
}