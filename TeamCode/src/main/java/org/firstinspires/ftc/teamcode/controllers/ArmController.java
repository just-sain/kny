package org.firstinspires.ftc.teamcode.controllers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmController {
    // motors
    private DcMotorEx motor;
    // k of pid
    public static double kP = 0.03, kI = 0.0007, kD = 0.00075, kF = 0.175;
    // target
    public static int target = 0;
    // ticks in degree
    private final double ticks_in_degree = 537.7 / 180.0;
    // pid controller
    private PIDController controller;

    // arm positions
    public enum Position {
        HOME(0),
        FORWARD(90),
        BASKET(230),
        BACKWARD(350);

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
        motor = hardwareMap.get(DcMotorEx.class, "arm");
        // setting direction
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        // setting zero power behavior to brake
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // stop and reset encoder on the left arm motor
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // running two arm motors without encoder
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        target = ArmController.Position.HOME.getPos();

        controller = new PIDController(kP, kI, kD);
    }

    // set target pos
    public void setTargetPosition(ArmController.Position position) {
        target = position.getPos();
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

        target = ArmController.Position.HOME.getPos();
    }

    // periodic
    public void periodic() {
        controller.setPID(kP, kI, kD);
        int armPos = motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * kF;

        double power = pid + ff;

        motor.setPower(power);
    }

    // logs for debugging
    public void showLogs(Telemetry telemetry) {
        telemetry.addData("arm Power", motor.getPower());
        telemetry.addData("arm Position", motor.getCurrentPosition());
        telemetry.addData("arm Target", target);
    }
}
