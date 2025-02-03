// LiftPID adapted for RoadRunner
package org.firstinspires.ftc.teamcode.controllers;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LiftController {
    private DcMotorEx lift;
    // k of pid
    private double integralSum = 0;
    public static double kP = 0.012, kI = 0.00000002, kD = 0.000005;
    // needed
    public static double target = Params.LiftParams.Position.HOME.getPos();
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    // is manual or pid mode
    private boolean isManMode = false;

    // initialized
    public void initialize(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        // motor directions
        lift.setDirection(DcMotorEx.Direction.FORWARD);
        // zero power behavior
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // mode
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // default target position;
        target = Params.LiftParams.Position.HOME.getPos();
    }

    // custom pid controller
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
        lift.setPower(power);

        isManMode = true;
    }

    // set target to current position
    public void setTargetToCurrentPos() {
        target = lift.getCurrentPosition();
        isManMode = false;
    }

    // mode handles
    public boolean getIsManMode() {
        return isManMode;
    }

    // set target
    public void setTargetPosition(Params.LiftParams.Position position) {
        target = position.getPos();
    }

    // get current position
    public int getCurrentPosition() {
        return lift.getCurrentPosition();
    }

    // reset encoders
    public void resetEncoders() {
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // running two lift motors without encoder
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        target = Params.LiftParams.Position.HOME.getPos();
    }

    public Action resetEncodersAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                resetEncoders();

                return false;
            }
        };
    }

    // periodic
    public void periodic() {
        isManMode = false;

        double power = CustomPIDControl(target, lift.getCurrentPosition());
        lift.setPower(power);
    }

    // lift actions
    // do not use!!!
    public Action moveToPosition(Params.LiftParams.Position position) {
        // move to positions by pid
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                target = position.getPos();
                initialized = true;

                double power = CustomPIDControl(target, lift.getCurrentPosition());
                lift.setPower(power);

                double currentPosition = lift.getCurrentPosition();
                packet.put("Lift Position", currentPosition);
                packet.put("Lift Target", target);

                return Math.abs(target - currentPosition) > 15;
            }
        };
    }

    public Action setTargetPositionAction(Params.LiftParams.Position position) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                target = position.getPos();

                return false;
            }
        };
    }

    // logs for telemetry
    public void showLogs(Telemetry telemetry) {
        telemetry.addData("lift power", lift.getPower());
        telemetry.addData("lift pos", lift.getCurrentPosition());
        telemetry.addData("lift target", target);
    }
}