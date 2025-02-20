// LiftPID adapted for RoadRunner
package org.firstinspires.ftc.teamcode.controllers;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LiftController {
    private DcMotorEx left, right;
    // k of pid
    private double integralSum = 0;
    public static double kP = 0.012, kI = 0.00000002, kD = 0.000005;
    // needed
    public static double target = Position.HOME.getPos();
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    // is manual or pid mode
    private boolean isManMode = false;

    public enum Position {
        // home
        HOME(0),
        // TAKE
        TAKE(50),
        // CYLIIS
        CYLIIS(1250),
        // hang
        HANG(2000),
        // basket
        BASKET(2300),
        // pass
        PASS(300),
        // max
        MAX(2500);

        Position(int pos) {
            this.position = pos;
        }

        private int position;

        public int getPos() {
            return position;
        }
    }

    // initialized
    public void initialize(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "l-lift");
        right = hardwareMap.get(DcMotorEx.class, "r-lift");


        // motor directions
        left.setDirection(DcMotorEx.Direction.FORWARD);
        right.setDirection(DcMotorEx.Direction.REVERSE);
        // zero power behavior
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // mode
        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // default target position;
        target = Position.HOME.getPos();
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
        left.setPower(power);
        right.setPower(power);

        isManMode = true;
    }

    // set target to current position
    public void setTargetToCurrentPos() {
        target = left.getCurrentPosition();
        isManMode = false;
    }

    // mode handles
    public boolean getIsManMode() {
        return isManMode;
    }

    // set target
    public void setTargetPosition(Position position) {
        target = position.getPos();
    }

    // get current position
    public int getCurrentPosition() {
        return left.getCurrentPosition();
    }

    // set target position action
    public Action setTargetPositionAction(Position position) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                target = position.getPos();

                return false;
            }
        };
    }


    // reset encoders
    public void resetEncoders() {
        left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // running two lift motors without encoder
        left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        target = Position.HOME.getPos();
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

        double power = CustomPIDControl(target, left.getCurrentPosition());
        left.setPower(power);
        right.setPower(power);
    }


    // logs for telemetry
    public void showLogs(Telemetry telemetry) {
        telemetry.addData("lift power", left.getPower());
        telemetry.addData("lift pos", left.getCurrentPosition());
        telemetry.addData("lift target", target);
    }
}