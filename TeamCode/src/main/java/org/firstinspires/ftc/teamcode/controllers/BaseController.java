package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class BaseController {

    private DcMotor lf; // left front
    private DcMotor rf; // right front
    private DcMotor lr; // left rear
    private DcMotor rr; // right rear

    public void initialize(HardwareMap hardwareMap) {
        // init
        lf = hardwareMap.get(DcMotor.class, "lf"); // 0
        rf = hardwareMap.get(DcMotor.class, "rf"); // 1
        lr = hardwareMap.get(DcMotor.class, "lr"); // 2
        rr = hardwareMap.get(DcMotor.class, "rr"); // 3

        // setting direction
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lr.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.FORWARD);

        // setting the motor 0 mode power to be brake as it actively stops the robot
        // and doesn't rely on the surface to slow down once the robot power is set to 0
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // setting motors run mode, will help with autonomous
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // setting 0 power to the motors wheel
        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
    }

    // non encoders
    public void controlBySticks(double left_stick_y, double right_stick_x, double left_stick_x) {
        double speed  = -left_stick_y;
        double strafe = left_stick_x;
        double turn   = right_stick_x;

        lf.setPower(speed + turn + strafe);
        rf.setPower(speed - turn - strafe);
        lr.setPower(speed + turn - strafe);
        rr.setPower(speed - turn + strafe);
    }

    public void setMotorsPower(double power) {
        lf.setPower(power);
        rf.setPower(power);
        lr.setPower(power);
        rr.setPower(power);
    }

    // custom
    public void setCustomMotorsPower(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        lf.setPower(leftFrontPower);
        rf.setPower(rightFrontPower);
        lr.setPower(leftRearPower);
        rr.setPower(rightRearPower);
    }

    public void setCustomSidesMotorsPower(double leftSidePower, double rightSidePower) {
        lf.setPower(leftSidePower);
        rf.setPower(rightSidePower);
        lr.setPower(leftSidePower);
        rr.setPower(rightSidePower);
    }

    // showing telemetry
    public void showTelemetry(Telemetry telemetry) {
        telemetry.addData("lf power:", lf.getPower());
        telemetry.addData("rf power:", rf.getPower());
        telemetry.addData("lr power:", lr.getPower());
        telemetry.addData("rr power:", rr.getPower());

        telemetry.addData("lf encoder:", lf.getCurrentPosition());
        telemetry.addData("rf encoder:", rf.getCurrentPosition());
        telemetry.addData("lr encoder:", lr.getCurrentPosition());
        telemetry.addData("rr encoder:", rr.getCurrentPosition());
    }
}