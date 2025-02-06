package org.firstinspires.ftc.teamcode.controllers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmController {
    // motors
    private Servo lArm, rArm;

    public static double HOME_POS = 0;
    public static double FORWARD_POS = 0.1;
    public static double BACKWARD_POS = 1;
    public static double BASKET_POS = 0.65;
    public static double HIT_POS = 0.1;


    // initialize
    public void initialize(HardwareMap hardwareMap) {
        // getting from hardware map
        lArm = hardwareMap.get(Servo.class, "l-arm");
        rArm = hardwareMap.get(Servo.class, "r-arm");
        // setting direction
        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);
        // scale range
        lArm.scaleRange(0, 1);
        rArm.scaleRange(0, 1);
        // set default pos
        lArm.setPosition(HOME_POS);
        rArm.setPosition(HOME_POS);
    }

    // position
    public void setHome() {lArm.setPosition(HOME_POS); rArm.setPosition(HOME_POS);}
    public void setForward() {lArm.setPosition(FORWARD_POS); rArm.setPosition(FORWARD_POS);}
    public void setBackward() {lArm.setPosition(BACKWARD_POS); rArm.setPosition(BACKWARD_POS);}
    public void setBasket() {lArm.setPosition(BASKET_POS); rArm.setPosition(BASKET_POS);}
    public void setHit() {lArm.setPosition(HIT_POS); rArm.setPosition(HIT_POS);}

    // logs for debugging
    public void showLogs(Telemetry telemetry) {
        telemetry.addData("l arm", lArm.getPosition());
        telemetry.addData("r arm", rArm.getPosition());
    }
}
