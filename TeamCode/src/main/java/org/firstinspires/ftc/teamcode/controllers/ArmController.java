package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmController {
    // motors
    private Servo lArm, rArm;

    public static double PASS_POS = 1;
    public static double CHAMBER_POS = 0.975;
    public static double BASKET_POS = 0.85;
    public static double TAKE_POS = 0.09;
    public static double HOME_POS = 0;


    // initialize
    public void initialize(HardwareMap hardwareMap) {
        // getting from hardware map
        lArm = hardwareMap.get(Servo.class, "l-arm");
        rArm = hardwareMap.get(Servo.class, "r-arm");
        // setting direction
        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);
        // set default pos
        lArm.setPosition(HOME_POS);
        rArm.setPosition(HOME_POS);
    }

    // position
    public void setPass() {lArm.setPosition(PASS_POS); rArm.setPosition(PASS_POS);}
    public void setChamber() {lArm.setPosition(CHAMBER_POS); rArm.setPosition(CHAMBER_POS);}
    public void setTake() {lArm.setPosition(TAKE_POS); rArm.setPosition(TAKE_POS);}
    public void setBasket() {lArm.setPosition(BASKET_POS); rArm.setPosition(BASKET_POS);}

    // logs for debugging
    public void showLogs(Telemetry telemetry) {
        telemetry.addData("l arm", lArm.getPosition());
        telemetry.addData("r arm", rArm.getPosition());
    }
}
