package org.firstinspires.ftc.teamcode.controllers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class OuttakeController {
    // wrist
    private Servo wrist;
    // wrist positions
    public static double WRIST_CYLIIS_POS = 0.75;
    public static double WRIST_CYBUGS_POS = 0.3;
    public static double WRIST_TAKE_POS   = 0.61;
    public static double WRIST_PASS_POS   = 0.28;
    public static double WRIST_BASKET_POS = 0.45;

    // hand
    private Servo hand;
    // hand positions
    public static double HAND_FORWARD_POS  = 0.84;
    public static double HAND_BACKWARD_POS = 0.08;

    // claw
    private Servo claw;
    // claw positions
    public static double CLAW_OPEN_POS = 0.6;
    public static double CLAW_CLOSE_POS = 0.8;

    // initialize
    public void initialize(HardwareMap hardwareMap, boolean isAuto) {
        // isAuto is very important, when auto mode
        // all servos go to the chamber pos, on
        // teleop mode everything is ready to take

        // wrist
        wrist = hardwareMap.get(Servo.class, "o-wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        wrist.scaleRange(0, 1);
        wrist.setPosition(isAuto ? WRIST_CYBUGS_POS : WRIST_TAKE_POS);

        // hand
        hand = hardwareMap.get(Servo.class, "o-hand");
        hand.setDirection(Servo.Direction.FORWARD);
        hand.scaleRange(0, 1);
        hand.setPosition(isAuto ? HAND_FORWARD_POS : HAND_BACKWARD_POS);

        // claw
        claw = hardwareMap.get(Servo.class, "o-claw");
        claw.scaleRange(0, 1);
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(CLAW_CLOSE_POS);
    }

    // wrist
    // wrist handles
    public void setWristCybugs() {
        wrist.setPosition(WRIST_CYBUGS_POS);
    }
    public void setWristCyliis() {
        wrist.setPosition(WRIST_CYLIIS_POS);
    }
    public void setWristTake() {
        wrist.setPosition(WRIST_TAKE_POS);
    }
    public void setWristPass() {
        wrist.setPosition(WRIST_PASS_POS);
    }
    public void setWristBasket() {
        wrist.setPosition(WRIST_BASKET_POS);
    }

    // hand handles
    public void setHandForward() {
        hand.setPosition(HAND_FORWARD_POS);
    }
    public void setHandBackward() {
        hand.setPosition(HAND_BACKWARD_POS);
    }

    // claw handles
    public void setClawOpen() {
        claw.setPosition(CLAW_OPEN_POS);
    }
    public void setClawClose() {
        claw.setPosition(CLAW_CLOSE_POS);
    }

    // claw actions
    public Action setClawOpenAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(CLAW_OPEN_POS);
                packet.put("Claw Position", CLAW_OPEN_POS);
                return false;
            }
        };
    }
    public Action setClawCloseAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(CLAW_CLOSE_POS);
                packet.put("Claw Position", CLAW_CLOSE_POS);
                return false;
            }
        };
    }

    // logs for debugging
    public void showLogs(Telemetry telemetry) {
        telemetry.addData("o wrist Position", wrist.getPosition());
        telemetry.addData("o hand Position", hand.getPosition());
        telemetry.addData("o claw Position", claw.getPosition());
    }
}