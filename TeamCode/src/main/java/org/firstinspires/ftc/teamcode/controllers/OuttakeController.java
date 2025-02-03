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
    public static double WRIST_HORIZONTAL_POS   = 0;
    public static double WRIST_HIT_POS     = 0.5;

    // claw
    private Servo claw;
    // claw positions
    public static double CLAW_OPEN_POS = 0.38;
    public static double CLAW_CLOSE_POS = 0.725;

    // initialize
    public void initialize(HardwareMap hardwareMap) {
        // wrist
        wrist = hardwareMap.get(Servo.class, "o-wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        wrist.scaleRange(0, 1);
        wrist.setPosition(WRIST_HORIZONTAL_POS);

        // claw
        claw = hardwareMap.get(Servo.class, "o-claw");
        claw.scaleRange(0, 1);
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(CLAW_CLOSE_POS);
    }

    // wrist
    // wrist handles
    public void setWristHit() {
        wrist.setPosition(WRIST_HIT_POS);
    }
    public void setWristHorizontal() {
        wrist.setPosition(WRIST_HORIZONTAL_POS);
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
        telemetry.addData("wrist Position", wrist.getPosition());
        telemetry.addData("claw Position", claw.getPosition());
    }
}