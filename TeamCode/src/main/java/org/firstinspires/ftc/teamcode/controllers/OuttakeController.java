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
    // claw
    private Servo claw;
    // claw positions
    public static double CLAW_OPEN_POS = 0.5;
    public static double CLAW_OPEN_TELEOP_POS = 0.55;
    public static double CLAW_CLOSE_POS = 0.3;

    // initialize
    public void initialize(HardwareMap hardwareMap, boolean isClawOpen) {
        // claw
        claw = hardwareMap.get(Servo.class, "o-claw");
        claw.scaleRange(0, 1);
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(isClawOpen ? CLAW_OPEN_TELEOP_POS : CLAW_CLOSE_POS);
    }

    // claw handles
    public void setClawOpen() {
        claw.setPosition(CLAW_OPEN_POS);
    }
    public void setClawOpenTeleop() {
        claw.setPosition(CLAW_OPEN_TELEOP_POS);
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
                packet.put("claw pos", CLAW_OPEN_POS);
                return false;
            }
        };
    }
    public Action setClawCloseAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(CLAW_CLOSE_POS);
                packet.put("claw pos", CLAW_CLOSE_POS);
                return false;
            }
        };
    }

    // logs for debugging
    public void showLogs(Telemetry telemetry) {
        telemetry.addData("o claw Position", claw.getPosition());
    }
}