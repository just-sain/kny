package org.firstinspires.ftc.teamcode.controllers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class StickController {
    // stick
    private Servo stick;
    // stick positions
    public static double STICK_USE_POS  = 0.04;
    public static double STICK_HIDE_POS = 0.68;

    public void initialize(HardwareMap hardwareMap) {
        stick = hardwareMap.get(Servo.class, "stick");
        stick.scaleRange(0, 1);
        stick.setDirection(Servo.Direction.FORWARD);
        stick.setPosition(STICK_HIDE_POS);
    }

    public void setStickUse() {
        stick.setPosition(STICK_USE_POS);
    }
    public void setStickHide() {
        stick.setPosition(STICK_HIDE_POS);
    }

    public Action setStickUseAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stick.setPosition(STICK_USE_POS);
                packet.put("claw pos", STICK_USE_POS);
                return false;
            }
        };
    }
    public Action setStickHideAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stick.setPosition(STICK_HIDE_POS);
                packet.put("claw pos", STICK_HIDE_POS);
                return false;
            }
        };
    }

    public void showLogs(Telemetry telemetry) {
        telemetry.addData("stick", stick.getPosition());
    }
}
