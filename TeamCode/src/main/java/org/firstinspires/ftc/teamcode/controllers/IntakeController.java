package org.firstinspires.ftc.teamcode.controllers;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntakeController {
    // wrist
    private Servo wrist;
    // wrist positions
    public static double WRIST_HOME_POS   = 0;
    public static double WRIST_DETECT_POS = 0.55;
    public static double WRIST_UP_POS     = 0.45;
    public static double WRIST_TAKE_POS   = 0.89;

    // hand
    private Servo hand;
    // hand positions
    public static double HAND_MIN_POS     = 0;
    public static double HAND_NEUTRAL_POS = 0.53;
    public static double HAND_MAX_POS     = 1;

    // claw
    private Servo claw;
    // claw positions
    public static double CLAW_OPEN_POS  = 0.525;
    public static double CLAW_CLOSE_POS = 0.87;

    // initialize
    public void initialize(HardwareMap hardwareMap) {
        // wrist
        wrist = hardwareMap.get(Servo.class, "i-wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
        wrist.scaleRange(0, 1);
        wrist.setPosition(WRIST_UP_POS);

        // hand
        hand = hardwareMap.get(Servo.class, "i-hand");
        hand.setDirection(Servo.Direction.FORWARD);
        hand.scaleRange(0, 1);
        hand.setPosition(HAND_NEUTRAL_POS);

        // claw
        claw = hardwareMap.get(Servo.class, "i-claw");
        claw.scaleRange(0, 1);
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(CLAW_CLOSE_POS);
    }

    // wrist
    // wrist handles
    public void setWristTake() {
        wrist.setPosition(WRIST_TAKE_POS);
    }
    public void setWristUp() {
        wrist.setPosition(WRIST_UP_POS);
    }
    public void setWristHome() {
        wrist.setPosition(WRIST_HOME_POS);
    }
    public void setWristDetect() {
        wrist.setPosition(WRIST_DETECT_POS);
    }

    // wrist actions
        public Action setWristTakeAction() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    wrist.setPosition(WRIST_TAKE_POS);
                    return false;
                }
            };
        }
    public Action setWristUpAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_UP_POS);
                packet.put("Wrist Position", WRIST_UP_POS);
                return false;
            }
        };
    }
    public Action setWristHomeAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_HOME_POS);
                packet.put("Wrist Position", WRIST_HOME_POS);
                return false;
            }
        };
    }

    // hand handles
    public void setHandMin() {
        hand.setPosition(HAND_MIN_POS);
    }
    public void setHandNeutral() {
        hand.setPosition(HAND_NEUTRAL_POS);
    }
    public void setHandMax() {
        hand.setPosition(HAND_MAX_POS);
    }
    public void setHandCustom(double stickInput) {
        double pos = (-stickInput + 1) / 2;

        if (pos <= HAND_MIN_POS) pos = HAND_MIN_POS;
        if (pos >= HAND_MAX_POS) pos = HAND_MAX_POS;

        hand.setPosition(pos);
    }

    // hand actions
    public Action setHandNeutralAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hand.setPosition(HAND_NEUTRAL_POS);
                packet.put("Hand Position", HAND_NEUTRAL_POS);
                return false;
            }
        };
    }
    public Action setHandMaxPosAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hand.setPosition(HAND_MAX_POS);
                packet.put("Hand Position", HAND_MAX_POS);
                return false;
            }
        };
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
        telemetry.addData("hand Position", hand.getPosition());
        telemetry.addData("claw Position", claw.getPosition());
    }
}