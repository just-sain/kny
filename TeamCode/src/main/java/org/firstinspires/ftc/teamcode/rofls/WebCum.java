package org.firstinspires.ftc.teamcode.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.controllers.VisionController;

import java.util.Timer;
import java.util.TimerTask;

@Config
@TeleOp(name = "web cum", group = "rofls")
public class WebCum extends LinearOpMode {

    VisionController visionController = new VisionController();
    ArmController armController       = new ArmController();
    IntakeController intakeController = new IntakeController();

    MultipleTelemetry multipleTelemetry;

    public static boolean blueAlliance = true;
    int sampleAngle = 0;

    public boolean isAutoStart = false;
    boolean isTaking = false;

    // x - forward and backward
    // y - left and right

    @Override
    public void runOpMode() {
        visionController.initialize(hardwareMap, blueAlliance);
        armController.initialize(hardwareMap);
        intakeController.initialize(hardwareMap);

        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeController.setWristDetect();
        intakeController.setClawOpen();

        waitForStart();

        while(opModeIsActive()) {
            // MultipleTelemetry multipleTelemetry, boolean debug, boolean alliance
            VisionController.BlobInfo blobInfo = visionController.getBiggestBlobInfo(multipleTelemetry, false, true);

            if (blobInfo != null) {
                telemetry.addData("y", blobInfo.deltaY);
                telemetry.addData("x", blobInfo.deltaX);
                telemetry.addData("angle", blobInfo.angle);
            }

            // start auto
            if (gamepad1.a) {
                isAutoStart = true;
            }

            if (isAutoStart) {
                // if sample on arm pos
                if (blobInfo != null && blobInfo.deltaX <= 30) {
                    // if y going fuck do nothing
                    if (blobInfo.deltaY >= -30 && blobInfo.deltaY <= 75) {
                        armController.setTargetCustomPosition(armController.getCurrentPosition());

                        new Timer().schedule(new TimerTask() {
                            @Override
                            public void run() {
                                isAutoStart = false;
                                isTaking = true;
                            }
                        }, 200);
                    }
                } else {
                    // if no sample or low x pos go arm forward
                    armConfigure(true);
                }
            }

            if (isTaking) {
                if (blobInfo != null) {
                    sampleAngle = blobInfo.angle;

                    new Timer().schedule(new TimerTask() {
                        @Override
                        public void run() {
                            intakeController.setWristTake();

                            new Timer().schedule(new TimerTask() {
                                @Override
                                public void run() {
                                    intakeWristConfigure(sampleAngle);

                                    new Timer().schedule(new TimerTask() {
                                        @Override
                                        public void run() {
                                            intakeController.setClawClose();

                                            new Timer().schedule(new TimerTask() {
                                                @Override
                                                public void run() {
                                                    intakeController.setWristUp();
                                                }
                                            }, 300);

                                        }
                                    }, 600);

                                }
                            }, 600);

                        }
                    }, 500);
                }
            }

            // reset
            if (gamepad1.b) {
                sampleAngle = 0;
                isAutoStart = false;
                isTaking = false;
                reset();
            }

            armController.periodic();
            intakeController.showLogs(telemetry);

            telemetry.update();
        }
    }

    public void reset() {
        armController.setTargetPosition(ArmController.Position.HOME);
        intakeController.setWristDetect();
        intakeController.setClawOpen();
        intakeController.setHandNeutral();
    }

    public void armConfigure(boolean forward) {
        if (forward) {
            armController.setTargetCustomPosition(armController.getCurrentPosition() + 100);
        } else {
            armController.setTargetCustomPosition(armController.getCurrentPosition() - 50);
        }
    }

    public void intakeWristConfigure(int angle) {
        if (angle >= 110 && angle <= 140) {
            intakeController.setHandMaxDiagonal();
        } else if (angle >= 20 && angle <= 70) {
            intakeController.setHandMinDiagonal();
        } else if (angle >= 140 || angle <= 50) {
            intakeController.setHandMax();
        } else {
            intakeController.setHandNeutral();
        }
    }
}
