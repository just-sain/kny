package org.firstinspires.ftc.teamcode.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.IntakeController;

@TeleOp(name = "rofl intake", group = "rofls")
public class RoflIntake extends LinearOpMode {

    IntakeController intakeController = new IntakeController();

    MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() throws InterruptedException {

        // init
        intakeController.initialize(hardwareMap, false);

        // wait for start
        waitForStart();

        // run
        while(opModeIsActive()) {
            // wrist
            if (gamepad1.a) {
                intakeController.setWristTake();
            } else if (gamepad1.b) {
                intakeController.setWristUp();
            } else if (gamepad1.y) {
                intakeController.setWristHome();
            }

            // hand
            if (gamepad1.dpad_up) {
                intakeController.setHandMin();
            } else if (gamepad1.dpad_down) {
                intakeController.setHandMax();
            } else if (gamepad1.dpad_right) {
                intakeController.setHandNeutral();
            }

            // claw
            if (gamepad1.left_bumper) {
                intakeController.setClawOpen();
            } else if (gamepad1.right_bumper) {
                intakeController.setClawClose();
            }

            // logs
            intakeController.showLogs(telemetry);

            // update
            telemetry.update();

        }

    }
}
