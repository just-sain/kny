package org.firstinspires.ftc.teamcode.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.OuttakeController;

@Config
@TeleOp(name = "rofl outtake", group = "rofls")
public class RoflOuttake extends LinearOpMode {
    OuttakeController outtakeController = new OuttakeController();

    @Override
    public void runOpMode() {

        MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        outtakeController.initialize(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.b) {
                outtakeController.setHandBackward();
                outtakeController.setWristTake();
            } else if (gamepad1.y) {
                outtakeController.setHandForward();
                outtakeController.setWristCybugs();
            } else if (gamepad1.x) {
                outtakeController.setHandForward();
                outtakeController.setWristCyliis();
            }

            if (gamepad1.left_bumper) {
                outtakeController.setClawOpen();
            } else if (gamepad1.right_bumper) {
                outtakeController.setClawClose();
            }


            outtakeController.showLogs(telemetry);

            telemetry.update();
        }
    }
}