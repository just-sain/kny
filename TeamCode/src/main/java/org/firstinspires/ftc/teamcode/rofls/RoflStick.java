package org.firstinspires.ftc.teamcode.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.BaseController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.controllers.StickController;

@TeleOp(name = "rofl stick", group = "rofls")
public class RoflStick extends LinearOpMode {
    StickController stickController = new StickController();
    BaseController baseController = new BaseController();

    @Override
    public void runOpMode() {

        MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        stickController.initialize(hardwareMap);
        baseController.initialize(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {

            baseController.controlBySticks(
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x
            );

            if (gamepad1.left_bumper) {
                stickController.setStickHide();
            } else if (gamepad1.right_bumper) {
                stickController.setStickUse();
            }

            stickController.showLogs(telemetry);

            telemetry.update();
        }
    }
}