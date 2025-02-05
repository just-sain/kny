package org.firstinspires.ftc.teamcode.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.ExtendController;

@TeleOp(name = "rofl extend", group = "rofls")
public class RoflExtend extends LinearOpMode {
    ExtendController extendController = new ExtendController();

    @Override
    public void runOpMode() {

        MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        extendController.initialize(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a) {
                extendController.setTargetPosition(ExtendController.Position.HOME);
            } else if (gamepad1.b) {
                extendController.setTargetPosition(ExtendController.Position.MIDDLE);
            } else if (gamepad1.y) {
                extendController.setTargetPosition(ExtendController.Position.LONG);
            }

            extendController.periodic();

            extendController.showLogs(telemetry);

            telemetry.update();
        }
    }
}
