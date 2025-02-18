package org.firstinspires.ftc.teamcode.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;

@TeleOp(name = "rofl up", group = "rofls")
public class RoflUp extends LinearOpMode {
    OuttakeController outtakeController = new OuttakeController();
    ArmController armController = new ArmController();

    @Override
    public void runOpMode() {

        MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        outtakeController.initialize(hardwareMap, false);
        armController.initialize(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.dpad_up) {
                // cybugs
                armController.setChamber();
            } else if (gamepad1.dpad_left) {
                // basket
                armController.setBasket();
            } else if (gamepad1.dpad_down) {
                armController.setTake();
            } else if (gamepad1.dpad_right) {
                armController.setHome();
            }

            if (gamepad1.left_bumper) {
                outtakeController.setClawOpen();
            } else if (gamepad1.right_bumper) {
                outtakeController.setClawClose();
            }


            // telemetry
            outtakeController.showLogs(telemetry);

            telemetry.update();
        }
    }
}