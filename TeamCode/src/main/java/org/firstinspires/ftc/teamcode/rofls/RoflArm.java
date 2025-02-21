package org.firstinspires.ftc.teamcode.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.ArmController;

@TeleOp(name = "rofl arm", group = "rofls")
public class RoflArm extends LinearOpMode {
    ArmController armController = new ArmController();

    @Override
    public void runOpMode() {

        MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        armController.initialize(hardwareMap, true);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a) {
                armController.setHome();
            } else if (gamepad1.b) {
                armController.setTake();
            } else if (gamepad1.y) {
                armController.setChamber();
            } else if (gamepad1.x) {
                armController.setBasket();
            }

            armController.showLogs(telemetry);

            telemetry.update();
        }
    }
}
