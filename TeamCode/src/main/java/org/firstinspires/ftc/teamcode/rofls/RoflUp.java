package org.firstinspires.ftc.teamcode.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;

@Config
@TeleOp(name = "rofl up", group = "rofls")
public class RoflUp extends LinearOpMode {
    OuttakeController outtakeController = new OuttakeController();
    ArmController armController = new ArmController();

    @Override
    public void runOpMode() {

        MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        outtakeController.initialize(hardwareMap, false);
        armController.initialize(hardwareMap, false);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.b) {
                // take
                armController.setTake();
                outtakeController.setHandBackward();
                outtakeController.setWristTake();
            } else if (gamepad1.y) {
                // cybugs
                armController.setChamber();
                outtakeController.setHandForward();
                outtakeController.setWristCybugs();
            } else if (gamepad1.x) {
                // cyliis
                armController.setChamber();
                outtakeController.setHandBackward();
                outtakeController.setWristCyliis();
            } else if (gamepad1.a) {
                // basket
                armController.setBasket();
                outtakeController.setWristBasket();
                outtakeController.setHandForward();
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