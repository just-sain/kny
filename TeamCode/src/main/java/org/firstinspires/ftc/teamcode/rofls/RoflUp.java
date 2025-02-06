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
        outtakeController.initialize(hardwareMap);
        armController.initialize(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.b) {
                // backward
                armController.setBackward();
                outtakeController.setHandBackward();
                outtakeController.setWristBackward();
            } else if (gamepad1.y) {
                // hit
                armController.setHit();
                outtakeController.setHandForward();
                outtakeController.setWristHit();
            } else if (gamepad1.x) {
                armController.setForward();
                outtakeController.setHandForward();
                outtakeController.setWristForward();
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