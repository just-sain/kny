package org.firstinspires.ftc.teamcode.rofls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.controllers.ExtendController;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;

@TeleOp(name = "rofl pass", group = "rofls")
public class RoflPass extends LinearOpMode {

    LiftController liftController = new LiftController();
    OuttakeController outtakeController = new OuttakeController();
    IntakeController intakeController = new IntakeController();
    ArmController armController = new ArmController();
    ExtendController extendController = new ExtendController();

    @Override
    public void runOpMode() throws InterruptedException {

        MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        outtakeController.initialize(hardwareMap, false);
        extendController.initialize(hardwareMap);
        intakeController.initialize(hardwareMap);
        liftController.initialize(hardwareMap);
        armController.initialize(hardwareMap, false);


        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.left_bumper) {
                outtakeController.setClawOpen();
            } else if (gamepad1.right_bumper) {
                outtakeController.setClawClose();
            }

            if (gamepad1.right_trigger > 0.05) {
                intakeController.setClawClose();
            } else if (gamepad1.left_trigger > 0.05) {
                intakeController.setClawOpen();
            }

            if (gamepad1.a) {
                armController.setPass();
                outtakeController.setHandForward();
                outtakeController.setWristPass();
                liftController.setTargetPosition(LiftController.Position.PASS);
            } else if (gamepad1.b) {
                intakeController.setWristHome();
            } else if (gamepad1.y) {
                liftController.setTargetPosition(LiftController.Position.BASKET);
                outtakeController.setWristBasket();
                outtakeController.setHandForward();
                armController.setBasket();
            }

            extendController.periodic();
            liftController.periodic();
        }

        // telemetry
        outtakeController.showLogs(telemetry);
        armController.showLogs(telemetry);
        liftController.showLogs(telemetry);
        intakeController.showLogs(telemetry);

        telemetry.update();

    }
}
