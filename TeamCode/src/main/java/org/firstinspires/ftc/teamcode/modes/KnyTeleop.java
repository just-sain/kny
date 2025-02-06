package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.controllers.BaseController;
import org.firstinspires.ftc.teamcode.controllers.ExtendController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;


// kiroshi it's not yaku KNY
@TeleOp(name = "Kny teleop", group = "modes")
public class KnyTeleop extends LinearOpMode {

    BaseController baseController = new BaseController();
    ExtendController extendController = new ExtendController();

    LiftController liftController = new LiftController();
    ArmController armController = new ArmController();
    OuttakeController outtakeController = new OuttakeController();

    @Override
    public void runOpMode() {

        baseController.initialize(hardwareMap);
        extendController.initialize(hardwareMap);

        liftController.initialize(hardwareMap);
        armController.initialize(hardwareMap);
        outtakeController.initialize(hardwareMap);

        // ftc dashboard debug
//        MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        while(opModeIsActive()) {

            // --- start keymap of robot ---
            // -- base --
            baseController.controlBySticks(
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x
            );

            // --- lift ---
            // -- lift setting target position --
            if (gamepad2.dpad_down) {
                // take specimen
                liftController.setTargetPosition(LiftController.Position.HOME);
            } else if (gamepad2.dpad_up) {
                // hit high chamber
                liftController.setTargetPosition(LiftController.Position.HIT);
            } else if (gamepad2.dpad_right) {
                // chamber - push
                liftController.setTargetPosition(LiftController.Position.CHAMBER);
            } else if (gamepad2.dpad_left) {
                // high basket
                liftController.setTargetPosition(LiftController.Position.BASKET);
            }

            // -- lift pid mode switch --
            if (gamepad2.left_trigger > 0.05) {
                // man mode - down
                liftController.setPower(-gamepad2.left_trigger);
            }
            if (gamepad2.right_trigger > 0.05) {
                // man mode - up
                liftController.setPower(gamepad2.right_trigger);
            } else {
                // periodic
                if (liftController.getIsManMode()) {
                    liftController.setTargetToCurrentPos();
                } else {
                    liftController.periodic();
                }
            }

            // -- resetting lift encoders, just in case --
            if (gamepad2.right_stick_button) {
                liftController.resetEncoders();
            }

            // --- outtake ---
            // -- outtake claw --
            if (gamepad1.left_bumper) {
                // open outtake claw
                outtakeController.setClawOpen();
            } else if (gamepad1.right_bumper) {
                // close outtake claw
                outtakeController.setClawClose();
            }

            // --- arm ---
            // -- arm --
            if (gamepad1.dpad_left) {
                armController.setBasket();
            } else if (gamepad1.dpad_up) {
                armController.setForward();
                outtakeController.setWristForward();
            } else if (gamepad1.dpad_down) {
                armController.setBackward();
            } else if (gamepad1.dpad_right) {
                armController.setHome();
            }


            // --- end of keymap of robot ---

            // --- telemetry ---
            telemetry.addLine("--- BASE ---");
            baseController.showTelemetry(telemetry);

            telemetry.addLine("--- EXTEND ---");
            extendController.showLogs(telemetry);

            telemetry.addLine("--- LIFT ---");
            liftController.showLogs(telemetry);

            telemetry.addLine("--- ARM ---");
            armController.showLogs(telemetry);

            telemetry.addLine("--- OUTTAKE ---");
            outtakeController.showLogs(telemetry);

            telemetry.update();
        }
    }
}
