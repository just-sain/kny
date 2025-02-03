package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.controllers.BaseController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.OArmController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;


// kiroshi it's not yaku KNY
@TeleOp(name = "Kny teleop", group = "")
public class KnyTeleop extends LinearOpMode {

    BaseController baseController = new BaseController();
    ArmController armController = new ArmController();
    OArmController oArmController = new OArmController();
    LiftController liftController = new LiftController();
    OuttakeController outtakeController = new OuttakeController();

    @Override
    public void runOpMode() {

        baseController.initialize(hardwareMap);
        armController.initialize(hardwareMap);
        oArmController.initialize(hardwareMap);
        liftController.initialize(hardwareMap);
        outtakeController.initialize(hardwareMap);


        waitForStart();

        while(opModeIsActive()) {

            // --- start keymap of robot ---
            // -- base --
            baseController.controlBySticks(
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x
            );

            // -- arm pid --
            if (gamepad1.a) {
                // home pos
                armController.setTargetPosition(ArmController.Position.HOME);
            } else if (gamepad1.b) {
                // middle
                armController.setTargetPosition(ArmController.Position.MIDDLE);
            } else if (gamepad1.y) {
                // long
                armController.setTargetPosition(ArmController.Position.LONG);
            }

            // -- arm pid mode switch --
            if (gamepad1.left_trigger > 0.05) {
                // man mode - down
                armController.setPower(-gamepad1.left_trigger);
            }
            if (gamepad1.right_trigger > 0.05) {
                // man mode - up
                armController.setPower(gamepad1.right_trigger);
            } else {
                if (armController.getIsManMode()) {
                    armController.setTargetToCurrentPos();
                } else {
                    armController.periodic();
                }
            }

            // -- resetting arm encoders, just in case --
            if (gamepad1.right_stick_button) {
                armController.resetEncoders();
            }

            // --- o arm ---
            oArmController.periodic();

            // --- lift ---
            // -- lift setting target position --
//            if (gamepad2.dpad_down) {
//                // take specimen
//                liftController.setTargetPosition(Params.LiftParams.Position.HOME);
//                outtakeController.setHandHorizontal();
//
//            } else if (gamepad2.dpad_up) {
//                // high chamber
//                liftController.setTargetPosition(Params.LiftParams.Position.HIT);
//                outtakeController.setHandHit();
//
//            } else if (gamepad2.dpad_right) {
//                liftController.setTargetPosition(Params.LiftParams.Position.CHAMBER);
//                outtakeController.setHandHorizontal();
//            } else if (gamepad2.dpad_left) {
//                outtakeController.setHandHide();
//            }

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

            // --- end of keymap of robot ---

            // --- telemetry ---
            telemetry.addLine("--- BASE ---");
            baseController.showTelemetry(telemetry);
            telemetry.addLine("--- ARM ---");
            armController.showLogs(telemetry);
            telemetry.addLine("--- OUTTAKE ARM ---");
            oArmController.showLogs(telemetry);
            telemetry.addLine("--- LIFT ---");
            liftController.showLogs(telemetry);
            telemetry.addLine("--- OUTTAKE ---");
            outtakeController.showLogs(telemetry);

            telemetry.update();
        }
    }
}
