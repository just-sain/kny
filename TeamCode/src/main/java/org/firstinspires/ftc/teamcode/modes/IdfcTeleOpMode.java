package org.firstinspires.ftc.teamcode.modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.controllers.BaseController;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.controllers.Params;

@TeleOp(name = "idfc teleop", group = "prod")
public class IdfcTeleOpMode extends LinearOpMode {

    // subsystems
    private final BaseController baseController = new BaseController();
    private final LiftController liftController = new LiftController();
    private final ArmController armController = new ArmController();
    private final IntakeController intakeController = new IntakeController();
    private final OuttakeController outtakeController = new OuttakeController();

    @Override
    public void runOpMode() {

        // initializing
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // subsystems
        baseController.initialize(hardwareMap);
        liftController.initialize(hardwareMap);
        armController.initialize(hardwareMap);
        intakeController.initialize(hardwareMap);
        outtakeController.initialize(hardwareMap);

        // waiting for start
        telemetry.addData("status", "ready for start");
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
                intakeController.setWristUp();
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

            // --- intake ---
            // -- intake wrist
            if (gamepad2.a) {
                intakeController.setWristTake();
            } else if (gamepad2.b) {
                intakeController.setWristUp();
            } else if (gamepad2.y) {
                intakeController.setWristHome();
            }

            // -- intake hand --
            if (Math.abs(gamepad2.left_stick_x) > 0.05) {
                intakeController.setHandCustom(gamepad2.left_stick_x);
            } else {
                intakeController.setHandNeutral();
            }

            // -- intake claw --
            if (gamepad2.left_bumper) {
                // open intake claw
                intakeController.setClawOpen();
            } else if(gamepad2.right_bumper) {
                // close intake claw
                intakeController.setClawClose();
            }

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
            telemetry.addData("status", "running");

            telemetry.addLine("--- BASE ---");
            baseController.showTelemetry(telemetry);

            telemetry.addLine("--- LIFT ---");
            liftController.showLogs(telemetry);

            telemetry.addLine("--- ARM ---");
            armController.showLogs(telemetry);

            telemetry.addLine("--- INTAKE ---");
            intakeController.showLogs(telemetry);

            telemetry.addLine("--- OUTTAKE ---");
            outtakeController.showLogs(telemetry);

            telemetry.update();
        }
    }

}