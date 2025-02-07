package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.controllers.BaseController;
import org.firstinspires.ftc.teamcode.controllers.ExtendController;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;


// kiroshi it's not yaku KNY
@TeleOp(name = "Kny teleop", group = "modes")
public class KnyTeleop extends LinearOpMode {

    BaseController baseController = new BaseController();
    ExtendController extendController = new ExtendController();
    IntakeController intakeController = new IntakeController();

    LiftController liftController = new LiftController();
    ArmController armController = new ArmController();
    OuttakeController outtakeController = new OuttakeController();

    @Override
    public void runOpMode() {

        baseController.initialize(hardwareMap);
        extendController.initialize(hardwareMap);
        intakeController.initialize(hardwareMap);

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
                takeSpecimen();
            } else if (gamepad2.dpad_up) {
                cybugs();
            } else if (gamepad2.dpad_right) {
                cyliis();
            } else if (gamepad2.dpad_left) {
                highBasket();
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

            // -- resetting extend encoders, just in case --
            if (gamepad2.right_stick_button) {
                liftController.resetEncoders();
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

            // --- extend ---
            // -- extend setting target position --
            if (gamepad1.a) {
                extendController.setTargetPosition(ExtendController.Position.HOME);
                intakeController.setWristUp();
            } else if (gamepad1.b) {
                extendController.setTargetPosition(ExtendController.Position.MIDDLE);
            } else if (gamepad1.y) {
                extendController.setTargetPosition(ExtendController.Position.LONG);
            }

            // -- extend pid mode switch --
            if (gamepad1.left_trigger > 0.05) {
                // man mode - down
                extendController.setPower(-gamepad1.left_trigger);
            }
            if (gamepad1.right_trigger > 0.05) {
                // man mode - up
                extendController.setPower(gamepad1.right_trigger);
            } else {
                // periodic
                if (extendController.getIsManMode()) {
                    extendController.setTargetToCurrentPos();
                } else {
                    extendController.periodic();
                }
            }

            // -- resetting extend encoders, just in case --
            if (gamepad1.right_stick_button) {
                extendController.resetEncoders();
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

            telemetry.addLine("--- EXTEND ---");
            extendController.showLogs(telemetry);

            telemetry.addLine("--- INTAKE ---");
            intakeController.showLogs(telemetry);

            telemetry.addLine("--- LIFT ---");
            liftController.showLogs(telemetry);

            telemetry.addLine("--- ARM ---");
            armController.showLogs(telemetry);

            telemetry.addLine("--- OUTTAKE ---");
            outtakeController.showLogs(telemetry);

            telemetry.update();
        }
    }
    // cybugs
    public void cybugs() {
        // cybugs high chamber
        liftController.setTargetPosition(LiftController.Position.CYBUGS);

        armController.setChamber();
        outtakeController.setWristCybugs();
        outtakeController.setHandForward();
    }
    // cyliis
    public void cyliis() {
        // cyliis high chamber
        liftController.setTargetPosition(LiftController.Position.CYLIIS);

        armController.setChamber();
        outtakeController.setWristCyliis();
        outtakeController.setHandBackward();

    }

    // take
    public void takeSpecimen() {
        // take specimen
        liftController.setTargetPosition(LiftController.Position.HOME);

        armController.setTake();
        outtakeController.setWristTake();
        outtakeController.setHandBackward();
    }

    // high basket
    public void highBasket() {
        // high basket
        liftController.setTargetPosition(LiftController.Position.BASKET);

        armController.setBasket();
        outtakeController.setHandForward();
        outtakeController.setWristBasket();
    }

}
