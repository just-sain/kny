package org.firstinspires.ftc.teamcode.rofls;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.controllers.Params;

@TeleOp(name = "out lift", group = "rofls")
public class OutLift extends LinearOpMode {

    LiftController liftController = new LiftController();
    OuttakeController outtakeController = new OuttakeController();

    @Override
    public void runOpMode() {

        liftController.initialize(hardwareMap);
        outtakeController.initialize(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {

            // claw
            if (gamepad1.left_bumper) {
                outtakeController.setClawOpen();
            } else if (gamepad1.right_bumper) {
                outtakeController.setClawClose();
            }

//            if (gamepad1.a) {
//                outtakeController.setHandHorizontal();
//            } else if (gamepad1.y) {
//                outtakeController.setHandHit();
//            } else if (gamepad1.x) {
//                outtakeController.setHandHide();
//            }

            if (gamepad1.dpad_up) {
                liftController.setTargetPosition(Params.LiftParams.Position.CHAMBER);
            } else if (gamepad1.dpad_down) {
                liftController.setTargetPosition(Params.LiftParams.Position.HOME);
            } else if (gamepad1.dpad_left) {
                liftController.setTargetPosition(Params.LiftParams.Position.HIT);
            } else if (gamepad1.dpad_right) {
                liftController.setTargetPosition(Params.LiftParams.Position.LOW_BASKET);
            }

            liftController.periodic();

            outtakeController.showLogs(telemetry);
            liftController.showLogs(telemetry);
        }
    }
}
