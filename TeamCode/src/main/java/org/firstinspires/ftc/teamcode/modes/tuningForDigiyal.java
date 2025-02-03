package org.firstinspires.ftc.teamcode.modes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.PinpointDrive;
import org.firstinspires.ftc.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.controllers.Params;


@Config
@Autonomous(name = "ONE BASKET", group = "prod")
public class tuningForDigiyal extends LinearOpMode {

    public static double TEST = 18.5;

    // controllers
    OuttakeController outtakeController = new OuttakeController();
    LiftController liftController = new LiftController();
    ArmController armController = new ArmController();

    @Override
    public void runOpMode() throws InterruptedException {
//
//        // controllers
//        outtakeController.initialize(hardwareMap);
//        liftController.initialize(hardwareMap);
//        armController.initialize(hardwareMap);
//
//        // init
//        Pose2d initialPose = new Pose2d(0, 0, 0);
//
//        // pinpoint
//        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose, liftController, armController);
//
//
//        Vector2d toBasket = new Vector2d(TEST, 0);
//
//
//        Vector2d fromBasket = new Vector2d(10, 0);
//
//        Vector2d secondTo = new Vector2d(19.5, 0);
//
//        // trajectories
//        TrajectoryActionBuilder ToBasket = drive.actionBuilder(initialPose)
//                .afterDisp(0, () -> {
//                    outtakeController.setHandHide();
//                })
//                .strafeTo(toBasket)
//                .turn(Math.toRadians(20))
//                .strafeTo(secondTo);
//
//        TrajectoryActionBuilder FromBasket = drive.actionBuilder(initialPose)
//                .strafeTo(fromBasket);
//
//        // wait for start
//        waitForStart();
//
//        // run
//        Actions.runBlocking(
//                new SequentialAction(
//                        liftController.setTargetPositionAction(Params.LiftParams.Position.LOW_BASKET),
//                        sleepAction(1500),
//                        ToBasket.build(),
//                        sleepAction(1000),
//                        // set 1 low Basket
//                        outtakeController.setClawOpenAction(),
//                        sleepAction(200),
//                        FromBasket.build(),
//                        sleepAction(1000),
//                        liftController.setTargetPositionAction(Params.LiftParams.Position.HOME),
//                        sleepAction(1000)
//                        )
//        );
    }

    // sleep action, sometimes help
    public Action sleepAction(long milliseconds) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                for (int i = 0; i <= milliseconds; i += 25) {
                    liftController.periodic();
                    armController.periodic();

                    sleep(25);
                }
                return false;
            }
        };
    }
}
