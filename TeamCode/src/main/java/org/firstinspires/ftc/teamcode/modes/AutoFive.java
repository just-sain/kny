package org.firstinspires.ftc.teamcode.modes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.PinpointDrive;
import org.firstinspires.ftc.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.controllers.Params;

@Disabled
@Autonomous(name = "auto five", group = "prod")
public class AutoFive extends LinearOpMode {

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
//
//        // init
//        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
//
//        // pinpoint
//        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose, liftController, armController);
//
//        // tab, base actions
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .strafeTo(new Vector2d(23.7, 0));
//
//        // tabs, pushed auto
//        TrajectoryActionBuilder tab2ForPush = tab1.endTrajectory().fresh()
//                .afterDisp(3, () -> {
//                    outtakeController.setHandHorizontal();
//                    liftController.setTargetPosition(Params.LiftParams.Position.HOME);
//                })
//                .strafeTo(new Vector2d(23, 0))
//                .strafeTo(new Vector2d(20, -31)) // Strafe right\
//                .strafeTo(new Vector2d(48, -31)) // Move forward
//                .strafeTo(new Vector2d(48, -40)) // Strafe right
//                .strafeTo(new Vector2d(17, -40)) // Move backward
//                .strafeTo(new Vector2d(26, -35.2));
//        TrajectoryActionBuilder tab3ForPush = tab2ForPush.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(60, -37.2, Math.toRadians(-178)), Math.toRadians(0))
//                .strafeTo(new Vector2d(60, -50))
//                .strafeTo(new Vector2d(19, -48))
//                .strafeTo(new Vector2d(24, -48));
//        // turned and moved
//        TrajectoryActionBuilder turned = tab3ForPush.endTrajectory().fresh()
//                .strafeTo(new Vector2d(33, -40))
//                .strafeTo(new Vector2d(10, -40));
//
//        TrajectoryActionBuilder turned1 = turned.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(31, 4, Math.toRadians(0)), 0);
//
//        TrajectoryActionBuilder turned3 = turned1.endTrajectory().fresh()
//                .afterDisp(6, () -> {
//                    outtakeController.setHandHorizontal();
//                    liftController.setTargetPosition(Params.LiftParams.Position.HOME);
//                })
//                .splineToLinearHeading(new Pose2d(10, -25, Math.toRadians(-180) ), 0);
//
//        TrajectoryActionBuilder turnedNew = turned3.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(33, 8, Math.toRadians(0)), 0);
//        TrajectoryActionBuilder turnedNew2 = turned3.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(33, 6, Math.toRadians(0)), 0);
//
//
//        TrajectoryActionBuilder turned4 = turned3.endTrajectory().fresh()
//                .turnTo(Math.toRadians(-170))
//                .strafeTo(new Vector2d(4, -23));
//        TrajectoryActionBuilder turned5 = turned4.endTrajectory().fresh()
//                .afterDisp(6, () -> {
//                    outtakeController.setHandHit();
//                    liftController.setTargetPosition(Params.LiftParams.Position.HOME);
//                })
//                .splineToLinearHeading(new Pose2d(-5, -55, Math.toRadians(0)), 0);
//
//        // custom actions
//        Action liftToChamber = liftController.setTargetPositionAction(Params.LiftParams.Position.CHAMBER);
//        Action liftToHit = liftController.setTargetPositionAction(Params.LiftParams.Position.HIT);
//        Action liftToHome = liftController.setTargetPositionAction(Params.LiftParams.Position.HOME);
//
//        // wait for start
//        waitForStart();
//
//        // run
//        Actions.runBlocking(
//                new SequentialAction(
//                        liftController.resetEncodersAction(),
//                        sleepAction(100),
//                        liftToHit,
//                        outtakeController.setHandHitAction(),
//                        tab1.build(),
//                        outtakeController.setClawOpenAction(),
//                        sleepAction(75),
//                        tab2ForPush.build(),
//                        tab3ForPush.build(),
//                        // chamber sample 2
//                        turned.build(),
//                        outtakeController.setClawCloseAction(),
//                        sleepAction(225),
//                        new ParallelAction(
//                                turned1.build(),
//                                liftToHit,
//                                outtakeController.setHandHitAction()
//                        ),
//                        outtakeController.setClawOpenAction(),
//                        sleepAction(75),
//
//                        // sample 3
//                        turned3.build(),
//                        turned4.build(),
//                        outtakeController.setClawCloseAction(),
//                        sleepAction(225),
//                        new ParallelAction(
//                                turnedNew.build(),
//                                liftToHit,
//                                outtakeController.setHandHitAction()
//                        ),
//                        outtakeController.setClawOpenAction(),
//                        sleepAction(75),
//
//                        // sample 4
//                        turned3.build(),
//                        turned4.build(),
//                        outtakeController.setClawCloseAction(),
//                        sleepAction(225),
//                        new ParallelAction(
//                                turnedNew2.build(),
//                                liftToHit,
//                                outtakeController.setHandHitAction()
//                        ),
//                        outtakeController.setClawOpenAction(),
//                        sleepAction(75),
//
//                        turned5.build()
//                )
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
