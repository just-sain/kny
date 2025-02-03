package org.firstinspires.ftc.teamcode.modes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.PinpointDrive;
import org.firstinspires.ftc.teamcode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.controllers.Params;

import java.util.Arrays;


@Disabled
@Autonomous(name = "auto five 100%", group = "prod")
public class tuningForTheWeek extends LinearOpMode {

    // Максимально высокая скорость
    VelConstraint maxVelConstraint = new TranslationalVelConstraint(1000);
    AccelConstraint maxAccelConstraint = new ProfileAccelConstraint(-200.0, 400.0);

    // constant
    public static double TURN_180 = 230;
    public static double TURN_NEGATIVE_180 = -230;

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
//        // To Submarine
//        Vector2d submarineFirst = new Vector2d(26, 0);
//
//        // From Submarine
//        Vector2d fromSubmarine = new Vector2d(14, -26);
//
//        // push three samples
//        Vector2d forwardSampleFirst = new Vector2d(43, -42); // strafe to First Sample
//        Vector2d strafeSampleFirst = new Vector2d(43, -45); // strafe to first sample
//        Vector2d backSampleFirst = new Vector2d(18, -45); // go to Human ZONE first sample
//        Pose2d forwardSampleSecond  = new Pose2d(43, -45, 175); // forward go to the second sample
//        Vector2d strafeSampleSecond  = new Vector2d(43, -56); // strafe to second sample
//        Vector2d backSampleSecond  = new Vector2d(18, -56); // go to Human ZONE second sample
////        Pose2d forwardSampleThird  = new Pose2d(45.5, -54, Math.toRadians(175)); // forward go to the third sample
////        Vector2d strafeSampleThird  = new Vector2d(45.5, -62); // strafe to third sample
////        Vector2d backSampleThird  = new Vector2d(12, -62); // go to Human ZONE third sample
//
//        // BACK
//        Vector2d backGoSpecimenThird = new Vector2d(23, -50);
//
//        // chamber sample
//        Vector2d humanPlayerSecondSample = new Vector2d(3.5, -40);
//        Pose2d chamberSecond = new Pose2d(31, -4, Math.toRadians(0));
//        Vector2d chamberSecondStrafe = new Vector2d(31, -4);
//        Vector2d BeforeTurn = new Vector2d(5, -40);
//        Vector2d backgoSpecimenSecond = new Vector2d(30, -4);
//
//        Pose2d humanPlayerThirdSample = new Pose2d(4, -45, Math.toRadians(-173));
//        Vector2d humanPlayerThirdSampleStrafe = new Vector2d(4, -45);
//        Pose2d chamberThird = new Pose2d(31.5, 6, Math.toRadians(0));
//        Vector2d chamberThirdStrafe = new Vector2d(31.5, 6);
//        Vector2d backgoSpecimenThird = new Vector2d(29.5, 6);
//
//
//        Pose2d humanPlayerFourthSample = new Pose2d(6, -40, Math.toRadians(-176));
//        Vector2d humanPlayerFourthSampleStrafe = new Vector2d(6, -40);
//        Pose2d chamberFourth = new Pose2d(31.5, 4, Math.toRadians(0));
//        Vector2d chamberFourthStrafe = new Vector2d(31.5, 4);
//        Vector2d backGoSpecimenFourth = new Vector2d(29.5, 4);
//
//        Pose2d humanPlayerFifthSample = new Pose2d(4, -40, Math.toRadians(-173));
//        Pose2d chamberFifth = new Pose2d(31.5,4, Math.toRadians(0));
//        Vector2d backGoSpecimenFifth = new Vector2d(29.5, 4);
//
//
//        // parking
//        Vector2d parking = new Vector2d(3, -30);
//
//        // trajectories
//        TrajectoryActionBuilder SubmarineFirst = drive.actionBuilder(initialPose)
//                .afterDisp(0, () -> {
//                    liftController.setTargetPosition(Params.LiftParams.Position.HIT);
//                    outtakeController.setHandHit();
//                })
//                .strafeTo(submarineFirst, maxVelConstraint);
//
//        // from Submarine
//        TrajectoryActionBuilder FromSubmarine = SubmarineFirst.endTrajectory().fresh()
//                .afterDisp(0, () -> {
//                    outtakeController.setHandHorizontal();
//                    liftController.setTargetPosition(Params.LiftParams.Position.HOME);
//                })
//                .strafeTo(fromSubmarine, maxVelConstraint);
//
//        // pushed samples
//        TrajectoryActionBuilder PushedSamples = FromSubmarine.endTrajectory().fresh()
//                .strafeTo(forwardSampleFirst, maxVelConstraint)
//                .strafeTo(strafeSampleFirst, maxVelConstraint)
//                .strafeTo(backSampleFirst, maxVelConstraint)
//                .splineToLinearHeading(forwardSampleSecond, Math.toRadians(0), maxVelConstraint)
//                .strafeTo(strafeSampleSecond, maxVelConstraint)
//                .strafeTo(backSampleSecond, maxVelConstraint);
////
////        // go a second specimen
////        TrajectoryActionBuilder GoSpecimenSecond = PushedSamples.endTrajectory().fresh()
////                .strafeTo(humanPlayerSecondSample, maxVelConstraint);
////
////        // specimen second
////        TrajectoryActionBuilder SpecimenSecond = GoSpecimenSecond.endTrajectory().fresh()
////                .strafeTo(BeforeTurn, maxVelConstraint)
////                .turn(180)
////                .strafeTo(chamberSecondStrafe, maxVelConstraint);
////
////        // i am tired write comments
////
////        TrajectoryActionBuilder BackGoSpecimenSecond = SpecimenSecond.endTrajectory().fresh()
////                .afterDisp(0, () -> liftController.setTargetPosition(Params.LiftParams.Position.MIDDLE))
////                .strafeTo(backgoSpecimenSecond);
////        // i am so tired
////
////        // go a third specimen
////        TrajectoryActionBuilder GoSpecimenThird = BackGoSpecimenSecond.endTrajectory().fresh()
////                .afterDisp(0, () -> {
////                    outtakeController.setHandHorizontal();
////                    liftController.setTargetPosition(Params.LiftParams.Position.HOME);
////                })
////                .turn(0)
////                .strafeTo(humanPlayerThirdSampleStrafe, maxVelConstraint);
////
////        // specimen third
////        TrajectoryActionBuilder SpecimenThird = GoSpecimenThird.endTrajectory().fresh()
////                .strafeTo(BeforeTurn)
////                .turnTo(180)
////                .strafeTo(chamberThirdStrafe, maxVelConstraint);
////
////        TrajectoryActionBuilder BackGoSpecimenThird = SpecimenThird.endTrajectory().fresh()
////                .afterDisp(0, () -> liftController.setTargetPosition(Params.LiftParams.Position.MIDDLE))
////                .strafeTo(backgoSpecimenThird);
////
////
////        // go a fourth specimen
////        TrajectoryActionBuilder GoSpecimenFourth = BackGoSpecimenThird.endTrajectory().fresh()
////                .afterDisp(0, () -> {
////                    outtakeController.setHandHorizontal();
////                    liftController.setTargetPosition(Params.LiftParams.Position.HOME);
////                })
////                .turnTo(0)
////                .strafeTo(humanPlayerFourthSampleStrafe, maxVelConstraint);
////
////        // specimen fourth
////        TrajectoryActionBuilder SpecimenFourth = GoSpecimenFourth.endTrajectory().fresh()
////                .strafeTo(BeforeTurn)
////                .turnTo(180)
////                .strafeTo(chamberFourthStrafe, maxVelConstraint);
////
////        TrajectoryActionBuilder BackGoSpecimenFourth = SpecimenFourth.endTrajectory().fresh()
////                .afterDisp(0, () -> liftController.setTargetPosition(Params.LiftParams.Position.MIDDLE))
////                .strafeTo(backGoSpecimenFourth);
//
//        // go a fifth specimen
////        TrajectoryActionBuilder GoSpecimenFifth = BackGoSpecimenFourth.endTrajectory().fresh()
////                .afterDisp(0, () -> {
////                    outtakeController.setHandHorizontal();
////                    liftController.setTargetPosition(Params.LiftParams.Position.HOME);
////                })
////                .splineToLinearHeading(humanPlayerFifthSample, Math.toRadians(0), maxVelConstraint);
////
////        // specimen fourth
////        TrajectoryActionBuilder SpecimenFifty = GoSpecimenFifth.endTrajectory().fresh()
////                .splineToLinearHeading(chamberFifth, Math.toRadians(0), maxVelConstraint);
////
////        TrajectoryActionBuilder BackGoSpecimenFifth = SpecimenFifty.endTrajectory().fresh()
////                .afterDisp(0, () -> liftController.setTargetPosition(Params.LiftParams.Position.MIDDLE))
////                .strafeTo(backGoSpecimenFifth);
//
//        // parking
////        TrajectoryActionBuilder Parking = BackGoSpecimenFourth.endTrajectory().fresh()
////                .afterDisp(0, () -> {
////                    outtakeController.setHandHorizontal();
////                    liftController.setTargetPosition(Params.LiftParams.Position.HOME);
////                })
////                .strafeTo(parking, maxVelConstraint);
//
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
//                        SubmarineFirst.build(),
//                        // set 1 specimen
//                        outtakeController.setClawOpenAction(),
//                        sleepAction(25),
//
//                        FromSubmarine.build(),
//
//                        PushedSamples.build()
////
////                        GoSpecimenSecond.build(),
////                        // take 2 specimen
////                        outtakeController.setClawCloseAction(),
////                        sleepAction(175),
////                        liftToHit,
////                        outtakeController.setHandHitAction(),
////
////                        SpecimenSecond.build(),
////                        // set 2 specimen
////                        outtakeController.setClawOpenAction(),
////                        sleepAction(25),
////                        BackGoSpecimenSecond.build(),
////
////                        GoSpecimenThird.build(),
////                        // take 3 specimen
////                        outtakeController.setClawCloseAction(),
////                        sleepAction(175),
////                        liftToHit,
////                        outtakeController.setHandHitAction(),
////
////                        SpecimenThird.build(),
////                        // set 3 specimen
////                        outtakeController.setClawOpenAction(),
////                        sleepAction(25),
////                        BackGoSpecimenThird.build(),
////
////                        GoSpecimenFourth.build(),
////                        // take 4 specimen
////                        outtakeController.setClawCloseAction(),
////                        sleepAction(175),
////                        liftToHit,
////                        outtakeController.setHandHitAction(),
////
////                        SpecimenFourth.build(),
////                        // set 4 specimen
////                        outtakeController.setClawOpenAction(),
////                        sleepAction(25),
////                        BackGoSpecimenFourth.build(),
//
////                        Parking.build()
//
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
