package org.firstinspires.ftc.teamcode.modes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.controllers.ExtendController;
import org.firstinspires.ftc.teamcode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;

@Autonomous(name = "auto five", group = "prod")
public class AutoRoutine extends LinearOpMode {

    // controllers
    OuttakeController outtakeController = new OuttakeController();
    LiftController liftController = new LiftController();
    ArmController armController = new ArmController();
    ExtendController extendController = new ExtendController();
    IntakeController intakeController = new IntakeController();

    // max
    VelConstraint maxVel = new TranslationalVelConstraint(1000);

    @Override
    public void runOpMode() throws InterruptedException {

        // controllers
        outtakeController.initialize(hardwareMap);
        liftController.initialize(hardwareMap);
        armController.initialize(hardwareMap);
        extendController.initialize(hardwareMap);
        intakeController.initialize(hardwareMap);


        // init
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        // pinpoint
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose, liftController, armController);

        // push robot
//        Vector2d submarineFirst = new Vector2d(23.8, 0);
//        Vector2d fromSubmarine = new Vector2d(14, -21);

        // strafe
        Vector2d forwardSampleTwo = new Vector2d(42, -20);

        // push three samples
        Vector2d strafeSampleTwo = new Vector2d(42, -24);
        Vector2d backSampleTwo = new Vector2d(19.5, -22);
        Vector2d forwardTheeSample  = new Vector2d(43, -19);
        Vector2d strafeTheeSample  = new Vector2d(43, -33);
        Vector2d backTheeSample  = new Vector2d(19.5, -33);
        Vector2d forwardForthSample  = new Vector2d(42, -33);
        Vector2d strafeForthSample  = new Vector2d(42, -41.8);
        Vector2d backForthSample  = new Vector2d(19, -41.8);

        // chamber sample
        Vector2d humanPlayerFirstChamber = new Vector2d(2, -20);
        Vector2d chamberFirst = new Vector2d(33, 26);

        Vector2d humanPlayerSecondChamber = new Vector2d(4, -15);
        Vector2d chamberSecond = new Vector2d(33, 25);

        Vector2d humanPlayerThirdChamber = new Vector2d(4, -15);
        Vector2d chamberThird = new Vector2d(33, 24);

        Vector2d humanPlayerFourthChamber = new Vector2d(4, -15);
        Vector2d chamberFourth = new Vector2d(33, 23);

        Vector2d humanPlayerFifthChamber = new Vector2d(4, -15);
        Vector2d chamberFifth = new Vector2d(33, 22);

        // from chamber
        Vector2d FromSubmarineFirst = new Vector2d(20, 26);
        Vector2d FromSubmarineSecond = new Vector2d(20, 25);
        Vector2d FromSubmarineThird = new Vector2d(20, 24);
        Vector2d FromSubmarineFourth = new Vector2d(20, 23);
        Vector2d FromSubmarineFifth = new Vector2d(20, 22);

        // stick
        Pose2d StickToFirstSample = new Pose2d(14, -38, Math.toRadians(-130));
        Pose2d StickPushFirstSample = new Pose2d(14, -20, Math.toRadians(-160));
        Pose2d StickToSampleSecondSample  = new Pose2d(16, -38, Math.toRadians(-130));
        Pose2d StickPushSecondSample = new Pose2d(16, -20, Math.toRadians(-160));
        Pose2d StickToThirdSample  = new Pose2d(16, -38, Math.toRadians(-130));
        Pose2d StickPushThirdSample = new Pose2d(16, -20, Math.toRadians(-160));





//        // trajectories
//        TrajectoryActionBuilder SubmarineFirst = drive.actionBuilder(initialPose)
//                .strafeTo(submarineFirst);
//
//        // trajectories push
//        TrajectoryActionBuilder FromTheSubmarine = SubmarineFirst.endTrajectory().fresh()
//                .strafeTo(fromSubmarine, maxVel);

        double takeWaitSeconds = 0.3;

        TrajectoryActionBuilder ToSample = drive.actionBuilder(initialPose)
                .strafeTo(forwardSampleTwo, maxVel)
                .strafeTo(strafeSampleTwo, maxVel)
                .strafeTo(backSampleTwo, maxVel)
                .strafeTo(forwardTheeSample, maxVel)
                .strafeTo(strafeTheeSample, maxVel)
                .strafeTo(backTheeSample, maxVel)
                .strafeTo(forwardForthSample, maxVel)
                .strafeTo(strafeForthSample, maxVel)
                .strafeTo(backForthSample, maxVel);

        TrajectoryActionBuilder ChamberFirst = ToSample.endTrajectory().fresh()
                .strafeTo(humanPlayerFirstChamber, maxVel)

                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .strafeTo(chamberFirst, maxVel);


        TrajectoryActionBuilder ChamberSecond = ChamberFirst.endTrajectory().fresh()
                .strafeTo(FromSubmarineFirst, maxVel)
                .afterDisp(0, () -> takeSpecimen())

                .strafeTo(humanPlayerSecondChamber, maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .strafeTo(chamberSecond, maxVel);

        // fourth 3
        TrajectoryActionBuilder ChamberThird = ChamberSecond.endTrajectory().fresh()
                .strafeTo(FromSubmarineSecond, maxVel)
                .afterDisp(0, () -> takeSpecimen())

                .strafeTo(humanPlayerThirdChamber, maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .strafeTo(chamberThird, maxVel);

        // fifth 4
        TrajectoryActionBuilder ChamberFourth = ChamberThird.endTrajectory().fresh()
                .strafeTo(FromSubmarineThird, maxVel)
                .afterDisp(0, () -> takeSpecimen())

                .strafeTo(humanPlayerFourthChamber, maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .strafeTo(chamberFourth, maxVel);

        // fifth 5
        TrajectoryActionBuilder ChamberFifth = ChamberFourth.endTrajectory().fresh()
                .strafeTo(FromSubmarineFourth, maxVel)

                .afterDisp(0, () -> takeSpecimen())
                .strafeTo(humanPlayerFifthChamber, maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .strafeTo(chamberFifth, maxVel);

        // parking
        TrajectoryActionBuilder Parking = ChamberFifth.endTrajectory().fresh()
                .strafeTo(FromSubmarineFifth, maxVel)
                .strafeTo(humanPlayerFifthChamber, maxVel);


        // lift
        Action liftToBasket = liftController.setTargetPositionAction(LiftController.Position.BASKET);
        Action liftToHome = liftController.setTargetPositionAction(LiftController.Position.HOME);

        // wait for start
        waitForStart();

        // run
        Actions.runBlocking(
                new SequentialAction(
                        ToSample.build(),

                        ChamberFirst.build(),
                        outtakeController.setClawOpenAction(),

                        ChamberSecond.build(),
                        outtakeController.setClawOpenAction(),

                        ChamberThird.build(),
                        outtakeController.setClawOpenAction(),

                        ChamberFourth.build(),
                        outtakeController.setClawOpenAction(),

                        ChamberFifth.build(),
                        outtakeController.setClawOpenAction(),

                        Parking.build()
                )
        );
    }

    // cyliis
    public void cyliis() {
        // cyliis high chamber
        liftController.setTargetPosition(LiftController.Position.CYLIIS);
        armController.setChamber();
    }

    // take
    public void takeSpecimen() {
        // take specimen
        liftController.setTargetPosition(LiftController.Position.HOME);
        armController.setTake();
    }

    // sleep action, sometimes help
    public Action sleepAction(long milliseconds) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                for (int i = 0; i <= milliseconds; i += 25) {
                    liftController.periodic();

                    sleep(25);
                }
                return false;
            }
        };
    }
}