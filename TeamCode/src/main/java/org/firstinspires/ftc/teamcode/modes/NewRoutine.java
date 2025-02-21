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

@Autonomous(name = "new routine", group = "prod")
public class NewRoutine extends LinearOpMode {

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
        outtakeController.initialize(hardwareMap, false);
        liftController.initialize(hardwareMap);
        armController.initialize(hardwareMap, true);

        extendController.initialize(hardwareMap);
        intakeController.initialize(hardwareMap, true);


        // init
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        // pinpoint
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose, liftController);

        // push robot
        Vector2d firstChamber = new Vector2d(25, 0);

        Vector2d fromSubmarine = new Vector2d(10, -14);

        // go sample 1
        Vector2d forwardSampleTwo = new Vector2d(43, -45);

        // push three samples
        Vector2d strafeSampleTwo = new Vector2d(43, -49);
        Vector2d backSampleTwo = new Vector2d(19, -49);
        Vector2d forwardTheeSample  = new Vector2d(42, -40);
        Vector2d strafeTheeSample  = new Vector2d(42, -51);
        Vector2d backTheeSample  = new Vector2d(19.5, -51);
        Vector2d forwardForthSample  = new Vector2d(42, -46);
        Vector2d strafeForthSample  = new Vector2d(42, -52);
        Vector2d backForthSample  = new Vector2d(19, -52);

        // chamber sample
        Pose2d humanPlayerSecondChamber = new Pose2d(-2.2, -35, Math.toRadians(0));
        Pose2d chamberSecond = new Pose2d(38, 26, Math.toRadians(0));

        Pose2d humanPlayerThirdChamber = new Pose2d(-2.9, -45, Math.toRadians(0));
        Pose2d chamberThird = new Pose2d(37.5, 21, Math.toRadians(0));

        Pose2d humanPlayerFourthChamber = new Pose2d(-2.9, -45, Math.toRadians(0));
        Pose2d chamberFourth = new Pose2d(37.5, 18, Math.toRadians(0));

        Pose2d humanPlayerFifthChamber = new Pose2d(-2.9, -45, Math.toRadians(0));
        Pose2d chamberFifth = new Pose2d(37.5, 16, Math.toRadians(0));

        Pose2d humanPlayerParkingChamber = new Pose2d(-2.6, -45, Math.toRadians(0));


        // from chamber
        Vector2d FromSubmarineSecond = new Vector2d(27, 20);
        Vector2d FromSubmarineThird = new Vector2d(27, 18);
        Vector2d FromSubmarineFourth = new Vector2d(27, 16);
        Vector2d FromSubmarineFifth = new Vector2d(27, 16);


        // wait seconds
        double takeWaitSeconds = 0.14;

        TrajectoryActionBuilder ChamberFirst = drive.actionBuilder(initialPose)
                .afterDisp(0, () -> cyliis())
                .strafeTo(firstChamber);

        TrajectoryActionBuilder FromSubramineFirst = ChamberFirst.endTrajectory().fresh()
                .strafeTo(fromSubmarine, maxVel)
                .afterDisp(0, () -> takeSpecimen());

//        // trajectories
        TrajectoryActionBuilder ToSample = FromSubramineFirst.endTrajectory().fresh()
                .strafeTo(forwardSampleTwo, maxVel)
                .strafeTo(strafeSampleTwo, maxVel)
                .strafeTo(backSampleTwo, maxVel)
                .strafeTo(forwardTheeSample, maxVel)
                .strafeTo(strafeTheeSample, maxVel)
                .strafeTo(backTheeSample, maxVel)
                .strafeTo(forwardForthSample, maxVel)
                .strafeTo(strafeForthSample, maxVel)
                .strafeTo(backForthSample, maxVel)
                .afterDisp(0, () -> takeSpecimen());

        TrajectoryActionBuilder ChamberSecond = ToSample.endTrajectory().fresh()
                .splineToLinearHeading(humanPlayerSecondChamber, Math.toRadians(0), maxVel)

                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .splineToLinearHeading(chamberSecond, Math.toRadians(0));


        TrajectoryActionBuilder ChamberThird = ChamberSecond.endTrajectory().fresh()
                .strafeTo(FromSubmarineThird, maxVel)
                .afterDisp(0, () -> takeSpecimen())

                .splineToLinearHeading(humanPlayerThirdChamber, Math.toRadians(0), maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .splineToLinearHeading(chamberThird, Math.toRadians(0));

        // fourth 3
        TrajectoryActionBuilder ChamberFourth = ChamberThird.endTrajectory().fresh()
                .strafeTo(FromSubmarineFourth, maxVel)
                .afterDisp(0, () -> takeSpecimen())

                .splineToLinearHeading(humanPlayerFourthChamber, Math.toRadians(0), maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .splineToLinearHeading(chamberFourth,Math.toRadians(0));

        // fifth 4
        TrajectoryActionBuilder ChamberFifth = ChamberFourth.endTrajectory().fresh()
                .strafeTo(FromSubmarineFifth, maxVel)
                .afterDisp(0, () -> takeSpecimen())

                .splineToLinearHeading(humanPlayerFifthChamber, Math.toRadians(0), maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .splineToLinearHeading(chamberFifth, Math.toRadians(0));

        // parking
        TrajectoryActionBuilder Parking = ChamberFifth.endTrajectory().fresh()
                .strafeTo(FromSubmarineFifth, maxVel)
                .afterDisp(0, () -> toHome())
                .splineToLinearHeading(humanPlayerFifthChamber, Math.toRadians(0), maxVel);


        // lift
        Action liftToBasket = liftController.setTargetPositionAction(LiftController.Position.BASKET);
        Action liftToHome = liftController.setTargetPositionAction(LiftController.Position.HOME);

        long sleepTimeForChamber = 100;

        // wait for start
        waitForStart();

        // run
        Actions.runBlocking(
                new SequentialAction(
                        ChamberFirst.build(),
                        outtakeController.setClawOpenAction(),
                        sleepAction(sleepTimeForChamber),

                        FromSubramineFirst.build(),

                        ToSample.build(),

                        ChamberSecond.build(),
                        outtakeController.setClawOpenAction(),
                        sleepAction(sleepTimeForChamber),

                        ChamberThird.build(),
                        outtakeController.setClawOpenAction(),
                        sleepAction(sleepTimeForChamber),

                        ChamberFourth.build(),
                        outtakeController.setClawOpenAction(),
                        sleepAction(sleepTimeForChamber),

                        ChamberFifth.build(),
                        outtakeController.setClawOpenAction(),
                        sleepAction(sleepTimeForChamber),

                        Parking.build()
                )
        );
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

    // cyliis
    public void cyliis() {
        // cyliis high chamber
        liftController.setTargetPosition(LiftController.Position.CYLIIS);
        armController.setChamber();
    }

    // take
    public void takeSpecimen() {
        // take specimen
        liftController.setTargetPosition(LiftController.Position.TAKE);
        armController.setTake();
    }

    // home pos
    public void toHome() {
        // take specimen
        liftController.setTargetPosition(LiftController.Position.HOME);
        armController.setTake();
    }


}