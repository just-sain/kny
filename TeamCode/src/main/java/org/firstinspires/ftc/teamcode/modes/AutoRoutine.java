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
        outtakeController.initialize(hardwareMap, true);
        liftController.initialize(hardwareMap);
        armController.initialize(hardwareMap);

        extendController.initialize(hardwareMap);
        intakeController.initialize(hardwareMap, true);


        // init
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        // pinpoint
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose, liftController, armController);


        // go sample 1
        Vector2d forwardSampleTwo = new Vector2d(42, -20);

        // push three samples
        Vector2d strafeSampleTwo = new Vector2d(42, -28);
        Vector2d backSampleTwo = new Vector2d(19.5, -28);
        Vector2d forwardTheeSample  = new Vector2d(42, -18);
        Vector2d strafeTheeSample  = new Vector2d(42, -31);
        Vector2d backTheeSample  = new Vector2d(19.5, -31);
        Vector2d forwardForthSample  = new Vector2d(42, -28);
        Vector2d strafeForthSample  = new Vector2d(42, -38);
        Vector2d backForthSample  = new Vector2d(19, -38);

        // chamber sample
        Pose2d humanPlayerFirstChamber = new Pose2d(-0.8, -15, Math.toRadians(0));
        Pose2d chamberFirst = new Pose2d(36, 40, Math.toRadians(0));

        Pose2d humanPlayerSecondChamber = new Pose2d(-1.7, -23, Math.toRadians(0));
        Pose2d chamberSecond = new Pose2d(35, 37, Math.toRadians(0));

        Pose2d humanPlayerThirdChamber = new Pose2d(-1.4, -23, Math.toRadians(0));
        Pose2d chamberThird = new Pose2d(35, 35, Math.toRadians(0));

        Pose2d humanPlayerFourthChamber = new Pose2d(-1.8   , -23, Math.toRadians(0));
        Pose2d chamberFourth = new Pose2d(35, 34, Math.toRadians(0));

        Pose2d humanPlayerFifthChamber = new Pose2d(-2.55, -23, Math.toRadians(0));
        Pose2d chamberFifth = new Pose2d(35, 33, Math.toRadians(0));

        // from chamber
        Vector2d FromSubmarineFirst = new Vector2d(27, 36);
        Vector2d FromSubmarineSecond = new Vector2d(27, 33);
        Vector2d FromSubmarineThird = new Vector2d(27, 32);
        Vector2d FromSubmarineFourth = new Vector2d(27, 32);
        Vector2d FromSubmarineFifth = new Vector2d(27 , 32);

        // wait seconds
        double takeWaitSeconds = 0.125;


//        // trajectories
        TrajectoryActionBuilder ToSample = drive.actionBuilder(initialPose)
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

        TrajectoryActionBuilder ChamberFirst = ToSample.endTrajectory().fresh()
                .splineToLinearHeading(humanPlayerFirstChamber, Math.toRadians(0), maxVel)

                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .splineToLinearHeading(chamberFirst, Math.toRadians(0), maxVel);


        TrajectoryActionBuilder ChamberSecond = ChamberFirst.endTrajectory().fresh()
                .strafeTo(FromSubmarineFirst, maxVel)
                .afterDisp(0, () -> takeSpecimen())

                .splineToLinearHeading(humanPlayerSecondChamber, Math.toRadians(0), maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .splineToLinearHeading(chamberSecond, Math.toRadians(0), maxVel);

        // fourth 3
        TrajectoryActionBuilder ChamberThird = ChamberSecond.endTrajectory().fresh()
                .strafeTo(FromSubmarineSecond, maxVel)
                .afterDisp(0, () -> takeSpecimen())

                .splineToLinearHeading(humanPlayerThirdChamber, Math.toRadians(0), maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .splineToLinearHeading(chamberThird,Math.toRadians(0), maxVel);

        // fifth 4
        TrajectoryActionBuilder ChamberFourth = ChamberThird.endTrajectory().fresh()
                .strafeTo(FromSubmarineThird, maxVel)
                .afterDisp(0, () -> takeSpecimen())

                .splineToLinearHeading(humanPlayerFourthChamber, Math.toRadians(0), maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .splineToLinearHeading(chamberFourth, Math.toRadians(0), maxVel);

        // fifth 5
        TrajectoryActionBuilder ChamberFifth = ChamberFourth.endTrajectory().fresh()
                .strafeTo(FromSubmarineFourth, maxVel)

                .afterDisp(0, () -> takeSpecimen())
                .splineToLinearHeading(humanPlayerFifthChamber, Math.toRadians(0), maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(takeWaitSeconds)
                .afterDisp(0, () -> cyliis())

                .splineToLinearHeading(chamberFifth, Math.toRadians(0), maxVel);

        // parking
        TrajectoryActionBuilder Parking = ChamberFifth.endTrajectory().fresh()
                .strafeTo(FromSubmarineFifth, maxVel)
                .afterDisp(0, () -> toHome())
                .splineToLinearHeading(humanPlayerFifthChamber, Math.toRadians(0), maxVel);


        // lift
        Action liftToBasket = liftController.setTargetPositionAction(LiftController.Position.BASKET);
        Action liftToHome = liftController.setTargetPositionAction(LiftController.Position.HOME);

        long sleepTimeForChamber = 50;

        // wait for start
        waitForStart();

        // run
        Actions.runBlocking(
                new SequentialAction(
                        ToSample.build(),

                        ChamberFirst.build(),
                        outtakeController.setClawOpenAction(),
                        sleepAction(sleepTimeForChamber),

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