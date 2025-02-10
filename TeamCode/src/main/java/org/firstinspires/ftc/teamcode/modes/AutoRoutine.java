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
        outtakeController.initialize(hardwareMap, false);
        liftController.initialize(hardwareMap);
        armController.initialize(hardwareMap, false);
        extendController.initialize(hardwareMap);
        intakeController.initialize(hardwareMap);


        // init
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        // pinpoint
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose, liftController, armController);

        // push robot
        Vector2d submarineFirst = new Vector2d(23.8, 0);
        Vector2d fromSubmarine = new Vector2d(14, -21);

        // strafe
        Vector2d forwardSampleTwo = new Vector2d(46, -38);

        // push three samples
        Vector2d strafeSampleTwo = new Vector2d(47, -40);
        Vector2d backSampleTwo = new Vector2d(18, -40);
        Vector2d forwardTheeSample  = new Vector2d(46.5, -28);
        Vector2d strafeTheeSample  = new Vector2d(46.5, -46);
        Vector2d backTheeSample  = new Vector2d(18, -46);
        Vector2d forwardForthSample  = new Vector2d(45.5, -45);
        Vector2d strafeForthSample  = new Vector2d(45.5, -55);
        Vector2d backForthSample  = new Vector2d(18, -55);

        // chamber sample
        Vector2d humanPlayerSecondSample = new Vector2d(0, -40);
        Vector2d chamberSecond = new Vector2d(27.8, 6);

        Vector2d humanPlayerThirdSample = new Vector2d(0, -40);
        Vector2d chamberThird = new Vector2d(27.8, 5);

        Vector2d humanPlayerFourthSample = new Vector2d(-2, -40);
        Vector2d chamberFourth = new Vector2d(27.9, 2);

        Vector2d humanPlayerFifthSample = new Vector2d(0, -40);
        Vector2d chamberFifth = new Vector2d(27.8, 2);

        // from chamber
        Vector2d FromSubmarineThird = new Vector2d(15, 5);
        Vector2d FromSubmarineFourthSample = new Vector2d(15, 3);
        Vector2d FromSubmarineFifthSample = new Vector2d(15, 0);
        Vector2d FromSubmarineParking = new Vector2d(15, -2);




        // trajectories
        TrajectoryActionBuilder SubmarineFirst = drive.actionBuilder(initialPose)
                .strafeTo(submarineFirst);

        // trajectories push
        TrajectoryActionBuilder FromTheSubmarine = SubmarineFirst.endTrajectory().fresh()
                .strafeTo(fromSubmarine, maxVel);

        TrajectoryActionBuilder ToSample = FromTheSubmarine.endTrajectory().fresh()
                .strafeTo(forwardSampleTwo, maxVel)
                .strafeTo(strafeSampleTwo, maxVel)
                .strafeTo(backSampleTwo, maxVel)
                .strafeTo(forwardTheeSample)
                .strafeTo(strafeTheeSample, maxVel)
                .strafeTo(backTheeSample, maxVel)
                .strafeTo(forwardForthSample, maxVel)
                .strafeTo(strafeForthSample, maxVel)
                .strafeTo(backForthSample, maxVel);

        TrajectoryActionBuilder ChamberTwo = ToSample.endTrajectory().fresh()
                .strafeTo(humanPlayerSecondSample, maxVel)
                .strafeTo(chamberSecond);


        TrajectoryActionBuilder ChamberThird = ChamberTwo.endTrajectory().fresh()
                .strafeTo(FromSubmarineThird, maxVel)
                .afterDisp(0, () -> takeSpecimen())
                .strafeTo(humanPlayerThirdSample, maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(0.3)
                .afterDisp(0, () -> cybugs())
                .strafeTo(chamberThird)
                .afterDisp(0, () -> {
                    outtakeController.setClawOpen();
                    liftController.setTargetPosition(LiftController.Position.HOME);
                });

        // fourth 4
        TrajectoryActionBuilder ChamberFourth = ChamberThird.endTrajectory().fresh()
                .strafeTo(FromSubmarineFourthSample, maxVel)
                .afterDisp(0, () -> takeSpecimen())
                .strafeTo(humanPlayerFourthSample, maxVel)
                .afterDisp(0, () -> outtakeController.setClawClose())
                .waitSeconds(0.3)
                .afterDisp(0, () -> cybugs())
                .strafeTo(chamberFourth)
                .afterDisp(0, () -> {
                    outtakeController.setClawOpen();
                    liftController.setTargetPosition(LiftController.Position.HOME);
                });

//        // fifth 5
//        TrajectoryActionBuilder ChamberFifth = ChamberFourth.endTrajectory().fresh()
//                .strafeTo(FromSubmarineFifthSample, maxVel)
//                .afterDisp(0, () -> takeSpecimen())
//                .strafeTo(humanPlayerFifthSample, maxVel)
//                .afterDisp(0, () -> outtakeController.setClawClose())
//                .waitSeconds(0.3)
//                .afterDisp(0, () -> cybugs())
//                .strafeTo(chamberFifth)
//                .afterDisp(0, () -> {
//                    outtakeController.setClawOpen();
//                    liftController.setTargetPosition(LiftController.Position.HOME);
//                });


        // parking
        TrajectoryActionBuilder Parking = ChamberFourth.endTrajectory().fresh()
                .strafeTo(FromSubmarineParking)
                .afterTime(0.3, () -> takeSpecimen())
                .strafeTo(humanPlayerFifthSample, maxVel);



        // lift
        Action liftToBasket = liftController.setTargetPositionAction(LiftController.Position.BASKET);
        Action liftToCybugs = liftController.setTargetPositionAction(LiftController.Position.CYBUGS);
        Action liftToHome = liftController.setTargetPositionAction(LiftController.Position.HOME);

        // wait for start
        waitForStart();

        // run
        Actions.runBlocking(
                new SequentialAction(
                        SubmarineFirst.build(),

                        // pushes
                        FromTheSubmarine.build(),
                        ToSample.build()

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

    // take specimen
    public Action takeSpecimenAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                liftController.setTargetPosition(LiftController.Position.HOME);

                armController.setTake();
                outtakeController.setWristTake();
                outtakeController.setHandBackward();
                return false;
            }
        };
    }

    // cybugs
    public Action cybugsAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // cybugs high chamber
                liftController.setTargetPosition(LiftController.Position.CYBUGS);

                armController.setChamber();
                outtakeController.setWristCybugs();
                outtakeController.setHandForward();

                return false;
            }
        };
    }

    // cybugs
    public void cybugs() {
        // cybugs high chamber
        liftController.setTargetPosition(LiftController.Position.CYBUGS);

        armController.setChamber();
        outtakeController.setWristCybugs();
        outtakeController.setHandForward();
    }

    // cybugs 1
    public void cybugs1() {
        // cybugs high chamber
        liftController.setTargetPosition(LiftController.Position.CYBUGS1);

        armController.setChamber();
        outtakeController.setWristCybugs();
        outtakeController.setHandForward();
    }

    // take
    public void takeSpecimen() {
        // take specimen
        liftController.setTargetPosition(LiftController.Position.HOME);

        armController.setTake();
        outtakeController.setWristTake();
        outtakeController.setHandBackward();
    }
}