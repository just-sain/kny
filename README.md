# KIROSHI NOT YAKU (kimetsu no yaiba=)
robot code preview
here we use some tricks like Position Enum for PID controller, showLogs method and so on

## some info
kny robot from cafc 2025 or tech cup XI
**into the deep** season
without **ftclib**
here some

### motors
everywhere used **gobilda yellow jacket _435 rpm_** motor :3
// i like this rpm))

the robot weighs 15 kg and 435 rpm which are on two elevators can carry our robot at hang

### servo
with servo nothing interesting, nothing new, simple servo methods except rotation in intake controller

### some global methods and rules
used controller/subsystems method where controller has rule single responsibility, i hope

| - teamcode
| --- _/rofls_ directory is for rofls, so do not open :) 
| --- _/controllers_ there controllers
| --- _/modes_ teleop and auto run files
| --- _/auto_ tunning, localization, otos and pinpoint drive!

#### show logs method
**showLogs(Telemetry telemetry)** method helps to debug
method add all motors, servo, pid target, pos data
added only for debug

```
telemetry.addLine("--- ARM ---");
armController.showLogs(telemetry);

telemetry.addLine("--- OUTTAKE ---");
outtakeController.showLogs(telemetry);

telemetry.update();
```

#### initialize method
decided to not use class constructor when you create new class, example:
`BaseController baseController = new BaseController(HardwareMap hardwareMap, Telemetry telemetry);`

so we just add initialize method in every controller class, and now our code look:
```
BaseController baseController = new BaseController();
IntakeController intakeController = new IntakeController();
...
@Override
public void runOpMode() {
  baseController.initialize(hardwareMap);
  intakeController.initialize(hardwareMap, false);
  ...
  waitForStart();
```

also decided to separate auto and teleop init with `boolean isAuto` in initialize method

### extension and lifts
kny robot have one motor on extension and two motors for lifts
files path="TeamCode/src/main/java/org/firstinspires/ftc/teamcode/controllers"

1. extension (named ArmController) was created with **2 Stage Viper-Slide Kit (Belt-Driven, 336mm Slides)** used **gobilda _435 rpm_** 
2. lift (named LiftController) was also created with **4 Stage Viper-Slide Kit (Belt-Driven, 336mm Slides)** used **gobilda _435 rpm_** x2

code is simillar for extension and lift

#### custom pid controller
there is **custom pid controller**, works amazing
also had to add manual and pid modes which can help to set position manually or just press button and go to position
used **DcMotorEx** class
method **periodic** should work evenly with robot teleop/auto **run time**, so remember that
you can use also ftc dashboard for debugging

in controller also have a lot of methods for comfortable work with pid controller, such as
1. **setPower(double power)** will switch to manual mode
2. **resetEncoders()** will reset encoders
3. **getIsManMode()** will retrun _boolean_ is using manual mode
4. **periodic()** periodicly
5. **setTargetPosition(Position position)** Position **enum** where all usefull position such as: HOME, CHAMBER, BASKET, HANG..
6. **setTargetToCurrentPos()** will set target to current pos and change man mode to pid

as you can see dual mode is usefull for driver, 'cause driver can customly set target with two trigger on gamepad
here how to put logically to teleop mode
```
// -- lift pid mode switch --
if (gamepad2.left_trigger > 0.05) {
  // man mode - down
  liftController.setPower(-gamepad2.left_trigger);
} else if (gamepad2.right_trigger > 0.05) {
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
```

do not panic there is no stack, the **setTargetToCurrentPos()** will change mode

also if you gonna use manual mode, i recommend to set on R3/L3 (stick button) reset encoders
```
// -- resetting extend encoders, just in case --
if (gamepad1.right_stick_button) {
  extendController.resetEncoders();
}
```

**NOTE:** _if you gonna to use, please change pid coefficients_
**NOTE:** _remember about periodic method_

#### position enum
enum is powerfull
u should create separate **Constants** class with lift positions or write a lot of methods for every target position
with enum u can just add every numbers and use it without creating full class, 'cause enum static
how it looks
```
public enum Position {
  HOME(0),
  TAKE(25),
  CHAMBER(1225),
  HANG(2000),
  BASKET(2300),
  PASS(300),
  MAX(2500);

  Position(int pos) {
    this.position = pos;
  }
  private int position;

  public int getPos() {
    return position;
  }
}
```

so now you can set pos only from enum positions, not custom 'caues in `setTargetPosition(Position position)` you can set only from enum
if you wanna set number instead enum, you can create another method `setTargetCustomPosition(int position)`

with servo values you can do same thing!

### intake, the servo responsible for rotation
our driver like to use stick for rotation, so here how we realize that:
firstly delimit (will help if the servo hits something)
```
// hand
private Servo hand;
// hand positions
public static double HAND_MIN_POS     = 0;
public static double HAND_NEUTRAL_POS = 0.5;
public static double HAND_MAX_POS     = 0.79; // if in your situation there is no hits, you can just set max pos to 1
```

methods for mode
```
public void setHandNeutral() {
  hand.setPosition(HAND_NEUTRAL_POS);
}

public void setHandCustom(double stickInput) {
  double pos = (-stickInput + 1) / 2; // if there is reverse just put/delete - 

  if (pos <= HAND_MIN_POS) pos = HAND_MIN_POS;
  if (pos >= HAND_MAX_POS) pos = HAND_MAX_POS;

  hand.setPosition(pos);
}
```

how it looks in teleop mode file
```
// -- intake hand --
if (Math.abs(gamepad2.left_stick_x) > 0.05) {
  intakeController.setHandCustom(-gamepad2.left_stick_x);
} else {
  intakeController.setHandNeutral();
}
```
when you not touching stick rotation go to neutral position, in our situtation is 90 degree 

**NOTE:** _put teleop code to periodic (loop)_

### base controller
just used plus and minus
```
public void controlBySticks(double left_stick_y, double right_stick_x, double left_stick_x) {
  double speed  = -left_stick_y;
  double strafe = left_stick_x;
  double turn   = right_stick_x;

  lf.setPower(speed + turn + strafe);
  rf.setPower(speed - turn - strafe);
  lr.setPower(speed + turn - strafe);
  rr.setPower(speed - turn + strafe);
}
```

in old commits we use in auto mode **ZeroPowerBehavior.BRAKE** and teleop mode **ZeroPowerBehavior.FLOAT** to smoothly stop
about zeropowerbehavior you can read here: https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.ZeroPowerBehavior.html
initialize was:
```
public void initialize(HardwareMap hardwareMap, DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotor.RunMode runMode) {
  // init
  lf = hardwareMap.get(DcMotor.class, "lf"); // 0
  rf = hardwareMap.get(DcMotor.class, "rf"); // 1
  lr = hardwareMap.get(DcMotor.class, "lr"); // 2
  rr = hardwareMap.get(DcMotor.class, "rr"); // 3

  // setting direction
  lf.setDirection(DcMotor.Direction.REVERSE);
  rf.setDirection(DcMotor.Direction.FORWARD);
  lr.setDirection(DcMotor.Direction.REVERSE);
  rr.setDirection(DcMotor.Direction.FORWARD);

  // setting the motor 0 mode power to be brake as it actively stops the robot
  // and doesn't rely on the surface to slow down once the robot power is set to 0
  lf.setZeroPowerBehavior(zeroPowerBehavior);
  rf.setZeroPowerBehavior(zeroPowerBehavior);
  lr.setZeroPowerBehavior(zeroPowerBehavior);
  rr.setZeroPowerBehavior(zeroPowerBehavior);

  // reset encoders
  lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  // setting motors run mode, will help with autonomous
  lf.setMode(runMode);
  rf.setMode(runMode);
  lr.setMode(runMode);
  rr.setMode(runMode);

  // setting 0 power to the motors wheel
  lf.setPower(0);
  ...
}
```

`base.initialize(hardwareMap, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);`
something like that

## teleop
in teleop mode nothing interesting, just gamepad event listeners and some methods which need periodic
for beginners, if you have position for hang do something like this for automaticly hide everything to not break something
```
// --- lift ---
// -- lift setting target position --
if (gamepad2.dpad_down) {
  takeSpecimen();
} else if (gamepad2.dpad_right) {
  // hang
  liftController.setTargetPosition(LiftController.Position.HANG);
  // hiding everything 
  armController.setHome();

  extendController.setTargetPosition(ExtendController.Position.HANG);
  intakeController.setClawClose();
  intakeController.setWristUp();
}
```

## auto
pinpoint and 2 dead wheels
roadrunner 1.0 xD
rr docs -> https://rr.brott.dev/docs/v1-0/installation/

here will no tips how to tune rr 1.0, but..

use only strafe and spline

periodic methods of lift and extend controller we put in pinpoint drive, there is built-in loop which repeat every robot loop time
```
public PinpointDrive(HardwareMap hardwareMap, Pose2d pose, LiftController liftController) {
  super(hardwareMap, pose);
  ...
  // custom
  this.liftController = liftController;
  ...
```

here is actions without Action class -<
```
// set speciment to chamber
TrajectoryActionBuilder ChamberFirst = ToSample.endTrajectory().fresh()
  .splineToLinearHeading(humanPlayerFirstChamber, Math.toRadians(0))

  .afterDisp(0, () -> outtakeController.setClawClose())
  .waitSeconds(takeWaitSeconds)
  .afterDisp(0, () -> cyliis())

  .splineToLinearHeading(chamberFirst, Math.toRadians(0));
```

we use translational velocity constraint for some moments, which help to take slowly and push sample as faster fock
constants
```
// vel constants
VelConstraint maxVel = new TranslationalVelConstraint(1000);
VelConstraint slowlyVel = new TranslationalVelConstraint(200);
...
.strafeTo(FromSubmarineSecond, maxVel)
```

we use our cuctom sleep action for **Actions.runBlocking**, it's works like simple sleep
```
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
```

how to use **sleepAction(long milliseconds)**
```
ChamberFirst.build(),
outtakeController.setClawOpenAction(),
sleepAction(sleepTimeForChamber),
```

# by kiroshi
- kiroshi/just-sain
- kiroshi/swid-yera
