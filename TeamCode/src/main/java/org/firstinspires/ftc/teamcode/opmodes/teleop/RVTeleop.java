package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.checkerframework.checker.units.UnitsTools.mm;
import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.navigation.PID;


@TeleOp(name = "Minion_OP")
public class RVTeleop extends LinearOpMode {
    //DriveTrain Variables
    public static double turn_kp = 0.007;
    public static double turn_ki = 0.125;
    public static double turn_kd = 0.0028;
    public static double turn_a = 0.8;
    public static double turn_max_i_sum = 1;
    public static double turn_clip = 1;
    public static double forward_speed = 1;
    public static double strafe_speed = 1.1;
    public static double turn_speed = 0.75;
    private final PID turn_pid = new PID(turn_kp, turn_ki, turn_kd, 0.2, turn_max_i_sum, turn_a);
    private double heading_delta = 0;
    private double heading_was = 0;
    private double heading_unwrapped = 0;
    private double wraparounds = 0;
    private double turn_angle = 0;
    private boolean angled_turn = false;
    private double speed_dependent_steering = 0.5;
    private boolean field_centric = false;
    private boolean reverse_drive = false;
    boolean fastmode = false;
    //Slide FeedForward Variables
    private final double liftPosScale = 50;
    private final double liftPowScale = 0.0025;
    private final double liftPosScale2 = 50;
    private final double liftPowScale2 = 0.0025;
    public double liftPosCurrent = 0, liftPosDes = 0, liftPosError = 0, liftPow = 0;
    public double liftPosCurrent2 = 0, liftPosDes2 = 0, liftPosError2 = 0, liftPow2 = 0;
    double speedK = 1;
    double speedK2 = 1;
    public int pos = 1;
    //Declare Motors, Servos, Sensors, ETC
    public DcMotor leftFront, leftRear, rightRear, rightFront;
    public DcMotor LSLM, RSLM, Intake;
    public Servo RotServR, BackDropServ, AutoServ, Airplane;
    public CRServo HoldPix;
    public DistanceSensor LeftDis, RightDis;
    //public RevBlinkinLedDriver lights;
    public BNO055IMU imu_sensor;
    //Variables for Other Automations
    public ElapsedTime runtime = new ElapsedTime();
    public int LeftDisSens = 30;
    public int RightDisSens = 30;
    boolean timer = false;
    boolean box = false;

    public void runOpMode() { //code will run once only
        //Wheels Intiz Names
        leftFront = hardwareMap.get(DcMotorEx.class, "front left");
        leftRear = hardwareMap.get(DcMotorEx.class, "back left");
        rightRear = hardwareMap.get(DcMotorEx.class, "back right");
        rightFront = hardwareMap.get(DcMotorEx.class, "front right");
        //Slides Intiz Names
        RSLM = hardwareMap.get(DcMotorEx.class, "lift right");
        LSLM = hardwareMap.get(DcMotorEx.class, "lift left");
        //Intake Intiz Names
        Intake = hardwareMap.get(DcMotorEx.class, "intake spinner");
        //Servo Intiz Names
        HoldPix = hardwareMap.get(CRServo.class, "deposit wheel");
        RotServR = hardwareMap.get(Servo.class, "deposit right");
        BackDropServ = hardwareMap.get(Servo.class, "pixel dropper");
        AutoServ = hardwareMap.get(Servo.class, "pixel dragger");
        Airplane = hardwareMap.get(Servo.class, "air plane launcher");
        //Distance Sensor Intiz Names
        LeftDis = hardwareMap.get(DistanceSensor.class, "left distance sensor");
        RightDis = hardwareMap.get(DistanceSensor.class, "right distance sensor");
        //Light Intiz Name
        //lights = hardwareMap.get(RevBlinkinLedDriver.class, "RobotLights");
        //IMU Intiz Name and other
        imu_sensor = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.gyroRange = BNO055IMU.GyroRange.DPS2000;
        imu_sensor.initialize(parameters);

        //Reversing inverted motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        LSLM.setDirection(DcMotorSimple.Direction.REVERSE);
        //Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        //Slides reset for automation
        LSLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RSLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RSLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Deadwheels reset
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Intialize light color
        //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);

        telemetry.setAutoClear(false);
        telemetry.clearAll();
        telemetry.addData("LEFT Slide Enc: ", LSLM.getCurrentPosition());
        telemetry.addData("RIGHT Slide Enc: ", RSLM.getCurrentPosition());
        telemetry.addData("Center Deadwheel: ", leftFront.getCurrentPosition());
        telemetry.addData("Left Deadwheel: ", rightFront.getCurrentPosition());
        telemetry.addData("Right Deadwheel: ", rightRear.getCurrentPosition());
        telemetry.addData("Box Rotation Servo: ", RotServR.getPosition());
        telemetry.addData("Airplane Servo: ", Airplane.getPosition());
        telemetry.addData("Left Distance: ", LeftDis.getDistance(DistanceUnit.MM));
        telemetry.addData("Right Distance: ", RightDis.getDistance(DistanceUnit.MM));
        telemetry.addData("IMU First Angle: ", imu_sensor.getAngularOrientation().firstAngle);
        telemetry.addData("IMU Second Angle: ", imu_sensor.getAngularOrientation().secondAngle);
        telemetry.addData("IMU Third Angle: ", imu_sensor.getAngularOrientation().thirdAngle);
        telemetry.addLine("Airplane is Locked");
        telemetry.addData("Match Timer:", runtime.seconds());
        telemetry.update();

        //Gampbad Rumble builds for Teleop
        Gamepad.RumbleEffect GameTime;
        GameTime = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)
                .build();

        Gamepad.RumbleEffect Endgame;
        Endgame = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 300)
                .addStep(1.0, 0.0, 300)
                .build();

        Gamepad.RumbleEffect GameEnd;
        GameEnd = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.0, 1000)
                .addStep(0.0, 0.5, 1000)
                .addStep(0.5, 0.0, 1000)
                .addStep(0.0, 0.5, 1000)
                .addStep(1.0, 1.0, 2000)
                .build();


        telemetry.addLine("Ready to Deploy Teleop");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        while (opModeIsActive()) {

            //Timer resets once when program starts
            if (!timer) {
                runtime.reset();
                runtime.startTime();
                timer = true;
            }

            telemetry.clearAll();
            telemetry.addData("Running Teleop", "");

            //Lights automatically set to off
            //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

            //Turn fastmode off and on with gamepad 1 Start/Arrow Button
            if (gamepad1.start) {
                fastmode = !fastmode;
            }

            //Start Drive
            double heading = imu_sensor.getAngularOrientation().firstAngle;

            if (gamepad1.dpad_up) {
                turn_angle = 0;
                angled_turn = true;
            }
            if (gamepad1.dpad_left) {
                turn_angle = 90;
                angled_turn = true;
            }
            if (gamepad1.dpad_down) {
                turn_angle = 180;
                angled_turn = true;
            }
            if (gamepad1.dpad_right) {
                turn_angle = 270;
                angled_turn = true;
            }

            if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                angled_turn = false;
            }

            double slow = 1 - (gamepad1.left_trigger / 3);

            double y = -gamepad1.left_stick_x * forward_speed * slow;
            double x = gamepad1.left_stick_y * strafe_speed * slow;
            double rx = gamepad1.right_stick_x * turn_speed * slow;

            rx *= (1 - (Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y * 0.8, 2)) * speed_dependent_steering)); //pythagorean theorem

            //drive modes
            if (gamepad1.b) {
                reverse_drive = !reverse_drive;
            }
            if (gamepad1.a) {
                field_centric = !field_centric;
            }

            double rot = 0.0;
            if (Math.signum(-heading) == -1) {
                rot = ((-heading) + 360);
            } else {
                rot = -heading;
            }

            rot %= 360;

            if (Math.abs(turn_angle - rot) > Math.abs(turn_angle - (rot - 360))) {
                rot -= 360;
            }
            if (Math.signum(-heading) == -1) {
                rot = ((-heading) + 360);
            } else {
                rot = -heading;
            }

            rot %= 360;

            if (Math.abs(turn_angle - rot) > Math.abs(turn_angle - (rot - 360))) {
                rot -= 360;
            }
            if (angled_turn) {
                rx = turn_pid.getOutPut(turn_angle, rot, 0);
            }

            //drift correction
            heading_delta = heading - heading_was;

            if (heading_delta > 320) {
                heading_delta -= 360;
                wraparounds -= 1;
            }
            if (heading_delta < -320) {
                heading_delta += 360;
                wraparounds += 1;
            }

            heading_unwrapped = -(heading + (wraparounds * 360));

            if (gamepad1.right_stick_x != 0) {
                heading_delta = 0;
            }

            //drivetrain
            double heading_radians = Math.toRadians(heading);

            double rotX = x * Math.cos(-heading_radians) - y * Math.sin(-heading_radians);
            double rotY = x * Math.sin(-heading_radians) + y * Math.cos(-heading_radians);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            if (!reverse_drive) {
                if (field_centric) {
                    move(rotY, rotX, rx, (heading_delta * 0.001), denominator);
                } else {
                    move(y, x, rx, (heading_delta * 0.001));
                }
            } else {
                if (field_centric) {
                    reverseMove(rotY, rotX, rx, (heading_delta * 0.001), denominator);
                } else {
                    reverseMove(y, x, rx, (heading_delta * 0.001));
                }
            }

            heading_was = heading;
            //end drive

            //Rumbles Controllers every 30 seconds and in last 5 seconds it does countdown rumble
            if (runtime.seconds() == 30 || runtime.seconds() == 60) {
                gamepad1.runRumbleEffect(GameTime);
                gamepad2.runRumbleEffect(GameTime);
            } else if (runtime.seconds() == 90) {
                gamepad1.runRumbleEffect(Endgame);
                gamepad2.runRumbleEffect(Endgame);
            } else if (runtime.seconds() == 115) {
                gamepad1.runRumbleEffect(GameEnd);
                gamepad2.runRumbleEffect(GameEnd);
            }

            //Only allows airplane to be launched in last 30 seconds during Endgame
            while (runtime.seconds() >= 60.0) {
                if (gamepad2.x) {
                    Airplane.setPosition(0.1);
                }
            }

            //LEFT slide FeedForward
            liftPosDes += speedK * liftPosScale * gamepad2.right_stick_y / 0.75; //input scale factor
            liftPosError = liftPosDes - liftPosCurrent;
            liftPow = Math.min(Math.max(liftPowScale * liftPosError, -1.00), 1.00); //proportional gain
            if (liftPow >= 1) {
                liftPosDes = liftPosCurrent + (1 / liftPowScale);
            } //AntiWindup Code
            if (liftPow <= -1) {
                liftPosDes = liftPosCurrent - (1 / liftPowScale);
            } //AntiWindup Code
            LSLM.setPower(liftPow);

            //RIGHT slide FeedForward
            liftPosDes2 += speedK2 * liftPosScale2 * gamepad2.right_stick_y / 0.75;
            liftPosError2 = liftPosDes2 - liftPosCurrent2;
            liftPow2 = Math.min(Math.max(liftPowScale2 * liftPosError2, -1.00), 1.00);
            if (liftPow2 >= 1) {
                liftPosDes2 = liftPosCurrent2 + (1 / liftPowScale2);
            }
            if (liftPow2 <= -1) {
                liftPosDes2 = liftPosCurrent2 - (1 / liftPowScale2);
            }
            RSLM.setPower(liftPow2);

            //Calls the Intake(Gamepad 2 A button) or Deposit(Gamepad 2 B button Position for slides and sets box to intake or drop position
            if (gamepad2.a) {
                box = false;
                intakePos();
            } else if (gamepad2.b) {
                DropPos();
                box = true;
            }

            //If box ever needs to be set manually use Gamepad 2 Y button
            if (gamepad2.y) {
                box = !box;
            }

            //Box Positions for intake and drop
            if (box) {
                RotServR.setPosition(0.5);
            } else {
                RotServR.setPosition(0.0);
            }

            //Spins intake both ways based on Gamepad 2 Triggers
            if (gamepad2.right_trigger > 0.2) {
                Intake.setPower(1);
                HoldPix.setPower(1);
            } else if (gamepad2.left_trigger > 0.2) {
                Intake.setPower(-1);
                HoldPix.setPower(-1);
            }

            //Gamepad 1 A button aligns robot 30 mm away from an object, built for board
            if (gamepad1.a) {
                allign();
            }

            telemetry.addData("LEFT Slide Enc: ", LSLM.getCurrentPosition());
            telemetry.addData("RIGHT Slide Enc: ", RSLM.getCurrentPosition());
            telemetry.addData("Center Deadwheel: ", leftFront.getCurrentPosition());
            telemetry.addData("Left Deadwheel: ", rightFront.getCurrentPosition());
            telemetry.addData("Right Deadwheel: ", rightRear.getCurrentPosition());
            telemetry.addData("Box Rotation Servo: ", RotServR.getPosition());
            telemetry.addData("Airplane Servo: ", Airplane.getPosition());
            telemetry.addData("Left Distance: ", LeftDis.getDistance(DistanceUnit.MM));
            telemetry.addData("Right Distance: ", RightDis.getDistance(DistanceUnit.MM));
            telemetry.addData("IMU First Angle: ", imu_sensor.getAngularOrientation().firstAngle);
            telemetry.addData("IMU Second Angle: ", imu_sensor.getAngularOrientation().secondAngle);
            telemetry.addData("IMU Third Angle: ", imu_sensor.getAngularOrientation().thirdAngle);
            telemetry.addLine("Lights are off");
            if(runtime.seconds() >= 60.0){
                telemetry.addLine("Airplane is Unlocked");
            } else {
                telemetry.addLine("Airplane is Locked");
            }
            telemetry.addData("Match Timer:", runtime.seconds());
            telemetry.update();
        }
    }

    public void intakePos() {
        LSLM.setPower(0);
        RSLM.setPower(0);

        //set target position
        LSLM.setTargetPosition(-1845);
        RSLM.setTargetPosition(-1845);

        LSLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RSLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        LSLM.setPower(0.7);
        RSLM.setPower(0.7);

        while (LSLM.isBusy() && RSLM.isBusy()) {
            telemetry.clearAll();
            telemetry.addData("Running Teleop", "");

            //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);

            Gamepad.RumbleEffect GameTime;
            GameTime = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 1.0, 500)
                    .build();

            Gamepad.RumbleEffect Endgame;
            Endgame = new Gamepad.RumbleEffect.Builder()
                    .addStep(0.0, 1.0, 300)
                    .addStep(1.0, 0.0, 300)
                    .build();

            Gamepad.RumbleEffect GameEnd;
            GameEnd = new Gamepad.RumbleEffect.Builder()
                    .addStep(0.5, 0.0, 1000)
                    .addStep(0.0, 0.5, 1000)
                    .addStep(0.5, 0.0, 1000)
                    .addStep(0.0, 0.5, 1000)
                    .addStep(1.0, 1.0, 2000)
                    .build();

            if (runtime.seconds() == 30 || runtime.seconds() == 60) {
                gamepad1.runRumbleEffect(GameTime);
                gamepad2.runRumbleEffect(GameTime);
            } else if (runtime.seconds() == 90) {
                gamepad1.runRumbleEffect(Endgame);
                gamepad2.runRumbleEffect(Endgame);
            } else if (runtime.seconds() == 115) {
                gamepad1.runRumbleEffect(GameEnd);
                gamepad2.runRumbleEffect(GameEnd);
            }

            while (runtime.seconds() >= 60.0) {
                if (gamepad2.x) {
                    Airplane.setPosition(0.1);
                }
            }

            if (gamepad1.start) {
                fastmode = !fastmode;
            }

            //Start Drive
            double heading = imu_sensor.getAngularOrientation().firstAngle;

            if (gamepad1.dpad_up) {
                turn_angle = 0;
                angled_turn = true;
            }
            if (gamepad1.dpad_left) {
                turn_angle = 90;
                angled_turn = true;
            }
            if (gamepad1.dpad_down) {
                turn_angle = 180;
                angled_turn = true;
            }
            if (gamepad1.dpad_right) {
                turn_angle = 270;
                angled_turn = true;
            }

            if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                angled_turn = false;
            }

            double slow = 1 - (gamepad1.left_trigger / 3);

            double y = -gamepad1.left_stick_x * forward_speed * slow;
            double x = gamepad1.left_stick_y * strafe_speed * slow;
            double rx = gamepad1.right_stick_x * turn_speed * slow;

            rx *= (1 - (Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y * 0.8, 2)) * speed_dependent_steering)); //pythagorean theorem

            //drive modes
            if (gamepad1.b) {
                reverse_drive = !reverse_drive;
            }
            if (gamepad1.a) {
                field_centric = !field_centric;
            }

            double rot = 0.0;
            if (Math.signum(-heading) == -1) {
                rot = ((-heading) + 360);
            } else {
                rot = -heading;
            }

            rot %= 360;

            if (Math.abs(turn_angle - rot) > Math.abs(turn_angle - (rot - 360))) {
                rot -= 360;
            }
            if (Math.signum(-heading) == -1) {
                rot = ((-heading) + 360);
            } else {
                rot = -heading;
            }

            rot %= 360;

            if (Math.abs(turn_angle - rot) > Math.abs(turn_angle - (rot - 360))) {
                rot -= 360;
            }
            if (angled_turn) {
                rx = turn_pid.getOutPut(turn_angle, rot, 0);
            }

            //drift correction
            heading_delta = heading - heading_was;

            if (heading_delta > 320) {
                heading_delta -= 360;
                wraparounds -= 1;
            }
            if (heading_delta < -320) {
                heading_delta += 360;
                wraparounds += 1;
            }

            heading_unwrapped = -(heading + (wraparounds * 360));

            if (gamepad1.right_stick_x != 0) {
                heading_delta = 0;
            }

            //drivetrain
            double heading_radians = Math.toRadians(heading);

            double rotX = x * Math.cos(-heading_radians) - y * Math.sin(-heading_radians);
            double rotY = x * Math.sin(-heading_radians) + y * Math.cos(-heading_radians);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            if (!reverse_drive) {
                if (field_centric) {
                    move(rotY, rotX, rx, (heading_delta * 0.001), denominator);
                } else {
                    move(y, x, rx, (heading_delta * 0.001));
                }
            } else {
                if (field_centric) {
                    reverseMove(rotY, rotX, rx, (heading_delta * 0.001), denominator);
                } else {
                    reverseMove(y, x, rx, (heading_delta * 0.001));
                }
            }

            heading_was = heading;
            //end drive

            if (gamepad2.right_trigger > 0.2) {
                Intake.setPower(1);
                HoldPix.setPower(1);
            } else if (gamepad2.left_trigger > 0.2) {
                Intake.setPower(-1);
                HoldPix.setPower(-1);
            }

            telemetry.addData("LEFT Slide Enc: ", LSLM.getCurrentPosition());
            telemetry.addData("RIGHT Slide Enc: ", RSLM.getCurrentPosition());
            telemetry.addData("Center Deadwheel: ", leftFront.getCurrentPosition());
            telemetry.addData("Left Deadwheel: ", rightFront.getCurrentPosition());
            telemetry.addData("Right Deadwheel: ", rightRear.getCurrentPosition());
            telemetry.addData("Box Rotation Servo: ", RotServR.getPosition());
            telemetry.addData("Airplane Servo: ", Airplane.getPosition());
            telemetry.addData("Left Distance: ", LeftDis.getDistance(DistanceUnit.MM));
            telemetry.addData("Right Distance: ", RightDis.getDistance(DistanceUnit.MM));
            telemetry.addData("IMU First Angle: ", imu_sensor.getAngularOrientation().firstAngle);
            telemetry.addData("IMU Second Angle: ", imu_sensor.getAngularOrientation().secondAngle);
            telemetry.addData("IMU Third Angle: ", imu_sensor.getAngularOrientation().thirdAngle);
            telemetry.addLine("Lights are Strobe White");
            if(runtime.seconds() >= 60.0){
                telemetry.addLine("Airplane is Unlocked");
            } else {
                telemetry.addLine("Airplane is Locked");
            }
            telemetry.addData("Match Timer:", runtime.seconds());
            telemetry.update();
        }
        LSLM.setPower(0);
        RSLM.setPower(0);
    }

    public void DropPos() {
        LSLM.setPower(0);
        RSLM.setPower(0);

        //set target position
        LSLM.setTargetPosition(1845);
        RSLM.setTargetPosition(-1845);


        LSLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RSLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set power
        LSLM.setPower(0.7);
        RSLM.setPower(0.7);

        while (LSLM.isBusy() && RSLM.isBusy()) {
            telemetry.clearAll();
            telemetry.addData("Running Teleop", "");

            //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);

            Gamepad.RumbleEffect GameTime;
            GameTime = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 1.0, 500)
                    .build();

            Gamepad.RumbleEffect Endgame;
            Endgame = new Gamepad.RumbleEffect.Builder()
                    .addStep(0.0, 1.0, 300)
                    .addStep(1.0, 0.0, 300)
                    .build();

            Gamepad.RumbleEffect GameEnd;
            GameEnd = new Gamepad.RumbleEffect.Builder()
                    .addStep(0.5, 0.0, 1000)
                    .addStep(0.0, 0.5, 1000)
                    .addStep(0.5, 0.0, 1000)
                    .addStep(0.0, 0.5, 1000)
                    .addStep(1.0, 1.0, 2000)
                    .build();

            if (runtime.seconds() == 30 || runtime.seconds() == 60) {
                gamepad1.runRumbleEffect(GameTime);
                gamepad2.runRumbleEffect(GameTime);
            } else if (runtime.seconds() == 90) {
                gamepad1.runRumbleEffect(Endgame);
                gamepad2.runRumbleEffect(Endgame);
            } else if (runtime.seconds() == 115) {
                gamepad1.runRumbleEffect(GameEnd);
                gamepad2.runRumbleEffect(GameEnd);
            }

            while (runtime.seconds() >= 60.0) {
                if (gamepad2.x) {
                    Airplane.setPosition(0.1);
                }
            }

            if (gamepad1.start) {
                fastmode = !fastmode;
            }

            //Start Drive
            double heading = imu_sensor.getAngularOrientation().firstAngle;

            if (gamepad1.dpad_up) {
                turn_angle = 0;
                angled_turn = true;
            }
            if (gamepad1.dpad_left) {
                turn_angle = 90;
                angled_turn = true;
            }
            if (gamepad1.dpad_down) {
                turn_angle = 180;
                angled_turn = true;
            }
            if (gamepad1.dpad_right) {
                turn_angle = 270;
                angled_turn = true;
            }

            if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                angled_turn = false;
            }

            double slow = 1 - (gamepad1.left_trigger / 3);

            double y = -gamepad1.left_stick_x * forward_speed * slow;
            double x = gamepad1.left_stick_y * strafe_speed * slow;
            double rx = gamepad1.right_stick_x * turn_speed * slow;

            rx *= (1 - (Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y * 0.8, 2)) * speed_dependent_steering)); //pythagorean theorem

            //drive modes
            if (gamepad1.b) {
                reverse_drive = !reverse_drive;
            }
            if (gamepad1.a) {
                field_centric = !field_centric;
            }

            double rot = 0.0;
            if (Math.signum(-heading) == -1) {
                rot = ((-heading) + 360);
            } else {
                rot = -heading;
            }

            rot %= 360;

            if (Math.abs(turn_angle - rot) > Math.abs(turn_angle - (rot - 360))) {
                rot -= 360;
            }
            if (Math.signum(-heading) == -1) {
                rot = ((-heading) + 360);
            } else {
                rot = -heading;
            }

            rot %= 360;

            if (Math.abs(turn_angle - rot) > Math.abs(turn_angle - (rot - 360))) {
                rot -= 360;
            }
            if (angled_turn) {
                rx = turn_pid.getOutPut(turn_angle, rot, 0);
            }

            //drift correction
            heading_delta = heading - heading_was;

            if (heading_delta > 320) {
                heading_delta -= 360;
                wraparounds -= 1;
            }
            if (heading_delta < -320) {
                heading_delta += 360;
                wraparounds += 1;
            }

            heading_unwrapped = -(heading + (wraparounds * 360));

            if (gamepad1.right_stick_x != 0) {
                heading_delta = 0;
            }

            //drivetrain
            double heading_radians = Math.toRadians(heading);

            double rotX = x * Math.cos(-heading_radians) - y * Math.sin(-heading_radians);
            double rotY = x * Math.sin(-heading_radians) + y * Math.cos(-heading_radians);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            if (!reverse_drive) {
                if (field_centric) {
                    move(rotY, rotX, rx, (heading_delta * 0.001), denominator);
                } else {
                    move(y, x, rx, (heading_delta * 0.001));
                }
            } else {
                if (field_centric) {
                    reverseMove(rotY, rotX, rx, (heading_delta * 0.001), denominator);
                } else {
                    reverseMove(y, x, rx, (heading_delta * 0.001));
                }
            }

            heading_was = heading;
            //end drive

            if (gamepad2.right_trigger > 0.2) {
                Intake.setPower(1);
                HoldPix.setPower(1);
            } else if (gamepad2.left_trigger > 0.2) {
                Intake.setPower(-1);
                HoldPix.setPower(-1);
            }

            telemetry.addData("LEFT Slide Enc: ", LSLM.getCurrentPosition());
            telemetry.addData("RIGHT Slide Enc: ", RSLM.getCurrentPosition());
            telemetry.addData("Center Deadwheel: ", leftFront.getCurrentPosition());
            telemetry.addData("Left Deadwheel: ", rightFront.getCurrentPosition());
            telemetry.addData("Right Deadwheel: ", rightRear.getCurrentPosition());
            telemetry.addData("Box Rotation Servo: ", RotServR.getPosition());
            telemetry.addData("Airplane Servo: ", Airplane.getPosition());
            telemetry.addData("Left Distance: ", LeftDis.getDistance(DistanceUnit.MM));
            telemetry.addData("Right Distance: ", RightDis.getDistance(DistanceUnit.MM));
            telemetry.addData("IMU First Angle: ", imu_sensor.getAngularOrientation().firstAngle);
            telemetry.addData("IMU Second Angle: ", imu_sensor.getAngularOrientation().secondAngle);
            telemetry.addData("IMU Third Angle: ", imu_sensor.getAngularOrientation().thirdAngle);
            telemetry.addLine("Lights are Strobe Gold");
            if(runtime.seconds() >= 60.0){
                telemetry.addLine("Airplane is Unlocked");
            } else {
                telemetry.addLine("Airplane is Locked");
            }
            telemetry.addData("Match Timer:", runtime.seconds());
            telemetry.update();
        }
        LSLM.setPower(0);
        RSLM.setPower(0);
    }

    public void allign() {
        while (gamepad1.a) {
            while (LeftDis.getDistance(DistanceUnit.MM) > LeftDisSens || RightDis.getDistance(DistanceUnit.MM) > RightDisSens) {
                if (LeftDis.getDistance(DistanceUnit.MM) > LeftDisSens) {
                    leftFront.setPower(0.5);
                    leftRear.setPower(0.5);
                } else {
                    leftFront.setPower(0.0);
                    leftRear.setPower(0.0);
                }
                if (RightDis.getDistance(DistanceUnit.MM) > RightDisSens) {
                    leftFront.setPower(0.5);
                    leftRear.setPower(0.5);
                } else {
                    leftFront.setPower(0.0);
                    leftRear.setPower(0.0);
                }

                telemetry.clearAll();
                telemetry.addData("Running Teleop", "");

                //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);

                Gamepad.RumbleEffect GameTime;
                GameTime = new Gamepad.RumbleEffect.Builder()
                        .addStep(1.0, 1.0, 500)
                        .build();

                Gamepad.RumbleEffect Endgame;
                Endgame = new Gamepad.RumbleEffect.Builder()
                        .addStep(0.0, 1.0, 300)
                        .addStep(1.0, 0.0, 300)
                        .build();

                Gamepad.RumbleEffect GameEnd;
                GameEnd = new Gamepad.RumbleEffect.Builder()
                        .addStep(0.5, 0.0, 1000)
                        .addStep(0.0, 0.5, 1000)
                        .addStep(0.5, 0.0, 1000)
                        .addStep(0.0, 0.5, 1000)
                        .addStep(1.0, 1.0, 2000)
                        .build();

                if (runtime.seconds() == 30 || runtime.seconds() == 60) {
                    gamepad1.runRumbleEffect(GameTime);
                    gamepad2.runRumbleEffect(GameTime);
                } else if (runtime.seconds() == 90) {
                    gamepad1.runRumbleEffect(Endgame);
                    gamepad2.runRumbleEffect(Endgame);
                } else if (runtime.seconds() == 115) {
                    gamepad1.runRumbleEffect(GameEnd);
                    gamepad2.runRumbleEffect(GameEnd);
                }

                //LEFT slide FeedForward
                liftPosDes += speedK * liftPosScale * gamepad2.right_stick_y / 0.75; //input scale factor
                liftPosError = liftPosDes - liftPosCurrent;
                liftPow = Math.min(Math.max(liftPowScale * liftPosError, -1.00), 1.00); //proportional gain
                if (liftPow >= 1) {
                    liftPosDes = liftPosCurrent + (1 / liftPowScale);
                } //AntiWindup Code
                if (liftPow <= -1) {
                    liftPosDes = liftPosCurrent - (1 / liftPowScale);
                } //AntiWindup Code
                LSLM.setPower(liftPow);

                //RIGHT slide FeedForward
                liftPosDes2 += speedK2 * liftPosScale2 * gamepad2.right_stick_y / 0.75;
                liftPosError2 = liftPosDes2 - liftPosCurrent2;
                liftPow2 = Math.min(Math.max(liftPowScale2 * liftPosError2, -1.00), 1.00);
                if (liftPow2 >= 1) {
                    liftPosDes2 = liftPosCurrent2 + (1 / liftPowScale2);
                }
                if (liftPow2 <= -1) {
                    liftPosDes2 = liftPosCurrent2 - (1 / liftPowScale2);
                }
                RSLM.setPower(liftPow2);


                if (gamepad2.y) {
                    box = !box;
                }

                if (box) {
                    RotServR.setPosition(0.5);
                } else {
                    RotServR.setPosition(0.0);
                }

                if (gamepad2.right_trigger > 0.2) {
                    Intake.setPower(1);
                    HoldPix.setPower(1);
                } else if (gamepad2.left_trigger > 0.2) {
                    Intake.setPower(-1);
                    HoldPix.setPower(-1);
                }

                telemetry.addData("LEFT Slide Enc: ", LSLM.getCurrentPosition());
                telemetry.addData("RIGHT Slide Enc: ", RSLM.getCurrentPosition());
                telemetry.addData("Center Deadwheel: ", leftFront.getCurrentPosition());
                telemetry.addData("Left Deadwheel: ", rightFront.getCurrentPosition());
                telemetry.addData("Right Deadwheel: ", rightRear.getCurrentPosition());
                telemetry.addData("Box Rotation Servo: ", RotServR.getPosition());
                telemetry.addData("Airplane Servo: ", Airplane.getPosition());
                telemetry.addData("Left Distance: ", LeftDis.getDistance(DistanceUnit.MM));
                telemetry.addData("Right Distance: ", RightDis.getDistance(DistanceUnit.MM));
                telemetry.addData("IMU First Angle: ", imu_sensor.getAngularOrientation().firstAngle);
                telemetry.addData("IMU Second Angle: ", imu_sensor.getAngularOrientation().secondAngle);
                telemetry.addData("IMU Third Angle: ", imu_sensor.getAngularOrientation().thirdAngle);
                telemetry.addLine("Lights are Strobe Red");
                if(runtime.seconds() >= 60.0){
                    telemetry.addLine("Airplane is Unlocked");
                } else {
                    telemetry.addLine("Airplane is Locked");
                }
                telemetry.addData("Match Timer:", runtime.seconds());
                telemetry.update();
            }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
    }

    public void move(double forward, double strafe, double turn, double turn_correct) {
        if (fastmode) {
            leftFront.setPower((forward + strafe + (turn + turn_correct)));
            rightFront.setPower((forward - strafe - (turn + turn_correct)));
            leftRear.setPower((forward - strafe + (turn + turn_correct)));
            rightRear.setPower((forward + strafe - (turn + turn_correct)));
        } else {
            leftFront.setPower(((forward + strafe + (turn + turn_correct))) * 0.75);
            rightFront.setPower(((forward - strafe - (turn + turn_correct))) * 0.75);
            leftRear.setPower(((forward - strafe + (turn + turn_correct))) * 0.75);
            rightRear.setPower(((forward + strafe - (turn + turn_correct))) * 0.75);
        }
    }

    public void move(double forward, double strafe, double turn, double turn_correct, double denominator) {
        double v = (forward + strafe + (turn + turn_correct)) / denominator;
        double v1 = (forward - strafe - (turn + turn_correct)) / denominator;
        double v2 = (forward - strafe + (turn + turn_correct)) / denominator;
        double v3 = (forward + strafe - (turn + turn_correct)) / denominator;
        if (fastmode) {
            leftFront.setPower(v);
            rightFront.setPower(v1);
            leftRear.setPower(v2);
            rightRear.setPower(v3);
        } else {
            leftFront.setPower(v * 0.75);
            rightFront.setPower(v1 * 0.75);
            leftRear.setPower(v2 * 0.75);
            rightRear.setPower(v3 * 0.75);
        }
    }

    public void reverseMove(double forward, double strafe, double turn, double turn_correct) {
        double v = -(forward - strafe + (turn + turn_correct));
        double v1 = -(forward + strafe - (turn + turn_correct));
        double v2 = -(forward + strafe + (turn + turn_correct));
        double v3 = -(forward - strafe - (turn + turn_correct));

        if (fastmode) {
            leftFront.setPower(v2);
            rightFront.setPower(v3);
            leftRear.setPower(v);
            rightRear.setPower(v1);
        } else {
            leftFront.setPower(v2 * 0.75);
            rightFront.setPower(v3 * 0.75);
            leftRear.setPower(v * 0.75);
            rightRear.setPower(v1 * 0.75);
        }
    }

    public void reverseMove(double forward, double strafe, double turn, double turn_correct, double denominator) {
        double v = -((forward + strafe + (turn + turn_correct)) / denominator);
        double v1 = -((forward - strafe - (turn + turn_correct)) / denominator);
        double v2 = -((forward - strafe + (turn + turn_correct)) / denominator);
        double v3 = -((forward + strafe - (turn + turn_correct)) / denominator);
        if (fastmode) {
            leftFront.setPower(v);
            rightFront.setPower(v1);
            leftRear.setPower(v2);
            rightRear.setPower(v3);
        } else {
            leftFront.setPower(v * 0.75);
            rightFront.setPower(v1 * 0.75);
            leftRear.setPower(v2 * 0.75);
            rightRear.setPower(v3 * 0.75);
        }
    }
}