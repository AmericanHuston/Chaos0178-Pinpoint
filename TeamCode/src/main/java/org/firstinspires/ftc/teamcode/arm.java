package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "arm", group = "TeleOp")
public class arm extends LinearOpMode {
    public DcMotorEx SliderLeft;
    public DcMotorEx SliderRight;
    public DcMotorEx Shoulder;
    double backRightPower;
    double frontRightPower;
    double frontLeftPower;
    double backLeftPower;
    enum armState{
        RESTING,
        BASKET,
        SPECIMEN,
        COLLECTION,
        above_bar,
        below_bar

    }
    armState state;

    public static double rest;

    public static double RESTING_POWER = 150;
    public static double BASKET_POWER = 200;
    public static double SPECIMEN_POWER = 150;
    public static double COLLECTION_POWER = 150;
    public static double Sliderpowerup = 2500;
    public static double Sliderpowerdown = 1200;
    public static int resting_position = 50;
    public static int basket_position = 170;
    public static int specimen_position = 220;
    public static int collection_position = 410;
    public static double wristpos_resting = 0.15;
    public static double wristpos_basket = 0.6;
    public static double wristpos_specimen = 0.8;
    public static double wristpos_collection = 0.85;
    public static int slidersdown = 40;
    public static int slidersup = 2800;
    public static double MAX_POS     =  1.0;     // Maximum rotational position
    public static double MIN_POS     =  0.0;// Minimum rotational position
    public static int slider_above_bar_position = 500;
    public static int slider_below_bar_position = 400;
    public static int shoulder_bar_position = 170;
    public static double wrist_bar_position = 0.39;
    public static int shoulder_bar_velotity = 160;
    double output;
    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Servo wrist;
    Servo claw;
    TouchSensor sliderButton;
    public static double  wrist_position = (MAX_POS - MIN_POS) / 2;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        imu = hardwareMap.get(IMU.class, "imu");
        sliderButton = hardwareMap.touchSensor.get("sliderButton");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        SliderLeft = hardwareMap.get(DcMotorEx.class, "SliderLeft");
        SliderRight = hardwareMap.get(DcMotorEx.class, "SliderRight");
        Shoulder = hardwareMap.get(DcMotorEx.class, "Shoulder");
        Shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SliderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SliderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SliderRight.setDirection(DcMotorSimple.Direction.REVERSE); //It needs to be reversed because...
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        final double sliderSpeed = 0.35;
        state = armState.RESTING;
        claw.setPosition(0.5);
        boolean open = true;
        boolean changed = false;


        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.back) {
                imu.resetYaw();
            }
            if(gamepad2.right_trigger > 0.01) {
                output = Range.scale(gamepad2.right_trigger, 0.0, 1.0, 0.5, 0.99);
                claw.setPosition(output);
            }
            if(gamepad2.right_bumper && !changed){
                if(open) {
                    claw.setPosition(0.99);
                }else{
                    claw.setPosition(0.5);
                }
                changed = true;
                open = !open;
            } else if (!gamepad2.right_bumper) {
                changed = false;
            }
            if (gamepad2.right_stick_x> 0.01|| gamepad2.right_stick_x < -0.01)  {
                int shoulder_position = Shoulder.getTargetPosition();
                int shoulder_change = (int)gamepad2.right_stick_x * 10;
                int shoulder_new_position = shoulder_position + shoulder_change;;
                Shoulder.setTargetPosition(shoulder_new_position);
            }

            wrist_position = gamepad2.left_stick_y;
            if (gamepad2.y) { state = armState.RESTING; }
            if (gamepad2.x) { state = armState.BASKET; }
            if (gamepad2.a) { state = armState.SPECIMEN; }
            if (gamepad2.b) { state = armState.COLLECTION; }
            if (gamepad2.left_bumper) {
                rest +=.01;
            }
            if (gamepad2.right_bumper) {
                rest -=.01;
            }
            if (gamepad2.dpad_up) {
                SliderLeft.setTargetPosition(slidersup);
                SliderRight.setTargetPosition(slidersup);
                SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderLeft.setVelocity(Sliderpowerup);
                SliderRight.setVelocity(Sliderpowerup);
            }
            if (gamepad2.dpad_down) {
                SliderLeft.setTargetPosition(slidersdown);
                SliderRight.setTargetPosition(slidersdown);
                SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderLeft.setVelocity(Sliderpowerdown);
                SliderRight.setVelocity(Sliderpowerdown);
            }
            if (gamepad2.dpad_left) {
                state = armState.above_bar;
            }
            if (gamepad2.dpad_right) {
                state = armState.below_bar;
            }
            if (sliderButton.isPressed()){
                SliderLeft.setMotorDisable();
                SliderRight.setMotorDisable();
            }

            driving();

            if (gamepad1.a) {
                pointAtBasket();
            }
            arm();
            action();
            //slidersStop();
            telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("arm ticks", rest);
            telemetry.addData("Current ", Shoulder.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ClawPos", output);
            telemetry.update();
        }
    }

   //sets shoulder motor position need the right presets
    public void arm(){
        telemetry.addData("state", String.valueOf(state));
        switch (state){
            case RESTING:
                Shoulder.setTargetPosition(resting_position);
                Shoulder.setVelocity(RESTING_POWER);
                wrist.setPosition(wristpos_resting);
                SliderLeft.setTargetPosition(slidersdown);
                SliderRight.setTargetPosition(slidersdown);
                SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderLeft.setVelocity(Sliderpowerdown);
                SliderRight.setVelocity(Sliderpowerdown);

                break;
            case BASKET:
                Shoulder.setTargetPosition(basket_position);
                Shoulder.setVelocity(BASKET_POWER);
                wrist.setPosition(wristpos_basket);
                SliderLeft.setTargetPosition(slidersup);
                SliderRight.setTargetPosition(slidersup);
                SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderLeft.setVelocity(Sliderpowerup);
                SliderRight.setVelocity(Sliderpowerup);
                break;
            case SPECIMEN:
                Shoulder.setTargetPosition(specimen_position);
                Shoulder.setVelocity(SPECIMEN_POWER);
                wrist.setPosition(wristpos_specimen);
                break;
            case COLLECTION:
                Shoulder.setTargetPosition(collection_position);
                Shoulder.setVelocity(COLLECTION_POWER);
                wrist.setPosition(wristpos_collection);
                break;
            case above_bar:
                Shoulder.setTargetPosition(shoulder_bar_position);
                Shoulder.setVelocity(shoulder_bar_velotity);
                wrist.setPosition(wrist_bar_position);
                SliderLeft.setTargetPosition(slider_above_bar_position);
                SliderRight.setTargetPosition(slider_above_bar_position);
                SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderLeft.setVelocity(Sliderpowerup);
                SliderRight.setVelocity(Sliderpowerup);
                break;
            case below_bar:
                Shoulder.setTargetPosition(shoulder_bar_position);
                Shoulder.setVelocity(shoulder_bar_velotity);
                wrist.setPosition(wrist_bar_position);
                SliderLeft.setTargetPosition(slider_below_bar_position);
                SliderRight.setTargetPosition(slider_below_bar_position);
                SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderLeft.setVelocity(Sliderpowerdown);
                SliderRight.setVelocity(Sliderpowerdown);
                break;


        }


        Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    //resting or normal mode

    public void servo(Servo servo, double increment){
        double position = servo.getPosition();
        position = position + increment;
        servo.setPosition(position);
    }
    //driving is working, field centric
    private void pointAtBasket() {
       double pointedAtBasket = -45.0;
       pointAtAngle(pointedAtBasket);
    }
    private void pointAtAngle(double pointAt){
        double MAXPOWER = 0.5;
        double  Kp = 0.2;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw();
        double power = Kp *(pointAt - currentYaw);
        power = Range.clip(power, -MAXPOWER, MAXPOWER);
        backLeftPower = power;
        frontLeftPower = power;
        backRightPower  = -power;
        frontRightPower = -power;
    }
    public void driving() {
        double y = -gamepad1.left_stick_y / 2; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x / 2;
        double rx = gamepad1.right_stick_x / 2;
        if (gamepad1.right_trigger >= 0.01) {
            y = y * 2;
            x = x * 2;
            rx = rx * 2;
        }
        //speed up/slow down
        if (gamepad1.left_trigger >= 0.01) {
            y = y / 2;
            x = x / 2;
            rx = rx / 2;
        }
        //When dpad down is pressed it will point at basket

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
    }
    public void action() {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

    }
}
