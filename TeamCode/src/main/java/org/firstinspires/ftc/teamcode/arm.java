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
        COLLECTION
    }
    armState state;

    public static double rest;

    public static double RESTING_POWER = 100;
    public static double BASKET_POWER = 100;
    public static double SPECIMEN_POWER = 100;
    public static double COLLECTION_POWER = 150;
    public static double Sliderpowerup = 2500;
    public static double Sliderpowerdown = 500;
    public static int resting_position = 30;
    public static int basket_position = 170;
    public static int specimen_position = 220;
    public static int collection_position = 410;
    public static double wristpos_resting = 0;
    public static double wristpos_basket = 0.6;
    public static double wristpos_specimen = 0.8;
    public static double wristpos_collection = 0.85;
    public static int slidersdown = 0;
    public static int slidersup = 2800;
    public static double MAX_POS     =  1.0;     // Maximum rotational position
    public static double MIN_POS     =  0.0;// Minimum rotational position
    double output;
    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Servo wrist;
    Servo claw;
    public static double  wrist_position = (MAX_POS - MIN_POS) / 2;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        imu = hardwareMap.get(IMU.class, "imu");
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
            /*
            if(this.gamepad2.dpad_up){
                sliderPreset1();
//                slidersGo(sliderSpeed);
            }
            if(this.gamepad2.dpad_down){
                slidersGo(-sliderSpeed); //Go down, so negative
            }
            */
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


            wrist_position = gamepad2.left_stick_y;
            if (gamepad2.y) {
                state = armState.RESTING;
            }

            if (gamepad2.x) {
                state = armState.BASKET;
            }

            if (gamepad2.a) {
                state = armState.SPECIMEN;
            }

            if (gamepad2.b) {
                state = armState.COLLECTION;
            }
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
    //Fixes slider stopping issue
    /*public void slidersStop(){
        int rightTarget = SliderRight.getCurrentPosition();
        int leftTarget = SliderLeft.getCurrentPosition();
        SliderLeft.setTargetPosition(leftTarget);
        SliderRight.setTargetPosition(rightTarget);
        SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderRight.setPower(0.2);
        SliderLeft.setPower(0.2);

    }
    public void slidersGo(double power){
        int leftPos = SliderLeft.getCurrentPosition();
        int rightPos = SliderRight.getCurrentPosition();
        SliderRight.setTargetPosition(100);
        SliderLeft.setTargetPosition(100);
        SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("leftPos", leftPos);
        telemetry.addData("rightPos", rightPos);
        telemetry.update();
        SliderLeft.setPower(power);
        SliderRight.setPower(power);
    }*/
    //yet to be tested, kind of works
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
        power = clamp(power, -MAXPOWER, MAXPOWER);
        backLeftPower = power;
        frontLeftPower = power;
        backRightPower  = -power;
        frontRightPower = -power;
    }

    public double clamp(double input, double min, double max) {
        double result = input;
        if (input > max) {
            result = max;
        } else if (input < min) {
            result = min;
        }
        return result;
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
    //High basket, up_dpad
    public void sliderPreset1(){
        int rightTargetPos = 3800;
        int leftTargetPos = 3800;
        if(SliderRight.getCurrentPosition() < rightTargetPos || SliderLeft.getCurrentPosition() < leftTargetPos) {
            SliderLeft.setPower(0.35);
            SliderRight.setPower(0.35);
            SliderLeft.setTargetPosition(leftTargetPos);
            SliderRight.setTargetPosition(rightTargetPos);
            telemetry.addData("sliderpreset1", "Active");
        }
        telemetry.addData("sliderpreset1", "inactive");

    }
}
