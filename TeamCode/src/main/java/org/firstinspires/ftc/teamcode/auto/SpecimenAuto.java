package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.Locale;

@Autonomous(name = "SpecimenAuto", group="auto")
@Config
public class SpecimenAuto extends LinearOpMode {    public DcMotorEx SliderLeft;
    public DcMotorEx SliderRight;
    public DcMotorEx Shoulder;
    double backRightPower;
    double frontRightPower;
    double frontLeftPower;
    double backLeftPower;
    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Servo wrist;
    Servo claw;
    GoBildaPinpointDriverRR pinpoint;
    TouchSensor sliderButton;
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

    final double CLAW_OPEN = 0.5;
    final double CLAW_CLOSED = 0.99;

    public static int MS_FOR_BACKDRIVE = 450;
    public static int MS_FOR_FORWARDDRIVE = 600;
    public static int driveForwardTime = 6000;
    public static double RESTING_VELOCITY = 200;
    public static double BASKET_VELOCITY = 200;
    public static double SPECIMEN_VELOCITY = 210;
    public static double COLLECTION_VELOCITY = 210;
    public static double Slidervelocityup = 2500;
    public static double Slidervelocitydown = 1200;
    public static int resting_position = 50;
    public static int basket_position = 170;
    public static int specimen_position = 370;
    public static int collection_position = 500;
    public static int slidersdown = 40;
    public static int slidersup = 3500;
    public static double MAX_POS     =  1.0;     // Maximum rotational position
    public static double MIN_POS     =  0.0;// Minimum rotational position
    public static int slider_above_bar_position = 1050;
    public static int slider_below_bar_position = 400;
    public static int shoulder_bar_position = 170;
    public static int shoulder_bar_velotity = 200;
    public static double kp = 0.2;
    public static double desired_claw_position;
    public static int desired_shoulder_position;
    public static double desired_shoulder_velocity;
    public static double output;
    public static int desired_slider_position;
    public static double desired_slider_velocity;
    public static double desired_wrist_position = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        //Don't edit code below this point
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        //Don't edit code above this point

        // Declare our motors
        // Make sure your ID's match your configuration
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
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SliderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SliderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SliderRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD.ordinal());
        //pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            wrist.setPosition(0.5);

            state = armState.RESTING;
            arm();
            desired_claw_position = CLAW_CLOSED;
            armAction();
            //start with claw closed
            //raise sliders DONE
            //tilt arm DONE
            state = armState.above_bar;
            arm();
            armAction();
            sleep(driveForwardTime);
            //move forward DONE
            drive(0.0, -0.6, 0.0);
            driveAction(MS_FOR_FORWARDDRIVE);
            sleep(1000);
            //open claw
            state = armState.below_bar;
            arm();
            armAction();
            sleep(1000);
            desired_claw_position = CLAW_OPEN;
            armAction();
            sleep(1000);
            drive(0.0, 0.5, 0.0);
            driveAction(MS_FOR_BACKDRIVE);
            //lower sliders and arm
            state = armState.RESTING;
            arm();
            armAction();
            sleep(100);
            drive(-0.5, 0.0,0.0);
            driveAction(2000);
            //grab block A
            //turn 180 degrees
            //raise sliders and tilt arm
            //drive to the basket
            //rotate to match basket orientation
            //open claw
            //back up
            //lower sliders
            //drive to submersible
            //park
            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            frontLeftMotor.setPower(0.0);
            backLeftMotor.setPower(0.0);
            frontRightMotor.setPower(0.0);
            backRightMotor.setPower(0.0);
            SliderLeft.setPower(0.0);
            SliderRight.setPower(0.0);
            Shoulder.setPower(0.0);
            // Wait for 5 seconds
            sleep(30000);

            telemetry.update();
        }
    }
    //sets shoulder motor position need the right presets
    public void arm(){
        telemetry.addData("state", String.valueOf(state));
        switch (state){
            case RESTING:
                desired_shoulder_position = resting_position;
                desired_shoulder_velocity = RESTING_VELOCITY;
                desired_slider_position = slidersdown;
                desired_slider_velocity = Slidervelocitydown;
                break;
            case BASKET:
                desired_shoulder_position = basket_position;
                desired_shoulder_velocity = BASKET_VELOCITY;
                //desired_wrist_position = wristpos_basket;
                desired_slider_position = slidersup;
                desired_slider_velocity = Slidervelocityup;
                break;
            case SPECIMEN:
                desired_shoulder_position = specimen_position;
                desired_shoulder_velocity = SPECIMEN_VELOCITY;
                //desired_wrist_position = wristpos_specimen;
                desired_slider_position = slidersdown;
                desired_slider_velocity = Slidervelocitydown;
                break;
            case COLLECTION:
                desired_shoulder_position = collection_position;
                desired_shoulder_velocity = COLLECTION_VELOCITY;
                //desired_wrist_position = wristpos_collection;
                desired_slider_position = slidersdown;
                desired_slider_velocity = Slidervelocitydown;
                break;
            case above_bar:
                desired_shoulder_position = shoulder_bar_position;
                desired_shoulder_velocity = shoulder_bar_velotity;
                //desired_wrist_position = wrist_bar_position;
                desired_slider_position = slider_above_bar_position;
                desired_slider_velocity = Slidervelocityup;
                break;
            case below_bar:
                desired_shoulder_position = shoulder_bar_position;
                desired_shoulder_velocity = shoulder_bar_velotity;
                //desired_wrist_position = wrist_bar_position;
                desired_slider_position = slider_below_bar_position;
                desired_slider_velocity = Slidervelocitydown;
                break;
        }
    }
    public void armAction() {
        wrist.setPosition(desired_wrist_position);
        claw.setPosition(desired_claw_position);
        Shoulder.setTargetPosition(desired_shoulder_position);
        Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Shoulder.setVelocity(desired_shoulder_velocity);
        SliderLeft.setTargetPosition(desired_slider_position);
        SliderRight.setTargetPosition(desired_slider_position);
        SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderLeft.setVelocity(desired_slider_velocity);
        SliderRight.setVelocity(desired_slider_velocity);



    }
    public void driveAction(int time_in_ms) {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        sleep(time_in_ms);

        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }
    public void drive(double x, double y, double rx){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
    }
    public void driveActionDistance(double centimeters) {
        Pose2D position = pinpoint.getPosition();
        double startingY = position.getY(DistanceUnit.CM);
        while(Math.abs(startingY - position.getY(DistanceUnit.CM)) > Math.abs(centimeters)) {
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }



        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }
}