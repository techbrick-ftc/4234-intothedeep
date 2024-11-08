
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;


//intake1+ = outtake
@Autonomous(name="Working Auto 10/31 (Ethan)", group="Linear OpMode")
public class BasicAutoCentric extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    public DcMotor shortArm = hardwareMap.dcMotor.get("shortArm");
    public DcMotor longArm = hardwareMap.dcMotor.get("longArm");
    public CRServo intake1;
    public CRServo intake2;
    //armExtend.setDirection(DcMotorSimple.Direction.REVERSE); //Might not be needed; test on Tuesday


    //Setup IMU
    IMU imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    double armRotationsPerInch = 0.5;
    public void stopDrive() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void drive(double speed, double distance) {
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
        while (Math.abs(frontLeftMotor.getCurrentPosition())<distance && opModeIsActive()) {
            sleep(1);
        }
        stopDrive();
    }
    public void longArm(double speed, double position) {
        if (longArm.getCurrentPosition() > position) {
            longArm.setPower(Math.abs(speed) * -1); // Move in the negative direction
            while (Math.abs(longArm.getCurrentPosition()) > position && opModeIsActive()) {
                sleep(1);
            }
        } else {
            longArm.setPower(Math.abs(speed)); // Move in the positive direction
            while (Math.abs(longArm.getCurrentPosition()) < position && opModeIsActive()) {
                sleep(1);
            }
        }

        longArm.setPower(0);
    }
    public void shortArm(double speed, double position) {
        if (shortArm.getCurrentPosition() > position) {
            shortArm.setPower(Math.abs(speed) * -1); // Move in the negative direction
            while (Math.abs(shortArm.getCurrentPosition()) > position && opModeIsActive()) {
                sleep(1);
            }
        } else {
            shortArm.setPower(Math.abs(speed)); // Move in the positive direction
            while (Math.abs(shortArm.getCurrentPosition()) < position && opModeIsActive()) {
                sleep(1);
            }
        }

        shortArm.setPower(0);
    }
    public void driveLeft(double speed, double distance) {
        frontLeftMotor.setPower(0-speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(0-speed);
        while (Math.abs(frontLeftMotor.getCurrentPosition())<distance && opModeIsActive()) {
            sleep(1);
        }
        stopDrive();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontleft");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backleft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontright");
        backRightMotor = hardwareMap.get(DcMotor.class, "backright");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        /* DRIVE CODE */
//        double ticksPerInch = 29.8; // MAIN based on wheel sizes and motor rp
double ticksPerInch = 41.7; // TESTBOT based on wheel sizes and motor rpm
        double driveSpeed = 0.15;    // can be changed
        stopDrive();
        drive(driveSpeed, 27* ticksPerInch);
        driveLeft(driveSpeed, 24* ticksPerInch);

        drive(driveSpeed, 5*ticksPerInch); // add : raise arm 27 while driving forwards

        sleep(550);
        longArm(10, 5);
        shortArm(10, 5);
        // add : Release specimen

        sleep(550);
        drive(-driveSpeed,5*ticksPerInch);
        driveLeft(-driveSpeed, 24* ticksPerInch);
        drive(-driveSpeed, 27* ticksPerInch);
        //Intake()
        //rotate(-130);
        //extendArm()
        //release
        // }
}
