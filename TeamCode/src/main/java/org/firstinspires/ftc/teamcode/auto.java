
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import java.lang.Math;

//intake1+ = outtake
@Autonomous(name="124", group="Linear OpMode")
public class auto extends LinearOpMode {


    //Setup IMU
    IMU imu;
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor shortArm;
    private DcMotor longArm;
    public Servo wrist1;
    public Servo wrist2;
    public CRServo intake1;
    public CRServo intake2;
    public Servo winch;
    public Servo specimen;
    private DcMotor armExtend;
    private DcMotor armLift;
    double armState = 1;

    //private CRServo intake1;
    //private CRServo intake2;
    //armExtend.setDirection(DcMotorSimple.Direction.REVERSE); //Might not be needed; test on Tuesday

    double armRotationsPerInch = 0.5;
    public void stopDrive() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void armTO(int liftPosTo, int extendPosTo, float speed, float wrist) {

        armLift.setTargetPosition(liftPosTo);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setTargetPosition(extendPosTo);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(speed * 1.25 / 10);
        armExtend.setPower(speed / 10);
        armState = -1; // Automated action in progress
        wrist1.setPosition(1 - (wrist / 10));
        wrist2.setPosition(wrist / 10);
    }
    public void drive(double speed, double distance) {
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        while (Math.abs(frontLeft.getCurrentPosition())<distance && opModeIsActive()) {
            sleep(1);
        }
        stopDrive();
    }

    public void driveLeft(double speed, double distance) {
        frontLeft.setPower(0-speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(0-speed);
        while (Math.abs(frontLeft.getCurrentPosition())<distance && opModeIsActive()) {
            sleep(1);
        }
        stopDrive();
    }

    public void arm(double speed, double distance) {
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        while (Math.abs(frontLeft.getCurrentPosition())<distance && opModeIsActive()) {
            sleep(1);
        }
        stopDrive();
    }

    public void correct(double speed, double distance) {
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        while (Math.abs(frontLeft.getCurrentPosition())<distance && opModeIsActive()) {
            sleep(1);
        }
        stopDrive();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        armLift = hardwareMap.get(DcMotor.class, "armLift");
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        wrist1 = hardwareMap.get(Servo.class, "wrist1");
        wrist2 = hardwareMap.get(Servo.class, "wrist2");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        winch = hardwareMap.get(Servo.class, "winch");
        specimen = hardwareMap.get(Servo.class, "specimen");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        /* DRIVE CODE */
        double ticksPerInch = 29.8; // MAIN based on wheel sizes and motor rp
        double reversed = 1;
        //double ticksPerInch = 41.7; // TEST/BOT based on wheel sizes and motor rpm
        double driveSpeed = 0.3;    // can be changed
        stopDrive();
        drive(-driveSpeed, 5 * ticksPerInch);
        armTO(0, 0, 0, 0);
        driveLeft(-driveSpeed * reversed, 9* ticksPerInch);

        drive(driveSpeed, 50* ticksPerInch);
        driveLeft(-driveSpeed * reversed, 14* ticksPerInch);
        drive(-driveSpeed, 42* ticksPerInch);

        correct(-driveSpeed, 0);
        drive(driveSpeed, 46* ticksPerInch);
        driveLeft(-driveSpeed * reversed, 14* ticksPerInch);
        drive(-driveSpeed, 46* ticksPerInch);
        correct(-driveSpeed, 0);
//        drive(driveSpeed, 48* ticksPerInch);
        //      driveLeft(-driveSpeed * reversed, 18* ticksPerInch);
        //    drive(-driveSpeed, 48* ticksPerInch);

        //Intake()
        //rotate(-130);
        //extendArm()
        //release
    }
}
