
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


//intake1+ = outtake
@Autonomous(name="Working Auto 10/31 (Ethan)", group="Linear OpMode")
public class BasicAutoCentric2 extends LinearOpMode {


    //Setup IMU
    IMU imu;
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor shortArm;
    private DcMotor longArm;
    //private CRServo intake1;
    //private CRServo intake2;
    //armExtend.setDirection(DcMotorSimple.Direction.REVERSE); //Might not be needed; test on Tuesday

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
       // intake1 = hardwareMap.get(CRServo.class, "intake1");
       // intake2 = hardwareMap.get(CRServo.class, "intake2");
       shortArm = hardwareMap.get(DcMotor.class, "shortArm");
       longArm = hardwareMap.get(DcMotor.class, "longArm");

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
        double ticksPerInch = 29.8; // MAIN based on wheel sizes and motor rp
        //double ticksPerInch = 41.7; // TESTBOT based on wheel sizes and motor rpm
        double driveSpeed = 0.3;    // can be changed
        stopDrive();
        driveLeft(driveSpeed, 6* ticksPerInch);

        drive(driveSpeed, 54* ticksPerInch);
        driveLeft(driveSpeed, -6* ticksPerInch);
        drive(driveSpeed, -50* ticksPerInch);

        drive(driveSpeed, 50* ticksPerInch);
        driveLeft(driveSpeed, -6* ticksPerInch);
        drive(driveSpeed, -50* ticksPerInch);

        drive(driveSpeed, 50* ticksPerInch);
        driveLeft(driveSpeed, -6* ticksPerInch);
        drive(driveSpeed, -50* ticksPerInch);

        //Intake()
    //rotate(-130);
    //extendArm()
    //release
    }
}
