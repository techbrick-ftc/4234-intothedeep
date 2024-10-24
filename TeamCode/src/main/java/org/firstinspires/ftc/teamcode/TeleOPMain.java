//Package
package org.firstinspires.ftc.teamcode;


//Import
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


//Name program
@TeleOp(name="[MAIN TEST CODE] 10/23/2024 - TeleOP", group="Linear OpMode")


//Program
public class TeleOPMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        //Setup Motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
        DcMotor armExtend = hardwareMap.dcMotor.get("armextend");
        DcMotor armLift = hardwareMap.dcMotor.get("armlift");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        //Setup IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);


        //Loop code
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {


            //Position control inputs
            double y = -gamepad1.left_stick_y; //Y value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            //Arm control inputs
            double ae = gamepad2.right_stick_x; //Input values for arm controls
            double al = gamepad2.right_stick_y/2; //Input lift value of the arm


            //Limits, variables, and constants
            double driveSpeed = 1;
            double aep = armExtend.getCurrentPosition(); //Current position of arm
            double alp = armLift.getCurrentPosition(); //Current lift value of the arm
            double aepMax = 1000; //Maximum extension value of the arm, in encoder ticks
            double alMax = 200; //Maximum lift value of the arm, in encoder ticks
            double alMin = 200; //Minimum lift value of the arm, in encoder ticks
            double liftModifier = 0;
            if ((armExtend.getCurrentPosition() < 30) && (al > 0)) {liftModifier = 1;} else if (al < 0) {liftModifier = 1;} else {liftModifier = 0;} //To prevent arm collision issues


            //Log telemetry data
            telemetry.addData("Arm Extend", aep);
            telemetry.addData("Arm Lift", armLift.getCurrentPosition());


            //Allows player to reset IMU
            if (gamepad1.options) {imu.resetYaw();}


            //Modes
            if (gamepad1.left_bumper) {driveSpeed = .5;} //Toggle slow mode with left bumper


            //Field centric
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            //Motors
            if (ae > 0 && aep < aepMax) {armExtend.setPower(ae);} else if (ae < 0 && aep > 0) {armExtend.setPower(ae);} else {armExtend.setPower(0);}
            if (al > 0 && alp < alMax) {armLift.setPower(al * liftModifier);} else if (al < 0 && alp > alMin) {armLift.setPower(al * liftModifier);} else {armLift.setPower(0);}
            frontLeftMotor.setPower(frontLeftPower * driveSpeed);
            backLeftMotor.setPower(backLeftPower * driveSpeed);
            frontRightMotor.setPower(frontRightPower * driveSpeed);
            backRightMotor.setPower(backRightPower * driveSpeed);


            //Update telemetry
            telemetry.update();
        }
    }
}


//Developed by Team 4234 with help from GM0, for usage in the 2024 - 2025 FTC challenge Into the Deep
//Debugged with help from ChatGPT
//<3