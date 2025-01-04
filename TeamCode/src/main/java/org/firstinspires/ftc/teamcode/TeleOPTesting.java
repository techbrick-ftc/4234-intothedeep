package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.lang.Math;

@TeleOp(name="[In Testing] TeleOP")
@Disabled
public class TeleOPTesting extends LinearOpMode {

    // Servo Declarations
    public Servo wrist1;
    public Servo wrist2;
    public CRServo intake1;
    public CRServo intake2;
    public Servo winch;


    @Override
    public void runOpMode() throws InterruptedException {


        // Motor Declarations
        DcMotor fl = hardwareMap.dcMotor.get("frontLeft");
        DcMotor bl = hardwareMap.dcMotor.get("backLeft");
        DcMotor fr = hardwareMap.dcMotor.get("frontRight");
        DcMotor br = hardwareMap.dcMotor.get("backRight");
        DcMotor winchMotor = hardwareMap.dcMotor.get("winchs");
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        DcMotor armExtend = hardwareMap.dcMotor.get("armExtend");
        DcMotor armLift = hardwareMap.dcMotor.get("armLift");
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        armLift.setDirection(DcMotorSimple.Direction.REVERSE);


        // Servo Declarations
        wrist1 = hardwareMap.get(Servo.class, "wrist1"); // Setup servos
        wrist2 = hardwareMap.get(Servo.class, "wrist2");
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        winch = hardwareMap.get(Servo.class, "winch");


        // IMU
        IMU imu = hardwareMap.get(IMU.class, "imu"); // This allows for field centric
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, // Orientation of the control hub
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)); //Orientation of the control hub
        imu.initialize(parameters);


        // Variables
        double info = 0; // status info
        double driveModifier = 1; // Slow mode. 1 = full speed, 0.5 = half speed, 0.25 = quarter speed, ect.
        double armLiftMin = -100; // Minimum lift value of the arm
        double armLiftMax = 4500; // Maximum lift value of the arm
        double armExtendMax = 5020; // Maximum extension value of the arm
        double automation = 0; // Current automation
        double automationStep = 0; // Current automation step
        boolean fieldCentricActive = true;
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;
        double denominator = 1;
        boolean lastOptions = gamepad1.options;
        boolean options = false;
        boolean autoStopRequested = false;



        // Status
        telemetry.addData("Status", "Waiting for start");
        telemetry.update(); // Push status to driver station


        // Loop Code
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {


            // Chassis Movement

            // Field Centric Toggle
            lastOptions = options;
            options = gamepad1.options;
            if (options && !lastOptions) {
                fieldCentricActive = !fieldCentricActive;
            }


            // Drive Inputs
            double xP = -gamepad1.left_stick_x;
            double yP = -gamepad1.left_stick_y;
            double rP = gamepad1.right_stick_x;
            double slowMode = gamepad1.left_bumper ? .4 : 1;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = xP * Math.cos(-botHeading) - yP * Math.sin(-botHeading);
            double rotY = xP * Math.sin(-botHeading) + yP * Math.cos(-botHeading);
            rotX = rotX * 1.1;


            // Motor Power Calculations
            if (fieldCentricActive) {
                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rP), 1);
                frontLeftPower      =     (rotY + rotX + rP) / denominator;
                backLeftPower       =     (rotY - rotX + rP) / denominator;
                frontRightPower     =     (rotY - rotX - rP) / denominator;
                backRightPower      =     (rotY + rotX - rP) / denominator;
            } else {
                denominator = Math.max(Math.abs(yP) + Math.abs(xP) + Math.abs(rP), 1);
                frontLeftPower      =     (yP + xP + rP) / denominator;
                backLeftPower       =     (yP - xP + rP) / denominator;
                frontRightPower     =     (yP - xP - rP) / denominator;
                backRightPower      =     (yP + xP - rP) / denominator;            }


            //Arm and Intake Inputs
            double extendPower = -gamepad2.left_stick_y;
            double liftPower = -gamepad2.right_stick_y;
            double extendPos = armExtend.getCurrentPosition();
            double liftPos = armLift.getCurrentPosition();
            double armLiftRotation = (liftPos / 5000.0);
            double extendMax42 = (Math.cos((armLiftRotation) * (Math.PI / 2)) <= 0.01) ? Double.POSITIVE_INFINITY : 3000 / Math.cos((liftPos / 5000.0) * (Math.PI / 2)); // Calculates the arm extension limit to prevent teh arm exceeding the 42" limit
            double extendPowerOffset = ((extendPos - extendMax42) / 250);
            double liftPowerOffset =  1 - ((extendMax42 - extendPos) / 125);


            //Automation Inputs
            if ((gamepad1.y || gamepad2.y) && automation != 0) {
                autoStopRequested = true;
                info = 3;
            } else if (automation == 0) {
                if (gamepad1.a || gamepad2.a) {
                    automation = 1;
                    info = 2;
                } else if (gamepad1.b || gamepad2.b) {
                    automation = 2;
                    automationStep = 1;
                    info = 2;
                } else if (gamepad1.x || gamepad2.x) {
                    automation = 3;
                    automationStep = 1;
                    info = 2;
                }
            }


            // Reset IMU
            if (gamepad1.start || gamepad2.start) {
                imu.resetYaw();
            }

            // Drive Power
            fl.setPower(frontLeftPower * slowMode);
            bl.setPower(backLeftPower * slowMode);
            fr.setPower(frontRightPower * slowMode);
            br.setPower(backRightPower * slowMode);



            // Arm Power
 //           if (automation == 0) { // == Driver controlled
//
  //              // This subregion contains the code for the arm extension while in the driver controlled mode
   //             //region Arm Extend
//                if (extendPos > 150) { // Prevents arm hitting lower limit
//                    if (liftPos > 4500 || extendPos + 250 > extendMax42) { // Is arm extended too far?
//                        if (extendPos + 250 > extendMax42 && extendPos > 2500) { //Is arm at risk of hitting the horizontal expansion limit?
//                            armExtend.setPower(extendPower - extendPowerOffset);
//                        } else if (extendPos > armExtendMax) {
//                            if (extendPower > 0) {
//                                armExtend.setPower(0);
//                            } else {
//                                armExtend.setPower(extendPower);
//                            }
//                        }
//                    } else if (extendPos + 100 > armExtendMax) {
//                        if (extendPower > 0) {
//                            armExtend.setPower(0);
//                        } else {
//                            armExtend.setPower(extendPower);
//                        }
//                    } else {
//                        armExtend.setPower(extendPower);
//                    }
//                } else {
//                    if (extendPower > 0) {
//                        armExtend.setPower(extendPower);
//                    } else {
//                        if (extendPos > 50) {
//                            armExtend.setPower(Math.min(-.25, extendPower));
//                        } else if (extendPos > 25) {
//                            armExtend.setPower(0);
//                        } else {
//                            armExtend.setPower((25 - extendPos) / 100);
//                        }
//                    }
//                }
                //endregion //


                if (automation == 0) {


                    // Arm Rotation
                    if ((liftPower > 0 && liftPos < armLiftMax) || (liftPower < 0 && liftPos > armLiftMin)) {
                        armLift.setPower(liftPower);
                    } else {
                        armLift.setPower(0);
                    }


                    // Arm Extension
                    if ((extendPower > 0 && !(extendPos + 250 > extendMax42 && extendPos < 2500 ) && extendPos < armExtendMax) || extendPower < 0 && extendPos >= 50) {
                        armExtend.setPower(extendPower);
                    } else if (extendPos < 50) {
                        armExtend.setPower(0);
                    } else if (extendPos + 250 > extendMax42 && extendPos > 2500) {
                        armExtend.setPower(extendPower - extendPowerOffset);
                    } else if (extendPos + 100 > armExtendMax) {
                        armExtend.setPower(0);
                    }


                } else if (automation == 1) {

                    armExtend.setTargetPosition(600);
                    armExtend.setPower(1);
                    armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if (!armExtend.isBusy() || autoStopRequested) {
                        automation = 0;
                        info = autoStopRequested ? 3 : 4;
                        autoStopRequested = false;
                        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }


                }


//                if (liftPower > 0 && extendPos < armLiftMax) { // Is arm lowering and above limit
//                    if (Math.abs(extendPos - extendMax42) < 125) {
//                        armLift.setPower(liftPower + liftPowerOffset); // Is arm lowering too fast?
//                    } else {
//                        armLift.setPower(liftPower);
//                    }
//                }
//                else if (liftPower < 0 && liftPos > armLiftMin) { // Is arm rising and below limit?
//                    armLift.setPower(liftPower);
//                }
//                else { // Stop the arm
//                    armLift.setPower(0);
//                }
//                //endregion
//            } else if (automation == 1) {
//                if (Math.abs(extendPos - 1000) > 5) {
//                    armExtend.setPower((1000 - extendPos)/400);
//                } else {
//                    automation = 0;
//                    info = 4;
//                    armExtend.setPower(0);
//                }
//                //endregion
//            } else if (automation == 2) {
//                //Do nothing (FOR NOW!!)
//            } else if (automation == 3) {
//                if (automationStep == 1) {
//                    if (Math.abs(extendPos - 500) < 50) {
//                        armExtend.setPower((500 - extendPos) / 250);
//                    } else {
//                        armExtend.setPower((500 - extendPos) / 150);
//                    }
//                    if (Math.abs(liftPos - 500) < 50) {
//                        armLift.setPower((500 - liftPos) / 250);
//                    } else {
//                        armLift.setPower((500 - liftPos) / 150);
//                    }
//                    if (Math.abs(extendPos - 500) < 25 && Math.abs(liftPos - 100) < 25) {
//                        automation = 0;
//                        info = 4;
//                        armLift.setPower(0);
//                    }
//                }
//            }

            //Servos
            // region Wrist
            if (gamepad2.dpad_down) {
                wrist1.setPosition(.7);
                wrist2.setPosition(.3);
            } else if (gamepad2.dpad_up && liftPos < 3400) {
                wrist1.setPosition(0);
                wrist2.setPosition(1);
            } else if (gamepad2.dpad_left || gamepad2.dpad_right || (liftPos > 3400 && (wrist1.getPosition() == 0))) {
                wrist1.setPosition(.5);
                wrist2.setPosition(.5);
            }
            // endregion Wrist

            // region Intake
            if (gamepad2.left_bumper) { //Outtake
                intake1.setPower(.3);
                intake2.setPower(-.3);
            } else if (gamepad2.left_trigger > .2) { //Intake
                intake1.setPower(-.75);
                intake2.setPower(.75);
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
            }
            // endregion Intake

            //Winch Power
            if (gamepad1.dpad_up) {
                winch.setPosition(1);
            } else if (gamepad1.dpad_down) {
                winch.setPosition(0);
            }
            if (gamepad1.dpad_left) {
                winchMotor.setPower(1);
            } else if (gamepad1.dpad_right) {
                winchMotor.setPower(-1);
            } else {
                winchMotor.setPower(0);
            }
            //Debug Information
            if (info == 0) {
                telemetry.addData("Status", "Initializing - Waiting for Start");
                info = 1;
            } else if (info == 1) {
                telemetry.addData("Status", "Driver Controlled");
            } else if (info == 2) {
                telemetry.addData("Status", "Automation Active. To cancel, press [Y] on P2");
            } else if (info == 3) {
                telemetry.addData("Status", "Automation cancelled manually");
            } else if (info == 4) {
                telemetry.addData("Status", "Auto finished successfully");
            } else if (info == 5) {
                telemetry.addData("Status", "Unspecified Error");
            }
            telemetry.addData("Drive Mode [P1.Options to toggle]", fieldCentricActive ? "Field centric" : "Robot centric");
            telemetry.addData("Arm Extension", extendPos);
            telemetry.addData("Arm Extension %", extendPos / armExtendMax);
            telemetry.addData("Automation", automationStep);
            telemetry.addData("extendMax42", extendMax42);
            telemetry.addData("aepo", extendPowerOffset);
            telemetry.update();
        }
    }
}
