//Package
package org.firstinspires.ftc.teamcode;

//Import
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

//Name Program
@TeleOp(name="Extension Testing", group="Linear OpMode")

//Initialization Code
public class WristExtentionTesting extends LinearOpMode {

    double automation = 0;
    double automationStep = 1;
    double currentAutoTarget = 0;
    double armStartPos = 1000000;

    @Override
    public void runOpMode() throws InterruptedException {



        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {



            //Setup DcMotor armExtend
            DcMotor armExtend = hardwareMap.dcMotor.get("armExtend");



            //Setup armExtendPosition to Monitor the Extension Value of the Arm in Encoder ticks
            double armExtendPosition = armExtend.getCurrentPosition();



            // Automation Code
            if (gamepad2.y && automation != 0) {
                automation = 0;
            } else if (automation == 0) {
                if (gamepad2.a) {
                    automation = 1;
                    automationStep = 1;
                } else if (gamepad2.b) {
                    automation = 2;
                    automationStep = 1;
                } else if (gamepad2.x) {
                    automation = 3;
                    automationStep = 1;
                }
            }

            telemetry.addData("aep", armExtendPosition);
            telemetry.addData("aut", automation);

            telemetry.update();

            // Arm Extension Value
            if (automation == 0) {
                armExtend.setPower(0);
                if (armExtendPosition > 100) {
                }
            }
            if (automation == 1) {
                if (automationStep == 1) {
                    currentAutoTarget = 500;
                }
                if (armStartPos == 1000000) {
                    armStartPos = armExtendPosition;
                }
                if (armStartPos > currentAutoTarget) {
                    armExtend.setPower((currentAutoTarget + 50 - armExtendPosition) / 200);
                } else {
                    armExtend.setPower((currentAutoTarget - 25 - armExtendPosition) / 200);
                }
                if (Math.abs(armExtendPosition - currentAutoTarget) < 15) {
                    armExtend.setPower((currentAutoTarget + 10 - armExtendPosition) / 400);

                    if (armExtendPosition - currentAutoTarget < 5){
                        automation = 0;
                        armStartPos = 1000000;
                    }
                    armExtend.setPower((currentAutoTarget + 10 - armExtendPosition) / 400);
                    automation = 0;
                    armStartPos = 1000000;
                }
            }
        }
    }
}
