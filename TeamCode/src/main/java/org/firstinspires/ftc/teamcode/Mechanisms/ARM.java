//Package
package org.firstinspires.ftc.teamcode.Mechanisms;

//Import
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="ARM", group="Linear OpMode")
public class ARM extends LinearOpMode {

    double armState = 0; // Driver Controlled

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor armExtend       = hardwareMap.dcMotor.get("armExtend");
        DcMotor armLift         = hardwareMap.dcMotor.get("armLift");

        double extendPos = armExtend.getCurrentPosition();
        double liftPos = armLift.getCurrentPosition();

        int extendTarget = 4500;
        int liftTarget = 4700;
        int extendHome = 250;
        int liftHome = 50;

        boolean G1Y_Last = false;
        boolean G1Y = false;

        boolean G1X_Last = false;
        boolean G1X = false;
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLift.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            G1Y_Last = G1Y;
            G1Y = gamepad1.y;
            if (G1Y && !G1Y_Last) {
                armState = 1;
            }

            G1X_Last = G1X;
            G1X = gamepad1.x;
            if (G1X && !G1Y_Last) {
                armState = 2;
            }



            if (gamepad1.a) {
                armExtend.setPower(0);
                armLift.setPower(0);
                armState = 0;
                armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }



            if (armState == 0) {
                armExtend.setPower(0);
                armLift.setPower(0);
            } else if (armState == 1) {
                armExtend.setTargetPosition(extendTarget);
                armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armExtend.setPower(1);
                armLift.setTargetPosition(liftTarget);
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift.setPower(1);
                armState = -1;
            } else if (armState == 2) {
                armExtend.setTargetPosition(extendHome);
                armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armExtend.setPower(1);
                armLift.setTargetPosition(liftHome);
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift.setPower(1);
                armState = -1;
            }
                if (armState == -1) {
                    if (!armExtend.isBusy() && !armLift.isBusy()) {
                        armExtend.setPower(0);
                        armLift.setPower(0);
                        armState = 0;
                        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                }


            telemetry.addData("armExtend", extendPos);
            telemetry.addData("armLift", liftPos);
            telemetry.update();
        }
    }
}
