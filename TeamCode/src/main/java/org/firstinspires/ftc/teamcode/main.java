package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "main")
public class main extends LinearOpMode {

    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor armMotor;
    public CRServo intake;
    public Servo wrist;

    // Can be used convert degrees to ticks
    final double armTicksPerDegree = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;

    //Arm position Constants
    final double armClosedRobot = 0;
    final double armCollect = 250 * armTicksPerDegree;
    final double armClearBarrier = 230 * armTicksPerDegree;
    final double armScoreSpec = 160 * armTicksPerDegree;
    final double armScoreSampleLow = 160 * armTicksPerDegree;
    final double armHangHook = 120 * armTicksPerDegree;
    final double armWinchRobot = 15 * armTicksPerDegree;

    //Different motor speeds
    final double intakeCollect = -1.0;
    final double intakeOff = 0.0;
    final double intakeDeposit = 0.5;

    //Wrist position constants
    final double wristFoldedIn = 0.8333;
    final double wristFoldedOut = 0.5;

    //TODO Understand the following
    final double FudgeFactor = 15 * armTicksPerDegree;

    //Arm Movements
    double armPosition = (int)armClosedRobot;
    double armPositionFudgeFactor;

    @Override
    public void runOpMode() throws InterruptedException {
        //Drive train Variables
        double leftPower;
        double rightPower;
        double forward;
        double rotate;
        double max;

        //Initializing Motors
        leftDrive = hardwareMap.get(DcMotor.class, "motor1");
        rightDrive = hardwareMap.get(DcMotor.class, "motor2");
        armMotor = hardwareMap.get(DcMotor.class, "motor3");

        //Configuring DriveTrain
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Safety ALERT
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        //Configuring Arm
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Initializing Servos
        intake = hardwareMap.get(CRServo.class, "Servo1");
        wrist = hardwareMap.get(Servo.class, "wrist");

        //Configuring Servos
        intake.setPower(intakeOff);
        wrist.setPosition(wristFoldedIn);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //Getting joystick inputs
            forward = -gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;

            //Calculate Power to wheels
            leftPower = (forward + rotate) * 0.9;
            rightPower = (forward - rotate) * 0.9;
            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0)
            {
                leftPower /= max;
                rightPower /= max;
            }

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            //Toggling Intake Power
            if (gamepad1.a) {
                intake.setPower(intakeCollect);
            }
            else if (gamepad1.x) {
                intake.setPower(intakeOff);
            }
            else if (gamepad1.b) {
                intake.setPower(intakeDeposit);
            }


            //Toggling Arm Position
            if(gamepad1.right_bumper){
                armPosition = armCollect;
                wrist.setPosition(wristFoldedOut);
                intake.setPower(intakeCollect);
            }

            else if (gamepad1.left_bumper){
                armPosition = armClearBarrier;
            }

            else if (gamepad1.y){
                /* This is the correct height to score the sample in the LOW BASKET */
                armPosition = armScoreSampleLow;
            }

            else if (gamepad1.dpad_left) {
                armPosition = armClosedRobot;
                intake.setPower(intakeOff);
                wrist.setPosition(wristFoldedIn);
            }

            else if (gamepad1.dpad_right){
                armPosition = armScoreSpec;
                wrist.setPosition(wristFoldedOut);
            }

            else if (gamepad1.dpad_up){
                armPosition = armHangHook;
                intake.setPower(intakeOff);
                wrist.setPosition(wristFoldedIn);
            }

            else if (gamepad1.dpad_down){
                armPosition = armWinchRobot;
                intake.setPower(intakeOff);
                wrist.setPosition(wristFoldedIn);
            }

            //Safety Precautions
            armPositionFudgeFactor = FudgeFactor * (gamepad1.right_trigger + (-gamepad1.left_trigger));

            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }


            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}