package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import android.os.SystemClock;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "main")
public class main extends LinearOpMode {

    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor armMotor;
    public DcMotor leftSlider;
    public DcMotor rightSlider;
    public DcMotor topSlider;
    public CRServo intake;
    public Servo wrist;
    public Servo basket;

    // Can be used convert degrees to ticks
    final double armTicksPerDegree = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;

    //Arm position Constants
    final double armClearScore = 120 * armTicksPerDegree;
    final double armHangHook = 90 * armTicksPerDegree;
    final double armWinchRobot = 0;

    //Different motor speeds
    final double intakeCollect = -1.0;
    final double intakeOff = 0.0;
    final double intakeDeposit = 1;

    //Wrist position constants
    final double wristFoldedIn = 0;
    final double wristFoldedOut = 0.15;

    //Basket position constants
    final double basketStore = 0;
    final double basketScore = 0.4;

    //Time constants
    final long timeLimitVerticalSliders = 750;
    long secondPast = 0;
    long startTime = 0L;
    long endTime = 0L;

    //Arm Movements
    double armPosition = (int)armWinchRobot;
    double armPositionFudgeFactor;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initializing Motors
        leftDrive = hardwareMap.get(DcMotor.class, "motorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
        armMotor = hardwareMap.get(DcMotor.class, "motorArm");
        leftSlider = hardwareMap.get(DcMotor.class, "sliderLeft");
        rightSlider = hardwareMap.get(DcMotor.class, "sliderRight");
        topSlider = hardwareMap.get(DcMotor.class, "sliderTop");

        //Configuring DriveTrain
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftSlider.setDirection(DcMotor.Direction.FORWARD);
        rightSlider.setDirection(DcMotor.Direction.FORWARD);
        topSlider.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setDirection(DcMotor.Direction.REVERSE);


        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Safety ALERT
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        ((DcMotorEx) leftSlider).setCurrentAlert(5, CurrentUnit.AMPS);
        ((DcMotorEx) rightSlider).setCurrentAlert(5, CurrentUnit.AMPS);
        ((DcMotorEx) topSlider).setCurrentAlert(5, CurrentUnit.AMPS);

        //Configuring Arm
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Initializing Servos
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        basket = hardwareMap.get(Servo.class, "basket");

        //Configuring Servos
        wrist.setDirection(Servo.Direction.REVERSE);
        intake.setPower(intakeOff);
        wrist.setPosition(wristFoldedIn);
        basket.setPosition(basketStore);


        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controlDriveMotors();

            controlHorizontalSliders();

            controlVerticalSlider();
            //Retract top slider
            if (gamepad1.b) {
                retractVerticalSlider();
            }
            //Collect Sample
            else if (gamepad1.a) {
                collectSample();
            } else if (gamepad1.x) {
                basket.setPosition(basketScore);

            } else if (gamepad1.y) {

                basket.setPosition(basketStore);
            }

            //Toggling Arm Position
            if (gamepad1.dpad_up){
                armPosition = armHangHook;
                intake.setPower(intakeOff);
                wrist.setPosition(wristFoldedIn);
            } else if (gamepad1.dpad_down){
                armPosition = armWinchRobot;
                intake.setPower(intakeOff);
                wrist.setPosition(wristFoldedIn);
            } else if (gamepad1.dpad_left) {
                armPosition = armClearScore;
            }

            //Safety Precautions
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
            telemetry.addData("SliderSpeed: ", topSlider.getPower());
            telemetry.update();

        }
    }

    private void collectSample() {
        boolean run = true;
        while (run) {
            wrist.setDirection(Servo.Direction.FORWARD);
            wrist.setPosition(wristFoldedOut);
            intake.setPower(intakeCollect);
            if (gamepad1.dpad_right) {
                run = false;
                intake.setPower(intakeOff);
                wrist.setDirection(Servo.Direction.REVERSE);
                wrist.setPosition(wristFoldedIn);
            }  }

        try {
            leftSlider.setPower(1);

            rightSlider.setPower(1);
            Thread.sleep((long) (timeLimitVerticalSliders));
            leftSlider.setPower(0);
            rightSlider.setPower(0);
            intake.setPower(intakeDeposit);
            Thread.sleep((long) (1500));
            intake.setPower(intakeOff);
        } catch (InterruptedException e) {
            telemetry.addLine("FAIL");
        }

    }

    private void controlDriveMotors() {
        //Getting joystick inputs
        double forward = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        //Calculate Power to wheels
        double leftPower = (forward + rotate) * 0.9;
        double rightPower = (forward - rotate) * 0.9;
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0)
        {
            leftPower /= max;
            rightPower /= max;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    private void controlHorizontalSliders() {
        boolean isInverted = false;
        telemetry.addData("left trigger", gamepad1.left_trigger);
        telemetry.addData("right trigger", gamepad1.right_trigger);
        while (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0 || secondPast >= timeLimitVerticalSliders) {
            startTimer();
            if (gamepad1.left_trigger > 0) {
                leftSlider.setPower(-1);
                rightSlider.setPower(-1);
            } else {
                leftSlider.setPower(1);
                rightSlider.setPower(1);
                isInverted = true;
            }
        }
        endTimer(isInverted);
        leftSlider.setPower(0);
        rightSlider.setPower(0);
    }

    private void controlVerticalSlider() {
        telemetry.addData("right bumper", gamepad1.right_bumper);
        if (gamepad1.right_bumper) {
            try {
                topSlider.setPower(1);
                Thread.sleep((long) (1100));
                topSlider.setPower(0.2);
            } catch (InterruptedException e) {
                telemetry.addLine("FAIL");
            }
        }
    }

    private void retractVerticalSlider() {
        try {
            topSlider.setPower(-0.5);
            Thread.sleep((long) (2200));
            topSlider.setPower(0);
        } catch (InterruptedException e) {
            telemetry.addLine("FAIL");
        }
    }

    private void startTimer() {
        startTime = SystemClock.uptimeMillis();
    }

    private void endTimer(boolean isInverted) {
        endTime = SystemClock.uptimeMillis();
        if (isInverted) {
            secondPast += endTime - startTime;
        } else {
            secondPast -= endTime - startTime;
        }
    }
}