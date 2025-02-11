package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

    //TODO Understand the following
    final double FudgeFactor = 15 * armTicksPerDegree;

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
            leftSlider.setPower(0.5);

            rightSlider.setPower(0.5);
            Thread.sleep((long) (1500));
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
        telemetry.addData("left trigger", gamepad1.left_trigger);
        while (gamepad1.left_trigger > 0) {
            leftSlider.setPower(-0.7);
            rightSlider.setPower(-0.7);
        }
        leftSlider.setPower(0);
        rightSlider.setPower(0);
    }

    private void controlVerticalSlider() {
        boolean up = false;
        telemetry.addData("right trigger", gamepad1.right_trigger);
        while (gamepad1.right_trigger > 0) {
            topSlider.setPower(0.5);
            up = true;
        }
        if (up) {
            topSlider.setPower(0.2);

        }
    }

    private void retractVerticalSlider() {
        try {
            topSlider.setPower(-0.5);
            Thread.sleep((long) (2200));
            topSlider.setPower(0);
        }catch (InterruptedException e) {
            telemetry.addLine("FAIL");
        }
    }
}