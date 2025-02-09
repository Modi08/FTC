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
    final double armClosedRobot = 0;
    final double armHangHook = 120 * armTicksPerDegree;
    final double armWinchRobot = 15 * armTicksPerDegree;

    //Different motor speeds
    final double intakeCollect = -1.0;
    final double intakeOff = 0.0;
    final double intakeDeposit = 0.5;

    //Wrist position constants
    final double wristFoldedIn = 0.4;
    final double wristFoldedOut = 0.065;

    //Basket position constants
    final double basketStore = 0;
    final double basketScore = 0.5;

    //TODO Understand the following
    final double FudgeFactor = 15 * armTicksPerDegree;

    //Arm Movements
    double armPosition = (int)armClosedRobot;
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
        intake = hardwareMap.get(CRServo.class, "Servo1");
        wrist = hardwareMap.get(Servo.class, "wrist");
        basket = hardwareMap.get(Servo.class, "basket");

        //Configuring Servos
        intake.setPower(intakeOff);
        wrist.setPosition(wristFoldedIn);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controlDriveMotors();

            controlHorizontalSliders();

            controlVerticalSlider();
            //Retract top slider
            if (gamepad2.b) {
                retractVerticalSlider();
            }
            //Collect Sample
            if (gamepad1.a) {
                collectSample();
            }

            //Toggling Arm Position
            if (gamepad2.dpad_up){
                armPosition = armHangHook;
                intake.setPower(intakeOff);
                wrist.setPosition(wristFoldedIn);
            } else if (gamepad2.dpad_down){
                armPosition = armWinchRobot;
                intake.setPower(intakeOff);
                wrist.setPosition(wristFoldedIn);
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
            telemetry.update();

        }
    }

    private void collectSample() {
        boolean run = true;
        while (run) {
            wrist.setPosition(wristFoldedOut);
            intake.setPower(intakeCollect);
            if (gamepad1.a) {
                run = false;
                intake.setPower(intakeOff);
                wrist.setPosition(wristFoldedIn);
            }   
        }

        leftSlider.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlider.setDirection(DcMotorSimple.Direction.REVERSE);
        while (!((DcMotorEx) leftSlider).isOverCurrent() || !((DcMotorEx) rightSlider).isOverCurrent()) {
            leftSlider.setPower(1);
            rightSlider.setPower(1);
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
        while (gamepad2.left_trigger > 0) {
            double power = gamepad2.left_trigger;
            leftSlider.setPower(power);
            rightSlider.setPower(power);
        }
    }

    private void controlVerticalSlider() {
        while (gamepad2.right_trigger > 0) {
            double power = gamepad2.left_trigger;
            leftSlider.setPower(power);
            rightSlider.setPower(power);
        }
    }

    private void retractVerticalSlider() {
        while (!((DcMotorEx) topSlider).isOverCurrent()) {
            topSlider.setPower(1);
        }
    }
}