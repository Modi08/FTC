package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@Autonomous(name = "autoBasicFar")
public class autoBasicFar extends LinearOpMode{

    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    ElapsedTime timer = new ElapsedTime();
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor armMotor;
    public DcMotor leftSlider;
    public DcMotor rightSlider;
    public DcMotor topSlider;
    public CRServo intake;
    public Servo wrist;
    public Servo basket;


    final double armTicksPerDegree = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    final double motorTicksPerDegree = 28 * (3591.0 / 187.0) * 1/360.0;

    //Arm position Constants
    final double armClearScore = 200 * armTicksPerDegree;
    final double armWinchRobot = 20 * armTicksPerDegree;

    //Motor Constants
    final double wheelCircumference = 10.4;

    //Different motor speeds
    final double intakeCollect = -1.0;
    final double intakeOff = 0.0;
    final double intakeDeposit = 1;

    //Wrist position constants
    final double wristFoldedIn = 1;
    final double wristFoldedOut = 0.25;

    //Basket position constants
    final double basketStore = 0.1;
    final double basketScore = 0.55;

    //Time constants
    final long timeLimitHorizontalSliders = 750;
    final long timeExtendVerticalSliders = 1400;
    //Arm Movements
    double armPosition = 0;


    @Override
    public void runOpMode() throws InterruptedException{

        leftDrive = hardwareMap.get(DcMotor.class, "motorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
        armMotor = hardwareMap.get(DcMotor.class, "motorArm");
        leftSlider = hardwareMap.get(DcMotor.class, "sliderLeft");
        rightSlider = hardwareMap.get(DcMotor.class, "sliderRight");
        topSlider = hardwareMap.get(DcMotor.class, "sliderTop");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "IMU");
        gyro = navxMicro;

        //Configuring DriveTrain
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftSlider.setDirection(DcMotor.Direction.FORWARD);
        rightSlider.setDirection(DcMotor.Direction.FORWARD);
        topSlider.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setDirection(DcMotor.Direction.REVERSE);


        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Safety ALERT
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        //Configuring Slider
        rightSlider.setTargetPosition(0);
        rightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Configure Drive
        leftDrive.setTargetPosition(0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        //Configuring Arm
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Initializing Servos
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        basket = hardwareMap.get(Servo.class, "basket");

        //Configuring Servos
        intake.setPower(intakeOff);
        basket.setPosition(basketStore);
        wrist.setPosition(wristFoldedIn);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            waitFor(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();
        telemetry.log().clear();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("started again");
            moveBySetDistance(205, true);
            scoreSample();
            moveBySetDistance(400, false);
            waitFor(30000);
            telemetry.update();
        }
    }


    private void moveBySetDistance(double distance, boolean isReverse) {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        double currentTicks = leftDrive.getCurrentPosition();
        double targetTicks = 0;
        if (isReverse) {
            targetTicks = currentTicks - (motorTicksPerDegree * 360 * (distance / wheelCircumference)*0.33);
        } else {
            targetTicks = currentTicks + (motorTicksPerDegree * 360 * (distance / wheelCircumference)*0.25);
        }
        leftDrive.setTargetPosition((int) targetTicks);
        rightDrive.setTargetPosition((int) targetTicks);

        while ((leftDrive.getCurrentPosition() < leftDrive.getTargetPosition() && !isReverse) || (leftDrive.getCurrentPosition() > leftDrive.getTargetPosition() && isReverse)) {
            telemetry.addData("targetingPosition", leftDrive.getTargetPosition());
            telemetry.addData("currentPosition", leftDrive.getCurrentPosition());
            if (isReverse) {
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else {
                leftDrive.setPower(-1);
                rightDrive.setPower(-1);
            }
            telemetry.update();
        }
        stopDriveMotors();
        leftDrive.setTargetPosition(0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void collectSample() {

        wrist.setPosition(wristFoldedOut);
        intake.setPower(intakeCollect);
        extendHorizontalSliders();
        waitFor(2000);
        intake.setPower(intakeOff);
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(wristFoldedIn);


        leftSlider.setPower(1);
        rightSlider.setPower(1);
        waitFor(timeLimitHorizontalSliders);
        leftSlider.setPower(0);
        rightSlider.setPower(0);
        intake.setPower(intakeDeposit);
        waitFor(1500);
        intake.setPower(intakeOff);

    }

    private void extendHorizontalSliders() {
        rightSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (rightSlider.getCurrentPosition() < 1600) {
            leftSlider.setPower(-0.7);
            rightSlider.setPower(-0.7);
        }

        leftSlider.setPower(0);
        rightSlider.setPower(0);
    }

    private void scoreSample() {
        armMotor.setTargetPosition((int) armClearScore);
        while (armMotor.getCurrentPosition() != armMotor.getTargetPosition()) {

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()){
                break;
            }
        }

        extendVerticalSlider();
        basket.setPosition(basketScore);
        waitFor(1000);
        basket.setPosition(basketStore);
        retractVerticalSlider();
        armMotor.setTargetPosition(0);
        while (armMotor.getCurrentPosition() != armMotor.getTargetPosition()) {

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()){
                break;
            }
        }
    }

    private void extendVerticalSlider() {
        telemetry.addData("right bumper", gamepad1.right_bumper);
        stopDriveMotors();
        topSlider.setPower(1);
        waitFor(timeExtendVerticalSliders);
        topSlider.setPower(0.2);
    }

    private void retractVerticalSlider() {
        stopDriveMotors();
        topSlider.setPower(-1);
        waitFor(timeExtendVerticalSliders);
        topSlider.setPower(0);


    }

    private void stopDriveMotors() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void waitFor(long time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            telemetry.addLine("FAIL");
        }
    }

    private void turnRobot(double targetAngle, boolean isDirectionRight) {
        double tolerance = 1; // Allow 1 degree of error
        while (Math.abs(targetAngle - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > tolerance) {
            telemetry.addData("angle", Math.abs(targetAngle - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));
            double error = targetAngle - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("error", error);
            double power = error * 0.01; // Proportional control (adjust 0.01 as needed)
            telemetry.addData("power", power);
            // Set motor powers (assuming you have left and right motors)
            if (isDirectionRight) {
                leftDrive.setPower(-power);
                rightDrive.setPower(power);
            } else {
                telemetry.addData("power1", power);
                leftDrive.setPower(power);
                rightDrive.setPower(-power);
            }
            telemetry.update();
        }
    }
}
