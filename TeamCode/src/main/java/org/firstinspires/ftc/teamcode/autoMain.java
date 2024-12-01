package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name = "automain")
public class autoMain extends LinearOpMode{

    public HuskyLens lens;
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor armMotor;
    public CRServo intake;
    public Servo wrist;

    final double armTicksPerDegree = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;

    //Arm position Constants
    final double armClosedRobot = 0;
    final double armCollect = 250 * armTicksPerDegree;
    final double armScoreSampleLow = 160 * armTicksPerDegree;

    //Change variable value
    final double DriveTicksPerDegree = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;

    //Drive position Constants
    final double driveVector0 = 0;
    final double driveVector10 = 10 * armTicksPerDegree;
    final double driveVector90 = 90 * armTicksPerDegree;
    final double driveVector180 = 180 * armTicksPerDegree;

    //Different motor speeds
    final double intakeCollect = -1.0;
    final double intakeOff = 0.0;
    final double intakeDeposit = 0.5;

    //Wrist position constants
    final double wristFoldedIn = 0.8333;
    final double wristFoldedOut = 0.5;

    final double secondsPerCentimeter = 4;

    //Arm Movements
    double armPosition = (int)armClosedRobot;
    double armPositionFudgeFactor;

    @Override
    public void runOpMode() throws InterruptedException{
        double leftPower;
        double rightPower;
        HuskyLens.Block[] blocks;
        boolean isPieceFound = false;

        //Initializing HuskyLens
        lens = hardwareMap.get(HuskyLens.class, "lens");
        lens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);

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

            leftDrive.setTargetPosition(0);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightDrive.setTargetPosition(0);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



            while (!isPieceFound) {
                blocks = lens.blocks();
                if (blocks.length > 0) {
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    isPieceFound = true;
                }
                else {
                    leftDrive.setTargetPosition((int) driveVector10);
                    rightDrive.setTargetPosition((int) driveVector10);
                }
            }

            moveToTargetLocation(true);

            armMotor.setTargetPosition((int) (armCollect));
            intake.setPower(intakeCollect);
            wrist.setPosition(wristFoldedOut);

            Thread.sleep(5000);

            intake.setPower(intakeOff);

            lens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
            moveToTargetLocation(false);

            leftDrive.setTargetPosition((int) driveVector90);
            rightDrive.setTargetPosition((int) driveVector90);

            armMotor.setTargetPosition((int) (armScoreSampleLow));
            wrist.setPosition(wristFoldedOut);
            Thread.sleep(5000);
            intake.setPower(intakeDeposit);

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }


    public void moveToTargetLocation(boolean isObjectSample) {
        int targetWidth = isObjectSample ? 80 : 90;
        int targetHeight = isObjectSample ? 80 : 90;

        HuskyLens.Block block = lens.blocks()[0];
        boolean heightCheck = (block.height > (targetHeight - (targetHeight * 0.1))) && (block.height < (targetHeight + (targetHeight * 0.1)));
        boolean widthCheck = (block.width > (targetWidth - (targetWidth * 0.1))) && (block.width < (targetWidth + (targetWidth * 0.1)));

        while (!(widthCheck || heightCheck)) {
            block = lens.blocks()[0];
            heightCheck = (block.height > (targetHeight - (targetHeight * 0.1))) && (block.height < (targetHeight + (targetHeight * 0.1)));
            widthCheck = (block.width > (targetWidth - (targetWidth * 0.1))) && (block.width < (targetWidth + (targetWidth * 0.1)));

            leftDrive.setPower(1);
            rightDrive.setPower(1);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}