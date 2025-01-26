package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "automain")
public class autoMain extends LinearOpMode{

    private OpenCvWebcam leftCamera;
    private OpenCvWebcam rightCamera;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor armMotor;
    private CRServo intake;

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
    final double wristFoldedIn = 0.4;
    final double wristFoldedOut = 0.065;

    final double secondsPerCentimeter = 4;


    @Override
    public void runOpMode() throws InterruptedException{
        HuskyLens.Block[] blocks;
        boolean isPieceFound = false;

        //Initializing Motors
        leftDrive = hardwareMap.get(DcMotor.class, "motorLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
        armMotor = hardwareMap.get(DcMotor.class, "motorArm");

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
        Servo wrist = hardwareMap.get(Servo.class, "wrist");

        //Configuring Servos
        intake.setPower(intakeOff);
        wrist.setPosition(wristFoldedIn);

        //Initializing Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        leftCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        rightCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam2"), cameraMonitorViewId);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            leftCamera.setPipeline(new objectRecognition());
            leftCamera.setMillisecondsPermissionTimeout(30);

            rightCamera.setPipeline(new objectRecognition());
            rightCamera.setMillisecondsPermissionTimeout(30);

            leftCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    leftCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
        }
    }


    private void moveToTargetLocation(boolean isObjectSample) {

    }
}