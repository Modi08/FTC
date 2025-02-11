package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.bosch.BHI260IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "automain")
public class autoMain extends LinearOpMode{

    public BHI260IMU imu;
    public OpenCvWebcam leftCamera;
    public OpenCvWebcam rightCamera;
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor armMotor;
    public CRServo intake;

    final double armTicksPerDegree = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    final double motorTicksPerDegree = 28 * (3591.0 / 187.0) * 100.0 / 20.0 * 1/360.0;

    //Arm position Constants
    final double armClosedRobot = 0;
    final double armCollect = 250 * armTicksPerDegree;
    final double armScoreSampleLow = 160 * armTicksPerDegree;
    final double armWinchRobot = 15 * armTicksPerDegree;

    //Motor Constants
    final double wheelCircumference = 15.5;

    //Different motor speeds
    final double intakeCollect = -1.0;
    final double intakeOff = 0.0;
    final double intakeDeposit = 0.5;

    //Wrist position constants
    final double wristFoldedIn = 0.4;
    final double wristFoldedOut = 0.065;


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
            OpenCvPipeline leftPipelines = new objectRecognition();
            leftCamera.setPipeline(leftPipelines);
            leftCamera.setMillisecondsPermissionTimeout(30);

            OpenCvPipeline rightPipelines = new objectRecognition();
            rightCamera.setPipeline(rightPipelines);
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


    private void moveBySetDistance(double distance) {
        double currentTicks = leftDrive.getCurrentPosition();
        double targetTicks = currentTicks + (motorTicksPerDegree * 360 * (distance / wheelCircumference));
        leftDrive.setTargetPosition((int) targetTicks);
        rightDrive.setTargetPosition((int) targetTicks);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((leftDrive.getCurrentPosition() == leftDrive.getTargetPosition()) && (rightDrive.getCurrentPosition() == rightDrive.getTargetPosition())) {
            ((DcMotorEx) leftDrive).setVelocity(2100);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ((DcMotorEx) rightDrive).setVelocity(2100);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void aimHeaderOfRobot(objectRecognition xPipelines, objectRecognition yPipelines) {

        Orientation target = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double xDistance = (4 * 38 * 480)/(xPipelines.getObjectHeights().get(0) * 2.02);
        double yDistance = (4 * 38 * 480)/(yPipelines.getObjectHeights().get(0) * 2.02);


    }
}