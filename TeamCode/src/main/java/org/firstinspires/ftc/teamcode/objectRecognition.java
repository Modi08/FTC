package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.openftc.easyopencv.OpenCvPipeline;

public class objectRecognition extends OpenCvPipeline {

    Net yolo8 = Dnn.readNetFromTorch("yolov8n.pt");

    @Override
    public Mat processFrame(Mat input) {
        yolo8.setInput(input);

        return yolo8.forward();
    }
}
