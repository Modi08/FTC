package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Rect;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class objectRecognition extends OpenCvPipeline {

    Net yolo8 = Dnn.readNetFromTorch("best.pt");

    @Override
    public Mat processFrame(Mat input) {
        yolo8.setInput(input);
        return yolo8.forward();
    }

    public List<Integer> getObjectHeights(Mat input) {
        List<Integer> heights = new ArrayList<>();
        MatOfFloat data = new MatOfFloat();
        yolo8.setInput(input);
        yolo8.forward((List<Mat>) data, yolo8.getUnconnectedOutLayersNames());

        List<Mat> outputs = new ArrayList<>();
        List<Integer> classIds = new ArrayList<>();
        List<Float> confidences = new ArrayList<>();
        List<Rect> boxes = new ArrayList<>();

        MatOfInt indices = new MatOfInt();
        Dnn.NMSBoxes(data, confidences, 0.5f, 0.5f, indices);

        int[] nmsIndices = indices.toArray();
        for (int nmsIndex : nmsIndices) {
            float[] detection = data.get(nmsIndex, 0);
            float confidence = detection[4];

            if (confidence > 0.5f) {
                int classId = (int) detection[5];
                int x = (int) (detection[0] * input.cols());
                int y = (int) (detection[1] * input.rows());
                int width = (int) (detection[2] * input.cols()) - x;
                int height = (int) (detection[3] * input.rows()) - y;

                classIds.add(classId);
                confidences.add(confidence);
                boxes.add(new Rect(x, y, width, height));
                heights.add(height); // Add height to the list
            }
        }
        return heights;
    }
}
