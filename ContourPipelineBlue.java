package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ContourPipelineBlue extends OpenCvPipeline {


    int leftbound = 500;
    double largest = 0;
    double largestextent = 0;
    Scalar lowerHSV = new Scalar(100.0, 45.5, 56.5);//HSV for now
    Scalar upperHSV = new Scalar(160.0, 255.0, 255.0);//HSV for now
    int tickMark;
    @Override
    public Mat processFrame(Mat input) {

        //entire thing has to be in a try catch other wise if a contour is not found it will throw and error and stop running
        Mat end = new Mat();
        try {
            end = input;

            Mat src = input;

            //Freight Frenzy CV goal- to detect a custom team shipping element and determine which tape mark it is at- the left, middle, or right


            //this is YCRCB - Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
            //this is YCRCB - Scalar scalarUpperYCrCb = new Scalar(255.0, 100.0, 170.0);

            //those are hard to figure out ^^^^^^^^^^^ but they are in the Y CR CB color space

            //forming a color range
            //pick a range of HSV colors that aren't too specific nor general- they should contain the color of object you are trying to find in the range




            //Those are the boundaries of the accepted colors in HSV

            //open cv- Hue goes to 179, the other two go to 255
            //google - Hue goes to 360, the other two are percentages out of 100%

            //I found that translating them doesn't really work so I just made an easier range finder: https://tejusk2.github.io/FTCVISION/


            //gets the image ready for contour detection


            //Converts space to HSV
            Imgproc.cvtColor(src, src, Imgproc.COLOR_RGB2HSV);
            //filters out all colors not in this range
            Core.inRange(src, lowerHSV, upperHSV, src);
            // Remove Noise
            //choose one or the other or they cancel things out, I AM USING CLOSE and it is being used in the range finder

            Imgproc.morphologyEx(src, src, Imgproc.MORPH_OPEN, new Mat());
            // GaussianBlur
            Imgproc.blur(src, src, new Size(10, 10));


            //Finding Contours

            ArrayList<MatOfPoint> contours = new ArrayList<>();

            Mat hierarchey = new Mat();

            //finds contour
            Imgproc.findContours(src, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);







            largest = 0;
            largestextent = 0;

            Rect spot = null;
            for(MatOfPoint contour : contours) {
                MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());


                Rect rect = Imgproc.boundingRect(areaPoints);

                double rectArea = rect.area();
                double cntArea = Imgproc.contourArea(contour);

                double extent = cntArea / rectArea;
                //extent- ratio of contour(feature) area to bounding rectangle area



                if(extent > 0.3 && rect.width > 200 && rect.height >150 ) {
                    Imgproc.rectangle(end, rect, new Scalar(255,0,0));
                    if(rect.area() > largest && extent > largestextent){
                        largest = rect.area();
                        largestextent = extent;

                        spot = rect;
                    }



                    Imgproc.putText(end, String.valueOf(rect.width), new Point(rect.x+50, rect.y+50), 1, 3, new Scalar(255,0,0));
                    Imgproc.putText(end, String.valueOf(rect.height), new Point(rect.x-100, rect.y-50), 1, 3, new Scalar(255,0,0));

                }
                if(spot == null){
                    tickMark = 1;
                }


            }
            try {
                if (spot.x < leftbound) {
                    tickMark = 2;
                } else if (spot.x > leftbound) {
                    tickMark = 3;
                }
            }catch (NullPointerException error){

            }


        }catch (IndexOutOfBoundsException error){

        }
        return end;
    }



}
