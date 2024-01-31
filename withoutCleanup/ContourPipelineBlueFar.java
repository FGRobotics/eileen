package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ContourPipelineBlueFar extends OpenCvPipeline {


    Mat src = new Mat();
    Mat end = new Mat();
    int tickMark;
    int numCircles = 0;
    Scalar lowerHSV = new Scalar(140.0, 0.0, 180.0);//HSV for now
    Scalar upperHSV = new Scalar(160.0, 255.0, 255.0);//HSV for now
    Scalar scalarUpperBGR = new Scalar(0, 0,0);

    @Override
    public Mat processFrame(Mat input) {

        //entire thing has to be in a try catch other wise if a contour is not found it will throw and error and stop running
        Imgproc.cvtColor(input,end, Imgproc.COLOR_RGB2BGR);
        Imgproc.cvtColor(input, src, Imgproc.COLOR_RGB2GRAY);
        try {








            //Grayscale



            Imgproc.blur(src, src,new Size(5,5));

            //Finding Circles

            Mat circles = new Mat();
            Imgproc.HoughCircles(src, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                    (double)src.rows()/6, // change this value to detect circles with different distances to each other
                    70.0, 30.0, 60, 140); // change the last two parameters
            // (min_radius & max_radius) to detect larger circles

            //Candidate Selection
            boolean inCam = false;
            numCircles = circles.cols();
            Imgproc.line(end, new Point(100, 0), new Point(100, src.cols()), scalarUpperBGR, 6);
            Imgproc.line(end, new Point(300, 0), new Point(300, src.cols()), scalarUpperBGR, 6);

            Imgproc.line(end, new Point(0, 300), new Point(src.cols(), 300), scalarUpperBGR, 18);


            Imgproc.line(end, new Point(700, 0), new Point(700, src.cols()), scalarUpperBGR, 6);
            Imgproc.line(end, new Point(900, 0), new Point(900, src.cols()), scalarUpperBGR, 6);


            for (int x = circles.cols()-1; x > -1; x--) {
                double[] c = circles.get(0, x);
                Point center = new Point(Math.round(c[0]), Math.round(c[1]));
                int radius = (int) Math.round(c[2]);
                double k = Math.sin(Math.PI/4);
                int sinedradius = (int)Math.round(k*radius);

                Point topLeft = new Point(center.x-sinedradius, center.y-sinedradius);
                Point bottomRight = new Point(center.x+sinedradius, center.y+sinedradius);
                int accumulator = 0;
                //------------------------------------------------------------------------------ACCUMULATOR LOGIC--------------------------------------
                int area = ((int)bottomRight.y - (int)topLeft.y)*((int)bottomRight.x - (int)topLeft.x);
                if((center.y > 300) && ((center.x >100 && center.x < 300) || (center.x > 700 && center.x < 900))) {
                    for (int i = (int) topLeft.x; i < (int) bottomRight.x; i++) {
                        for (int j = (int) topLeft.y; j < (int) bottomRight.y; j++) {

                            if (end.get(j, i)[0] > 160) {
                                accumulator++;
                            }
                        }
                    }
                }
                double avg = (double)accumulator / area;
                if(center.y > 300) {
                    Imgproc.circle(end, center, radius, new Scalar(255, 0, 255), 3, 8, 0);
                    Imgproc.circle(end, bottomRight, 1, new Scalar(0, 100, 100), 3, 8, 0);
                    Imgproc.putText(end, String.valueOf(avg), center, 3, 3, new Scalar(255,0,255));
                }



                if((avg > 0.15 && center.y > 300) && ((center.x >100 && center.x < 300) || (center.x > 700 && center.x < 900))) {
                    inCam = true;
                    if(center.x < 600) {
                        tickMark = 2;
                    }else{
                        tickMark = 3;
                    }
                    Imgproc.circle(end, center, radius, new Scalar(0,0,0), 12, 8, 0 );
                    break;

                }


            }
            if(!inCam){
                tickMark = 1;
            }
        }catch (NullPointerException error){

        }
        return end;
    }
    public int getTick(){
        return tickMark;
    }


}

