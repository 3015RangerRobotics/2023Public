package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Preferences;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class IntakeCam {
  private static UsbCamera intakeCam;
  private static CvSink intakeCamSink;
  private static CvSource maskSource;
  private static Mat currentFrame;

  private static Scalar minValues;
  private static Scalar maxValues;

  private static final int WIDTH = 432;
  private static final int HEIGHT = 240;
  private static final double centerOffsetPx = -17.5;
  private static final double metersPerPixel = 0.00065917;

  public static void init() {
    intakeCam = CameraServer.startAutomaticCapture();
    maskSource = CameraServer.putVideo("Intake Cam Mask", WIDTH, HEIGHT);
    intakeCam.setResolution(WIDTH, HEIGHT);
    intakeCamSink = new CvSink("Intake Cam Sink");
    intakeCamSink.setSource(intakeCam);

    double minHue = Preferences.getDouble("intakeCamMinHue", 35);
    double minSaturation = Preferences.getDouble("intakeCamMinSaturation", 170);
    double minValue = Preferences.getDouble("intakeCamMinValue", 170);
    minValues = new Scalar(minHue, minSaturation, minValue);

    double maxHue = Preferences.getDouble("intakeCamMaxHue", 65);
    double maxSaturation = Preferences.getDouble("intakeCamMaxSaturation", 255);
    double maxValue = Preferences.getDouble("intakeCamMaxValue", 255);
    maxValues = new Scalar(maxHue, maxSaturation, maxValue);

    Preferences.setDouble("intakeCamMinHue", minHue);
    Preferences.setDouble("intakeCamMinSaturation", minSaturation);
    Preferences.setDouble("intakeCamMinValue", minValue);

    Preferences.setDouble("intakeCamMaxHue", maxHue);
    Preferences.setDouble("intakeCamMaxSaturation", maxSaturation);
    Preferences.setDouble("intakeCamMaxValue", maxValue);

    currentFrame = new Mat();
  }

  public static boolean isCameraConnected() {
    // Just check if the sink times out
    long ts = intakeCamSink.grabFrame(currentFrame);

    return ts != 0;
  }

  public static void doCalibrationFrame() {
    double minHue = Preferences.getDouble("intakeCamMinHue", 35);
    double minSaturation = Preferences.getDouble("intakeCamMinSaturation", 170);
    double minValue = Preferences.getDouble("intakeCamMinValue", 170);
    Scalar minValues = new Scalar(minHue, minSaturation, minValue);

    double maxHue = Preferences.getDouble("intakeCamMaxHue", 65);
    double maxSaturation = Preferences.getDouble("intakeCamMaxSaturation", 255);
    double maxValue = Preferences.getDouble("intakeCamMaxValue", 255);
    Scalar maxValues = new Scalar(maxHue, maxSaturation, maxValue);

    long ts = intakeCamSink.grabFrame(currentFrame);
    if (ts == 0) {
      return;
    }

    Mat blurredImage = new Mat();
    Mat hsvImage = new Mat();
    Mat mask = new Mat();

    // remove some noise
    Imgproc.blur(currentFrame, blurredImage, new Size(5, 5));

    // convert the frame to HSV
    Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);

    Core.inRange(hsvImage, minValues, maxValues, mask);

    Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6)));

    maskSource.putFrame(mask);
  }

  public static double getConeOffsetMeters() {
    long ts = intakeCamSink.grabFrame(currentFrame);

    if (ts == 0) {
      return 0.0;
    }

    Mat blurredImage = new Mat();
    Mat hsvImage = new Mat();
    Mat mask = new Mat();

    Mat croppedImage = new Mat(currentFrame, new Rect(0, HEIGHT / 2, WIDTH, HEIGHT / 2));

    // remove some noise
    Imgproc.blur(croppedImage, blurredImage, new Size(3, 3));

    // convert the frame to HSV
    Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);

    Core.inRange(hsvImage, minValues, maxValues, mask);

    Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2)));

    List<MatOfPoint> contours = new ArrayList<>();

    // find contours
    Imgproc.findContours(
        mask, contours, new Mat(), Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

    double largestContourArea = 0;
    MatOfPoint largestContour = null;

    for (MatOfPoint contour : contours) {
      double area = Imgproc.contourArea(contour);
      if (area > largestContourArea) {
        largestContour = contour;
        largestContourArea = area;
      }
    }

    if (largestContour == null) {
      return 0;
    }

    Moments moments = Imgproc.moments(largestContour);
    double pixelX = moments.get_m10() / moments.get_m00();
    pixelX += centerOffsetPx;

    return -(pixelX - (WIDTH / 2.0)) * metersPerPixel;
  }
}
