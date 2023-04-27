package frc.lib.vision.jetson;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.Vector3;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class JetsonClient {
  private final NetworkTable jetsonTable;
  private final NetworkTableListener timeSyncListener;
  //  private final NetworkTableListener aprilTagListener;
  private final NetworkTableListener objectsListener;
  private final NetworkTableListener barometerListener;

  //  private final AtomicReference<ArrayList<AprilTagDetection>> currentAprilTags =
  //      new AtomicReference<>();
  private final AtomicReference<ArrayList<ObjectDetection>> currentObjects =
      new AtomicReference<>();
  private final AtomicReference<Double> currentBaroPressure = new AtomicReference<>();
  private final AtomicReference<Double> timeOffset = new AtomicReference<>();

  public JetsonClient() {
    this.jetsonTable = NetworkTableInstance.getDefault().getTable("jetson");
    //    this.currentAprilTags.set(new ArrayList<>());
    this.currentObjects.set(new ArrayList<>());
    this.currentBaroPressure.set(101325.0); // Standard ATM pressure in pascals
    this.timeOffset.set(0.0);

    this.timeSyncListener =
        NetworkTableListener.createListener(
            this.jetsonTable.getEntry("timeSync"),
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (event) -> {
              if (event.valueData != null) {
                double jetsonTime = event.valueData.value.getDouble();
                if (jetsonTime != 0) {
                  this.timeOffset.set(Timer.getFPGATimestamp() - jetsonTime);
                }
              }
            });
    //    this.aprilTagListener =
    //        NetworkTableListener.createListener(
    //            this.jetsonTable.getEntry("aprilTags"),
    //            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
    //            (event) -> {
    //              if (event.valueData != null) {
    //                // Make sure we are time synced before processing apriltags
    //                if (this.timeOffset.get() != 0) {
    //                  String[] detections = event.valueData.value.getStringArray();
    //                  this.handleAprilTagData(detections);
    //                }
    //              }
    //            });
    this.objectsListener =
        NetworkTableListener.createListener(
            this.jetsonTable.getEntry("objects"),
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (event) -> {
              if (event.valueData != null) {
                // Make sure we are time synced before processing objects
                if (this.timeOffset.get() != 0) {
                  String[] detections = event.valueData.value.getStringArray();
                  this.handleObjectsData(detections);
                }
              }
            });
    this.barometerListener =
        NetworkTableListener.createListener(
            this.jetsonTable.getEntry("barometer"),
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (event) -> {
              if (event.valueData != null) {
                this.currentBaroPressure.set(event.valueData.value.getDouble());
              }
            });

    this.stopRecording();
  }

  //  private void handleAprilTagData(String[] detectionsJson) {
  //    ArrayList<AprilTagDetection> newDetections = new ArrayList<>();
  //    for (String d : detectionsJson) {
  //      try {
  //        JSONObject det = (JSONObject) new JSONParser().parse(d);
  //
  //        int id = ((Number) det.get("id")).intValue();
  //        double tx = ((Number) det.get("tx")).doubleValue();
  //        double ty = ((Number) det.get("ty")).doubleValue();
  //        double tz = ((Number) det.get("tz")).doubleValue();
  //        double rx = ((Number) det.get("rx")).doubleValue();
  //        double ry = ((Number) det.get("ry")).doubleValue();
  //        double rz = ((Number) det.get("rz")).doubleValue();
  //        double timestamp = ((Number) det.get("timestamp")).doubleValue();
  //
  //        double robotTime = timestamp + this.timeOffset.get();
  //
  //        newDetections.add(
  //            new AprilTagDetection(
  //                new AprilTag(id, new Pose3d(tx, ty, tz, new Rotation3d(rx, ry, rz))),
  // robotTime));
  //      } catch (Exception e) {
  //        // do nothing
  //      }
  //    }
  //    this.currentAprilTags.set(newDetections);
  //  }

  private void handleObjectsData(String[] detectionsJson) {
    ArrayList<ObjectDetection> newDetections = new ArrayList<>();
    for (String d : detectionsJson) {
      try {
        JSONObject det = (JSONObject) new JSONParser().parse(d);

        String className = (String) det.get("class_name");
        int objectID = ((Number) det.get("object_id")).intValue();
        double confidence = ((Number) det.get("confidence")).doubleValue();
        ObjectDetection.TrackingState trackingState = ObjectDetection.TrackingState.OFF;
        String trackingStateStr = (String) det.get("tracking_state");
        if (trackingStateStr.equals("OK")) {
          trackingState = ObjectDetection.TrackingState.OK;
        } else if (trackingStateStr.equals("SEARCHING")) {
          trackingState = ObjectDetection.TrackingState.SEARCHING;
        } else if (trackingStateStr.equals("TERMINATE")) {
          trackingState = ObjectDetection.TrackingState.TERMINATE;
        }
        boolean isMoving = (boolean) det.get("is_moving");

        JSONObject posJson = (JSONObject) det.get("position");
        JSONObject velJson = (JSONObject) det.get("velocity");
        JSONObject dimJson = (JSONObject) det.get("dimensions");

        double px = ((Number) posJson.get("x")).doubleValue();
        double py = ((Number) posJson.get("y")).doubleValue();
        double pz = ((Number) posJson.get("z")).doubleValue();
        double vx = ((Number) velJson.get("x")).doubleValue();
        double vy = ((Number) velJson.get("y")).doubleValue();
        double vz = ((Number) velJson.get("z")).doubleValue();
        double dx = ((Number) dimJson.get("x")).doubleValue();
        double dy = ((Number) dimJson.get("y")).doubleValue();
        double dz = ((Number) dimJson.get("z")).doubleValue();

        double timestamp = ((Number) det.get("timestamp")).doubleValue();
        double robotTime = timestamp + this.timeOffset.get();

        newDetections.add(
            new ObjectDetection(
                className,
                objectID,
                confidence,
                trackingState,
                isMoving,
                new Translation3d(px, py, pz),
                new Vector3(vx, vy, vz),
                new Vector3(dx, dy, dz),
                robotTime));
      } catch (Exception e) {
        // do nothing
      }
    }
    this.currentObjects.set(newDetections);
  }

  public void startRecording() {
    this.jetsonTable.getEntry("recording").setBoolean(true);
  }

  public void stopRecording() {
    this.jetsonTable.getEntry("recording").setBoolean(false);
  }

  //  public ArrayList<AprilTagDetection> getCurrentAprilTagDetections() {
  //    return this.currentAprilTags.get();
  //  }

  public ArrayList<ObjectDetection> getCurrentObjectDetections() {
    return this.currentObjects.get();
  }

  public double getCurrentAtmPressurePa() {
    return this.currentBaroPressure.get();
  }

  public void close() {
    this.timeSyncListener.close();
    //    this.aprilTagListener.close();
    this.objectsListener.close();
    this.barometerListener.close();
  }
}
