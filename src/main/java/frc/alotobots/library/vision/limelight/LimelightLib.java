// LimelightHelpers v1.2.1 (March 1, 2023)

package frc.alotobots.library.vision.limelight;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

@SuppressWarnings("ALL")
public class LimelightLib {

  public static class LimelightTarget_Retro {

    @JsonProperty("t6c_ts")
    private double[] cameraPose_TargetSpace;

    @JsonProperty("t6r_fs")
    private double[] robotPose_FieldSpace;

    @JsonProperty("t6r_ts")
    private double[] robotPose_TargetSpace;

    @JsonProperty("t6t_cs")
    private double[] targetPose_CameraSpace;

    @JsonProperty("t6t_rs")
    private double[] targetPose_RobotSpace;

    @SuppressWarnings("unused")
    public Pose3d getCameraPose_TargetSpace() {
      return toPose3D(cameraPose_TargetSpace);
    }

    @SuppressWarnings("unused")
    public Pose3d getRobotPose_FieldSpace() {
      return toPose3D(robotPose_FieldSpace);
    }

    @SuppressWarnings("unused")
    public Pose3d getRobotPose_TargetSpace() {
      return toPose3D(robotPose_TargetSpace);
    }

    @SuppressWarnings("unused")
    public Pose3d getTargetPose_CameraSpace() {
      return toPose3D(targetPose_CameraSpace);
    }

    @SuppressWarnings("unused")
    public Pose3d getTargetPose_RobotSpace() {
      return toPose3D(targetPose_RobotSpace);
    }

    @SuppressWarnings("unused")
    public Pose2d getCameraPose_TargetSpace2D() {
      return toPose2D(cameraPose_TargetSpace);
    }

    @SuppressWarnings("unused")
    public Pose2d getRobotPose_FieldSpace2D() {
      return toPose2D(robotPose_FieldSpace);
    }

    @SuppressWarnings("unused")
    public Pose2d getRobotPose_TargetSpace2D() {
      return toPose2D(robotPose_TargetSpace);
    }

    @SuppressWarnings("unused")
    public Pose2d getTargetPose_CameraSpace2D() {
      return toPose2D(targetPose_CameraSpace);
    }

    @SuppressWarnings("unused")
    public Pose2d getTargetPose_RobotSpace2D() {
      return toPose2D(targetPose_RobotSpace);
    }

    @SuppressWarnings("unused")
    @JsonProperty("ta")
    public double ta;

    @SuppressWarnings("unused")
    @JsonProperty("tx")
    public double tx;

    @SuppressWarnings("unused")
    @JsonProperty("txp")
    public double tx_pixels;

    @SuppressWarnings("unused")
    @JsonProperty("ty")
    public double ty;

    @SuppressWarnings("unused")
    @JsonProperty("typ")
    public double ty_pixels;

    @SuppressWarnings("unused")
    @JsonProperty("ts")
    public double ts;

    @SuppressWarnings("unused")
    public LimelightTarget_Retro() {
      cameraPose_TargetSpace = new double[6];
      robotPose_FieldSpace = new double[6];
      robotPose_TargetSpace = new double[6];
      targetPose_CameraSpace = new double[6];
      targetPose_RobotSpace = new double[6];
    }
  }

  public static class LimelightTarget_Fiducial {

    @SuppressWarnings("unused")
    @JsonProperty("fID")
    public double fiducialID;

    @SuppressWarnings("unused")
    @JsonProperty("fam")
    public String fiducialFamily;

    @JsonProperty("t6c_ts")
    private double[] cameraPose_TargetSpace;

    @JsonProperty("t6r_fs")
    private double[] robotPose_FieldSpace;

    @JsonProperty("t6r_ts")
    private double[] robotPose_TargetSpace;

    @JsonProperty("t6t_cs")
    private double[] targetPose_CameraSpace;

    @JsonProperty("t6t_rs")
    private double[] targetPose_RobotSpace;

    @SuppressWarnings("unused")
    public Pose3d getCameraPose_TargetSpace() {
      return toPose3D(cameraPose_TargetSpace);
    }

    @SuppressWarnings("unused")
    public Pose3d getRobotPose_FieldSpace() {
      return toPose3D(robotPose_FieldSpace);
    }

    @SuppressWarnings("unused")
    public Pose3d getRobotPose_TargetSpace() {
      return toPose3D(robotPose_TargetSpace);
    }

    @SuppressWarnings("unused")
    public Pose3d getTargetPose_CameraSpace() {
      return toPose3D(targetPose_CameraSpace);
    }

    @SuppressWarnings("unused")
    public Pose3d getTargetPose_RobotSpace() {
      return toPose3D(targetPose_RobotSpace);
    }

    @SuppressWarnings("unused")
    public Pose2d getCameraPose_TargetSpace2D() {
      return toPose2D(cameraPose_TargetSpace);
    }

    @SuppressWarnings("unused")
    public Pose2d getRobotPose_FieldSpace2D() {
      return toPose2D(robotPose_FieldSpace);
    }

    @SuppressWarnings("unused")
    public Pose2d getRobotPose_TargetSpace2D() {
      return toPose2D(robotPose_TargetSpace);
    }

    @SuppressWarnings("unused")
    public Pose2d getTargetPose_CameraSpace2D() {
      return toPose2D(targetPose_CameraSpace);
    }

    @SuppressWarnings("unused")
    public Pose2d getTargetPose_RobotSpace2D() {
      return toPose2D(targetPose_RobotSpace);
    }

    @SuppressWarnings("unused")
    @JsonProperty("ta")
    public double ta;

    @SuppressWarnings("unused")
    @JsonProperty("tx")
    public double tx;

    @SuppressWarnings("unused")
    @JsonProperty("txp")
    public double tx_pixels;

    @SuppressWarnings("unused")
    @JsonProperty("ty")
    public double ty;

    @SuppressWarnings("unused")
    @JsonProperty("typ")
    public double ty_pixels;

    @SuppressWarnings("unused")
    @JsonProperty("ts")
    public double ts;

    @SuppressWarnings("unused")
    public LimelightTarget_Fiducial() {
      cameraPose_TargetSpace = new double[6];
      robotPose_FieldSpace = new double[6];
      robotPose_TargetSpace = new double[6];
      targetPose_CameraSpace = new double[6];
      targetPose_RobotSpace = new double[6];
    }
  }

  @SuppressWarnings("unused")
  public static class LimelightTarget_Barcode {}

  public static class LimelightTarget_Classifier {

    @SuppressWarnings("unused")
    @JsonProperty("class")
    public String className;

    @SuppressWarnings("unused")
    @JsonProperty("classID")
    public double classID;

    @SuppressWarnings("unused")
    @JsonProperty("conf")
    public double confidence;

    @SuppressWarnings("unused")
    @JsonProperty("zone")
    public double zone;

    @SuppressWarnings("unused")
    @JsonProperty("tx")
    public double tx;

    @SuppressWarnings("unused")
    @JsonProperty("txp")
    public double tx_pixels;

    @SuppressWarnings("unused")
    @JsonProperty("ty")
    public double ty;

    @SuppressWarnings("unused")
    @JsonProperty("typ")
    public double ty_pixels;

    @SuppressWarnings("unused")
    public LimelightTarget_Classifier() {}
  }

  public static class LimelightTarget_Detector {

    @SuppressWarnings("unused")
    @JsonProperty("class")
    public String className;

    @SuppressWarnings("unused")
    @JsonProperty("classID")
    public double classID;

    @SuppressWarnings("unused")
    @JsonProperty("conf")
    public double confidence;

    @SuppressWarnings("unused")
    @JsonProperty("ta")
    public double ta;

    @SuppressWarnings("unused")
    @JsonProperty("tx")
    public double tx;

    @SuppressWarnings("unused")
    @JsonProperty("txp")
    public double tx_pixels;

    @SuppressWarnings("unused")
    @JsonProperty("ty")
    public double ty;

    @SuppressWarnings("unused")
    @JsonProperty("typ")
    public double ty_pixels;

    @SuppressWarnings("unused")
    public LimelightTarget_Detector() {}
  }

  @SuppressWarnings("SpellCheckingInspection")
  public static class Results {

    @SuppressWarnings("unused")
    @JsonProperty("pID")
    public double pipelineID;

    @SuppressWarnings("unused")
    @JsonProperty("tl")
    public double latency_pipeline;

    @SuppressWarnings("unused")
    @JsonProperty("cl")
    public double latency_capture;

    @SuppressWarnings("unused")
    public double latency_jsonParse;

    @SuppressWarnings("unused")
    @JsonProperty("ts")
    public double timestamp_LIMELIGHT_publish;

    @SuppressWarnings("unused")
    @JsonProperty("ts_rio")
    public double timestamp_RIOFPGA_capture;

    @SuppressWarnings("unused")
    @JsonProperty("v")
    @JsonFormat(shape = Shape.NUMBER)
    public boolean valid;

    @JsonProperty("botpose")
    public double[] botpose;

    @JsonProperty("botpose_wpired")
    public double[] botpose_wpired;

    @JsonProperty("botpose_wpiblue")
    public double[] botpose_wpiblue;

    @SuppressWarnings("unused")
    @JsonProperty("t6c_rs")
    public double[] camerapose_robotspace;

    @SuppressWarnings("unused")
    public Pose3d getBotPose3d() {
      return toPose3D(botpose);
    }

    @SuppressWarnings("unused")
    public Pose3d getBotPose3d_wpiRed() {
      return toPose3D(botpose_wpired);
    }

    @SuppressWarnings("unused")
    public Pose3d getBotPose3d_wpiBlue() {
      return toPose3D(botpose_wpiblue);
    }

    @SuppressWarnings("unused")
    public Pose2d getBotPose2d() {
      return toPose2D(botpose);
    }

    @SuppressWarnings("unused")
    public Pose2d getBotPose2d_wpiRed() {
      return toPose2D(botpose_wpired);
    }

    @SuppressWarnings("unused")
    public Pose2d getBotPose2d_wpiBlue() {
      return toPose2D(botpose_wpiblue);
    }

    @SuppressWarnings("unused")
    @JsonProperty("Retro")
    public LimelightTarget_Retro[] targets_Retro;

    @SuppressWarnings("unused")
    @JsonProperty("Fiducial")
    public LimelightTarget_Fiducial[] targets_Fiducials;

    @SuppressWarnings("unused")
    @JsonProperty("Classifier")
    public LimelightTarget_Classifier[] targets_Classifier;

    @SuppressWarnings("unused")
    @JsonProperty("Detector")
    public LimelightTarget_Detector[] targets_Detector;

    @SuppressWarnings("unused")
    @JsonProperty("Barcode")
    public LimelightTarget_Barcode[] targets_Barcode;

    @SuppressWarnings("unused")
    public Results() {
      botpose = new double[6];
      botpose_wpired = new double[6];
      botpose_wpiblue = new double[6];
      camerapose_robotspace = new double[6];
      targets_Retro = new LimelightTarget_Retro[0];
      targets_Fiducials = new LimelightTarget_Fiducial[0];
      targets_Classifier = new LimelightTarget_Classifier[0];
      targets_Detector = new LimelightTarget_Detector[0];
      targets_Barcode = new LimelightTarget_Barcode[0];
    }
  }

  public static class LimelightResults {
    @JsonProperty("Results")
    public Results targetingResults;

    @SuppressWarnings("unused")
    public LimelightResults() {
      targetingResults = new Results();
    }
  }

  private static ObjectMapper mapper;

  /** Print JSON Parse time to the console in milliseconds */
  static final boolean profileJSON = false;

  static String sanitizeName(String name) {
    if (name == "" || name == null) {
      return "limelight";
    }
    return name;
  }

  private static Pose3d toPose3D(double[] inData) {
    if (inData.length < 6) {
      System.err.println("Bad LL 3D Pose Data!");
      return new Pose3d();
    }
    return new Pose3d(
        new Translation3d(inData[0], inData[1], inData[2]),
        new Rotation3d(
            Units.degreesToRadians(inData[3]),
            Units.degreesToRadians(inData[4]),
            Units.degreesToRadians(inData[5])));
  }

  private static Pose2d toPose2D(double[] inData) {
    if (inData.length < 6) {
      System.err.println("Bad LL 2D Pose Data!");
      return new Pose2d();
    }
    Translation2d tran2d = new Translation2d(inData[0], inData[1]);
    Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
    return new Pose2d(tran2d, r2d);
  }

  public static NetworkTable getLimelightNTTable(String tableName) {
    return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
  }

  public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
    return getLimelightNTTable(tableName).getEntry(entryName);
  }

  public static double getLimelightNTDouble(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
  }

  public static void setLimelightNTDouble(String tableName, String entryName, double val) {
    getLimelightNTTableEntry(tableName, entryName).setDouble(val);
  }

  public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
    getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
  }

  public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
  }

  public static String getLimelightNTString(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getString("");
  }

  public static URL getLimelightURLString(String tableName, String request) {
    String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
    URL url;
    try {
      url = new URL(urlString);
      return url;
    } catch (MalformedURLException e) {
      System.err.println("bad LL URL");
    }
    return null;
  }

  /////
  /////

  public static double getTX(String limelightName) {
    return getLimelightNTDouble(limelightName, "tx");
  }

  @SuppressWarnings("unused")
  public static double getTY(String limelightName) {
    return getLimelightNTDouble(limelightName, "ty");
  }

  @SuppressWarnings("unused")
  public static double getTA(String limelightName) {
    return getLimelightNTDouble(limelightName, "ta");
  }

  @SuppressWarnings("unused")
  public static double getLatency_Pipeline(String limelightName) {
    return getLimelightNTDouble(limelightName, "tl");
  }

  @SuppressWarnings("unused")
  public static double getLatency_Capture(String limelightName) {
    return getLimelightNTDouble(limelightName, "cl");
  }

  @SuppressWarnings("unused")
  public static double getCurrentPipelineIndex(String limelightName) {
    return getLimelightNTDouble(limelightName, "getpipe");
  }

  public static String getJSONDump(String limelightName) {
    return getLimelightNTString(limelightName, "json");
  }

  /**
   * Switch to getBotPose
   *
   * @param limelightName
   * @return
   */
  @Deprecated
  public static double[] getBotpose(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose");
  }

  /**
   * Switch to getBotPose_wpiRed
   *
   * @param limelightName
   * @return
   */
  @Deprecated
  public static double[] getBotpose_wpiRed(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
  }

  /**
   * Switch to getBotPose_wpiBlue
   *
   * @param limelightName
   * @return
   */
  @Deprecated
  public static double[] getBotpose_wpiBlue(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
  }

  public static double[] getBotPose(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose");
  }

  public static double[] getBotPose_wpiRed(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
  }

  public static double[] getBotPose_wpiBlue(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
  }

  @SuppressWarnings("unused")
  public static double[] getBotPose_TargetSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
  }

  @SuppressWarnings("unused")
  public static double[] getCameraPose_TargetSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
  }

  @SuppressWarnings("unused")
  public static double[] getTargetPose_CameraSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
  }

  @SuppressWarnings("unused")
  public static double[] getTargetPose_RobotSpace(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
  }

  @SuppressWarnings("unused")
  public static double[] getTargetColor(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "tc");
  }

  @SuppressWarnings("unused")
  public static double getFiducialID(String limelightName) {
    return getLimelightNTDouble(limelightName, "tid");
  }

  @SuppressWarnings("unused")
  public static double getNeuralClassID(String limelightName) {
    return getLimelightNTDouble(limelightName, "tclass");
  }

  /////
  /////

  @SuppressWarnings("unused")
  public static Pose3d getBotPose3d(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose");
    return toPose3D(poseArray);
  }

  @SuppressWarnings("unused")
  public static Pose3d getBotPose3d_wpiRed(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    return toPose3D(poseArray);
  }

  @SuppressWarnings("unused")
  public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    return toPose3D(poseArray);
  }

  @SuppressWarnings("unused")
  public static Pose3d getBotPose3d_TargetSpace(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
    return toPose3D(poseArray);
  }

  @SuppressWarnings("unused")
  public static Pose3d getCameraPose3d_TargetSpace(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
    return toPose3D(poseArray);
  }

  @SuppressWarnings("unused")
  public static Pose3d getTargetPose3d_CameraSpace(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
    return toPose3D(poseArray);
  }

  @SuppressWarnings("unused")
  public static Pose3d getTargetPose3d_RobotSpace(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    return toPose3D(poseArray);
  }

  @SuppressWarnings("unused")
  public static Pose3d getCameraPose3d_RobotSpace(String limelightName) {
    double[] poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_robotspace");
    return toPose3D(poseArray);
  }

  /**
   * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
   *
   * @param limelightName
   * @return
   */
  @SuppressWarnings("unused")
  public static Pose2d getBotPose2d_wpiBlue(String limelightName) {

    double[] result = getBotPose_wpiBlue(limelightName);
    return toPose2D(result);
  }

  /**
   * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
   *
   * @param limelightName
   * @return
   */
  @SuppressWarnings("unused")
  public static Pose2d getBotPose2d_wpiRed(String limelightName) {

    double[] result = getBotPose_wpiRed(limelightName);
    return toPose2D(result);
  }

  /**
   * Gets the Pose2d for easy use with Odometry vision pose estimator (addVisionMeasurement)
   *
   * @param limelightName
   * @return
   */
  @SuppressWarnings("unused")
  public static Pose2d getBotPose2d(String limelightName) {

    double[] result = getBotPose(limelightName);
    return toPose2D(result);
  }

  public static boolean getTV(String limelightName) {
    return 1.0 == getLimelightNTDouble(limelightName, "tv");
  }

  /////
  /////

  @SuppressWarnings("unused")
  public static void setPipelineIndex(String limelightName, int pipelineIndex) {
    setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
  }

  /** The LEDs will be controlled by Limelight pipeline settings, and not by robot code. */
  @SuppressWarnings("unused")
  public static void setLEDMode_PipelineControl(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 0);
  }

  @SuppressWarnings("unused")
  public static void setLEDMode_ForceOff(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 1);
  }

  @SuppressWarnings("unused")
  public static void setLEDMode_ForceBlink(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 2);
  }

  @SuppressWarnings("unused")
  public static void setLEDMode_ForceOn(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 3);
  }

  @SuppressWarnings("unused")
  public static void setStreamMode_Standard(String limelightName) {
    setLimelightNTDouble(limelightName, "stream", 0);
  }

  @SuppressWarnings("unused")
  public static void setStreamMode_PiPMain(String limelightName) {
    setLimelightNTDouble(limelightName, "stream", 1);
  }

  @SuppressWarnings("unused")
  public static void setStreamMode_PiPSecondary(String limelightName) {
    setLimelightNTDouble(limelightName, "stream", 2);
  }

  @SuppressWarnings("unused")
  public static void setCameraMode_Processor(String limelightName) {
    setLimelightNTDouble(limelightName, "camMode", 0);
  }

  @SuppressWarnings("unused")
  public static void setCameraMode_Driver(String limelightName) {
    setLimelightNTDouble(limelightName, "camMode", 1);
  }

  /**
   * Sets the crop window. The crop window in the UI must be completely open for dynamic cropping to
   * work.
   */
  @SuppressWarnings("unused")
  public static void setCropWindow(
      String limelightName, double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
    double[] entries = new double[4];
    entries[0] = cropXMin;
    entries[1] = cropXMax;
    entries[2] = cropYMin;
    entries[3] = cropYMax;
    setLimelightNTDoubleArray(limelightName, "crop", entries);
  }

  @SuppressWarnings("unused")
  public static void setCameraPose_RobotSpace(
      String limelightName,
      double forward,
      double side,
      double up,
      double roll,
      double pitch,
      double yaw) {
    double[] entries = new double[6];
    entries[0] = forward;
    entries[1] = side;
    entries[2] = up;
    entries[3] = roll;
    entries[4] = pitch;
    entries[5] = yaw;
    setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
  }

  /////
  /////

  @SuppressWarnings("unused")
  public static void setPythonScriptData(String limelightName, double[] outgoingPythonData) {
    setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData);
  }

  @SuppressWarnings("unused")
  public static double[] getPythonScriptData(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "llpython");
  }

  /////
  /////

  /** Asynchronously take snapshot. */
  @SuppressWarnings("unused")
  public static CompletableFuture<Boolean> takeSnapshot(String tableName, String snapshotName) {
    return CompletableFuture.supplyAsync(() -> SYNCH_TAKESNAPSHOT(tableName, snapshotName));
  }

  private static boolean SYNCH_TAKESNAPSHOT(String tableName, String snapshotName) {
    URL url = getLimelightURLString(tableName, "capturesnapshot");
    try {
      HttpURLConnection connection = (HttpURLConnection) url.openConnection();
      connection.setRequestMethod("GET");
      if (snapshotName != null && snapshotName != "") {
        connection.setRequestProperty("snapname", snapshotName);
      }

      int responseCode = connection.getResponseCode();
      if (responseCode == 200) {
        return true;
      } else {
        System.err.println("Bad LL Request");
      }
    } catch (IOException e) {
      System.err.println(e.getMessage());
    }
    return false;
  }

  /** Parses Limelight's JSON results dump into a LimelightResults Object */
  @SuppressWarnings("unused")
  public static LimelightResults getLatestResults(String limelightName) {

    long start = System.nanoTime();
    LimelightResults results = new LimelightResults();
    if (mapper == null) {
      mapper =
          new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    }

    try {
      results = mapper.readValue(getJSONDump(limelightName), LimelightResults.class);
    } catch (JsonProcessingException e) {
      System.err.println("lljson error: " + e.getMessage());
    }

    long end = System.nanoTime();
    double millis = (end - start) * .000001;
    results.targetingResults.latency_jsonParse = millis;
    if (profileJSON) {
      System.out.printf("lljson: %.2f\r\n", millis);
    }

    return results;
  }
}
