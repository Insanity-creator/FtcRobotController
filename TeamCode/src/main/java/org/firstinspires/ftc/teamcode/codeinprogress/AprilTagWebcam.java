package org.firstinspires.ftc.teamcode.codeinprogress;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {

    // The processor that analyzes camera frames to detect AprilTags
    private AprilTagProcessor aprilTagProcessor;

    // The vision portal that manages the webcam stream and hardware
    private VisionPortal visionPortal;

    // A list storing all AprilTag detections found in the current frame
    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    // Telemetry object used to print debug info onto the Driver Station screen
    private Telemetry telemetry;

    /**
     * Initializes the webcam, configures the AprilTag processor, and starts the camera stream.
     */
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Build the AprilTag processor with visual aids and measurement units
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        // Build and start the VisionPortal using the webcam named "Scanner"
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Scanner"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);
        builder.enableLiveView(true);

        visionPortal = builder.build();
    }

    /**
     * Refreshes the list of detected tags by grabbing the latest frame from the camera.
     */
    public void updateVision(){
        detectedTags = aprilTagProcessor.getDetections();
    }

    /**
     * Returns the full list of all detected AprilTag detections.
     */
    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    /**
     * Filters all detected tags and returns only the ones that are AprilTag Clusters.
     */
    public List<AprilTagClusterDetection> getDetectedClusters(){
        List<AprilTagClusterDetection> clusters = new ArrayList<>();
        for(AprilTagDetection detection: detectedTags){
            if(detection instanceof AprilTagClusterDetection) {
                clusters.add((AprilTagClusterDetection) detection);
            }
        }
        return clusters;
    }

    /**
     * Searches through detected clusters and returns the one matching a specific name (e.g., "BLUE SCORING").
     */
    public AprilTagClusterDetection getClusterByName(String name) {
        for(AprilTagClusterDetection cluster: getDetectedClusters()){
            if(cluster.metadata != null && name.equals(cluster.metadata.name)){
                return cluster;
            }
        }
        return null;
    }

    /**
     * Finds and returns the nearest cluster of any color based on distance (ftcPose.range).
     */
    public AprilTagClusterDetection getNearestCluster(){
        AprilTagClusterDetection nearest = null;
        for(AprilTagClusterDetection cluster: getDetectedClusters()){
            if(cluster.ftcPose == null){
                continue;
            }
            // Check nearest == null FIRST to avoid NullPointerException
            if(nearest == null || cluster.ftcPose.range < nearest.ftcPose.range){
                nearest = cluster;
            }
        }
        return nearest;
    }

    /**
     * Prints detailed telemetry info about a specific cluster onto the Driver Station.
     */
    public void displayTelemetry(AprilTagClusterDetection cluster){
        if(cluster != null){
            telemetry.addLine(String.format("\n==== Tag Cluster (%s)", cluster.metadata.name));
            telemetry.addLine(String.format("Percent tags found: %d", cluster.percentClusterFound));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", cluster.ftcPose.x, cluster.ftcPose.y, cluster.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", cluster.ftcPose.pitch, cluster.ftcPose.roll, cluster.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", cluster.ftcPose.range, cluster.ftcPose.bearing, cluster.ftcPose.elevation));
        }
    }

    /**
     * Safely closes the webcam stream when the OpMode stops.
     */
    public void stop(){
        if(visionPortal != null){
            visionPortal.close();
        }
    }

    /**
     * Finds the nearest Blue alliance cluster ("BLUE SCORING" or "BLUE AUDIENCE").
     */
    public AprilTagClusterDetection getNearestBlueCluster() {
        return getNearestClusterByKeyword("BLUE");
    }

    /**
     * Finds the nearest Red alliance cluster ("RED SCORING" or "RED AUDIENCE").
     */
    public AprilTagClusterDetection getNearestRedCluster() {
        return getNearestClusterByKeyword("RED");
    }

    /**
     * Helper method that finds the nearest cluster whose name contains a specific keyword ("BLUE" or "RED").
     */
    private AprilTagClusterDetection getNearestClusterByKeyword(String keyword) {
        AprilTagClusterDetection nearest = null;
        for (AprilTagClusterDetection cluster : getDetectedClusters()) {
            if (cluster.metadata != null && cluster.metadata.name.contains(keyword)) {
                if (cluster.ftcPose == null) {
                    continue;
                }
                if (nearest == null || cluster.ftcPose.range < nearest.ftcPose.range) {
                    nearest = cluster;
                }
            }
        }
        return nearest;
    }

    // Camera is mounted facing upward at an angle of 20 to 30 degrees
    private static final double CAMERA_PITCH_OFFSET = 25.0;

    /**
     * Checks if a cluster is scorable (upright and not tipped over).
     * Accounts for the upward-facing camera angle and checks the true field-relative pitch.
     */
    public boolean isClusterScorable(AprilTagClusterDetection cluster) {
        if (cluster == null || cluster.ftcPose == null) {
            return false;
        }

        // Add the camera's upward offset to get the true field-relative pitch
        double fieldRelativePitch = cluster.ftcPose.pitch + CAMERA_PITCH_OFFSET;

        // When upright, the tag rests pointing up/back (15° to 30°).
        // When tipped, the pitch drops into a downward angle.
        // It is scorable only when the field-relative pitch is above 0 degrees (pointing upward/level, not downward).
        return fieldRelativePitch > 0.0;
    }

    /**
     * Finds the nearest Blue cluster that is upright and scorable (not tipped).
     */
    public AprilTagClusterDetection getNearestScorableBlueCluster() {
        return getNearestScorableClusterByKeyword("BLUE");
    }

    /**
     * Finds the nearest Red cluster that is upright and scorable (not tipped).
     */
    public AprilTagClusterDetection getNearestScorableRedCluster() {
        return getNearestScorableClusterByKeyword("RED");
    }

    /**
     * Helper method that finds the nearest scorable (not tipped) cluster matching a keyword ("BLUE" or "RED").
     */
    private AprilTagClusterDetection getNearestScorableClusterByKeyword(String keyword) {
        AprilTagClusterDetection nearest = null;
        for (AprilTagClusterDetection cluster : getDetectedClusters()) {
            if (cluster.metadata != null && cluster.metadata.name.contains(keyword)) {
                if (isClusterScorable(cluster)) {
                    if (nearest == null || cluster.ftcPose.range < nearest.ftcPose.range) {
                        nearest = cluster;
                    }
                }
            }
        }
        return nearest;
    }
}
