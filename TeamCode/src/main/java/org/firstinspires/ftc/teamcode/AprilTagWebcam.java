package org.firstinspires.ftc.teamcode;

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
    private AprilTagProcessor aprilTagProcessor;

    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Scanner"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);
        builder.enableLiveView(true);

        visionPortal = builder.build();

    }

    public void updateVision(){
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }


    //checks if it's an april tag then checks if it's a cluster
    public List<AprilTagClusterDetection> getDetectedClusters(){
        List<AprilTagClusterDetection> clusters = new ArrayList<>();
        for(AprilTagDetection detection: detectedTags){
            if(detection instanceof AprilTagClusterDetection) {
                clusters.add((AprilTagClusterDetection) detection);
            }
        }
        return clusters;
    }

    public AprilTagClusterDetection getClusterByName(String name) {
        for(AprilTagClusterDetection cluster: getDetectedClusters()){
            if(cluster.metadata != null && name.equals(cluster.metadata.name)){
                return cluster;
            }
        }
        return null;
    }

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

    public void displayTelemetry(AprilTagClusterDetection cluster){
        if(cluster != null){
            telemetry.addLine(String.format("\n==== Tag Cluster (%s)", cluster.metadata.name));
            telemetry.addLine(String.format("Percent tags found: %d", cluster.percentClusterFound));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", cluster.ftcPose.x, cluster.ftcPose.y, cluster.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", cluster.ftcPose.pitch, cluster.ftcPose.roll, cluster.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", cluster.ftcPose.range, cluster.ftcPose.bearing, cluster.ftcPose.elevation));

        }
    }

    public void stop(){
        if(visionPortal != null){
            visionPortal.close();
        }
    }

    public AprilTagClusterDetection getNearestBlueCluster() {
        return getNearestClusterByKeyword("BLUE");
    }

    public AprilTagClusterDetection getNearestRedCluster() {
        return getNearestClusterByKeyword("RED");
    }

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

    // TODO: create function if it is scorable
    // check if ftcPose.roll is less than 90, add absolute value around roll
}
