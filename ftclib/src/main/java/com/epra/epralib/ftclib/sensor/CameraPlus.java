package com.epra.epralib.ftclib.sensor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;

import java.util.List;

public class CameraPlus {
    public static final boolean USE_WEBCAM = true;
    private List<AprilTagDetection> currentDetections;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private int targetID;

    public CameraPlus (AprilTagProcessor aprilTag, VisionPortal visionPortal) {
        this.aprilTag = aprilTag;
        this.visionPortal = visionPortal;
        currentDetections = new ArrayList<AprilTagDetection>();
        updateDetections();
        startingTarget();
    }
    public void updateDetections () {
        if (aprilTag.getDetections().size() > 0) {
            currentDetections = aprilTag.getDetections();
        }
    }

    public int getID (int index) {
        updateDetections();
        return (index < currentDetections.size()) ? (currentDetections.get(index).metadata != null) ? currentDetections.get(index).id : -1 : -1;
    }

    public int getTargetID() {return targetID;}

    public void setTargetID(int t) {targetID = t;}
    public void startingTarget() {targetID = getID(0);}
    public void targetIDPlus(int t) {targetID += t;}

    public void targetIDMinus(int t) {targetID -= t;}

    public boolean atTargetID() {
        boolean r = false;
        updateDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == targetID) {r = true;}
        }
        return r;
    }

    public int getTotalDetections () {
        updateDetections();
        return currentDetections.size();
    }

    public String getName (int index) {
        updateDetections();
        return (index < currentDetections.size()) ? (currentDetections.get(index).metadata != null) ? currentDetections.get(index).metadata.name : "Index too high" : "Null";
    }

    public double getY(int index) {
        updateDetections();
        return (index < currentDetections.size()) ? (currentDetections.get(index).metadata != null) ? currentDetections.get(index).ftcPose.y : -1 : -1;
    }

    public double getX(int index) {
        updateDetections();
        return (index < currentDetections.size()) ? (currentDetections.get(index).metadata != null) ? currentDetections.get(index).ftcPose.x : -1 : -1;
    }

    public double getYaw(int index) {
        updateDetections();
        return (index < currentDetections.size()) ? (currentDetections.get(index).metadata != null) ? currentDetections.get(index).ftcPose.yaw : -1 : -1;
    }
}
