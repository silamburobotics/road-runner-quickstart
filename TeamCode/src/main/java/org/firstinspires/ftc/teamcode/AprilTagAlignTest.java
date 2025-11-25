package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import android.util.Size;

@Config
@TeleOp(name = "AprilTag Align Test", group = "Testing")
public class AprilTagAlignTest extends LinearOpMode {
    
    // Mecanum drive motors
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    
    // AprilTag detection system
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    
    // AprilTag settings
    public static int TARGET_TAG_ID_1 = 20;
    public static int TARGET_TAG_ID_2 = 24;
    
    // Alignment settings - tunable via FTC Dashboard
    public static double ALIGNMENT_KP = 0.015;  // Proportional gain
    public static double ALIGNMENT_TOLERANCE = 1.0;  // Degrees
    public static double MAX_TURN_POWER = 0.5;  // Cap turn power
    
    // Alignment state
    private boolean alignmentActive = false;
    private int targetTagId = 0;
    
    @Override
    public void runOpMode() {
        initializeDrive();
        initializeAprilTag();
        
        telemetry.addData("=== APRILTAG ALIGN TEST ===", "");
        telemetry.addData("Status", "Ready");
        telemetry.addData("", "");
        telemetry.addData("CONTROLS:", "");
        telemetry.addData("A", "Start Alignment (Tag 20 or 24)");
        telemetry.addData("B", "Stop Alignment");
        telemetry.addData("Left Stick", "Manual Drive");
        telemetry.addData("Right Stick X", "Manual Turn");
        telemetry.addData("", "");
        telemetry.addData("TUNABLE (via Dashboard):", "");
        telemetry.addData("ALIGNMENT_KP", ALIGNMENT_KP);
        telemetry.addData("ALIGNMENT_TOLERANCE", ALIGNMENT_TOLERANCE);
        telemetry.addData("MAX_TURN_POWER", MAX_TURN_POWER);
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            handleControls();
            handleAlignment();
            displayTelemetry();
            sleep(20);
        }
        
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    private void initializeDrive() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        
        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        
        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Set motor modes
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    private void initializeAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setLensIntrinsics(902.577, 902.577, 612.676, 364.762)  // OV9281 Arducam 1280x720
                .build();

        try {
            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(1280, 720));
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
            builder.enableLiveView(true);
            builder.setAutoStopLiveView(false);
            builder.addProcessor(aprilTag);
            
            visionPortal = builder.build();
            
            while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Vision Portal", "Initializing...");
                telemetry.update();
                sleep(20);
            }
            
        } catch (Exception e) {
            telemetry.addData("❌ AprilTag Error", e.getMessage());
            visionPortal = null;
            aprilTag = null;
        }
    }
    
    private void handleControls() {
        // A button - start alignment
        if (gamepad1.a && !alignmentActive) {
            startAlignment();
        }
        
        // B button - stop alignment
        if (gamepad1.b && alignmentActive) {
            stopAlignment();
        }
        
        // Manual drive when not aligning
        if (!alignmentActive) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            
            double leftFrontPower = drive + strafe + turn;
            double rightFrontPower = drive - strafe - turn;
            double leftBackPower = drive - strafe + turn;
            double rightBackPower = drive + strafe - turn;
            
            double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                                       Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
            
            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightBackPower /= maxPower;
            }
            
            setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }
    }
    
    private void startAlignment() {
        if (visionPortal == null || aprilTag == null) {
            return;
        }
        
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection targetTag = null;
        
        // Find target tag (20 or 24)
        for (AprilTagDetection detection : detections) {
            if ((detection.id == TARGET_TAG_ID_1 || detection.id == TARGET_TAG_ID_2) 
                && detection.ftcPose != null) {
                targetTag = detection;
                break;
            }
        }
        
        if (targetTag != null) {
            targetTagId = targetTag.id;
            alignmentActive = true;
        }
    }
    
    private void stopAlignment() {
        alignmentActive = false;
        setDrivePower(0, 0, 0, 0);
    }
    
    private void handleAlignment() {
        if (!alignmentActive) {
            return;
        }
        
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection currentTag = null;
        
        // Find current tag
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId && detection.ftcPose != null) {
                currentTag = detection;
                break;
            }
        }
        
        if (currentTag == null) {
            // Lost tag - stop
            stopAlignment();
            return;
        }
        
        double bearingDegrees = Math.toDegrees(currentTag.ftcPose.bearing);
        
        if (Math.abs(bearingDegrees) <= ALIGNMENT_TOLERANCE) {
            // Aligned - stop
            stopAlignment();
            return;
        }
        
        // Proportional control
        double turnPower = bearingDegrees * ALIGNMENT_KP;
        
        // Cap power
        if (Math.abs(turnPower) > MAX_TURN_POWER) {
            turnPower = Math.signum(turnPower) * MAX_TURN_POWER;
        }
        
        setDrivePower(turnPower, -turnPower, turnPower, -turnPower);
    }
    
    private void setDrivePower(double lf, double rf, double lb, double rb) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }
    
    private void displayTelemetry() {
        telemetry.addData("=== APRILTAG ALIGN TEST ===", "");
        telemetry.addData("Alignment Active", alignmentActive ? "YES" : "NO");
        
        if (aprilTag != null) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("Tags Detected", detections.size());
            telemetry.addData("", "");
            
            for (AprilTagDetection detection : detections) {
                if (detection.ftcPose != null) {
                    telemetry.addData("Tag ID", detection.id);
                    telemetry.addData("  Range", "%.2f inches", detection.ftcPose.range);
                    telemetry.addData("  Bearing", "%.2f°", Math.toDegrees(detection.ftcPose.bearing));
                    telemetry.addData("  Yaw", "%.2f°", Math.toDegrees(detection.ftcPose.yaw));
                    
                    if (alignmentActive && detection.id == targetTagId) {
                        double bearingDegrees = Math.toDegrees(detection.ftcPose.bearing);
                        double turnPower = bearingDegrees * ALIGNMENT_KP;
                        if (Math.abs(turnPower) > MAX_TURN_POWER) {
                            turnPower = Math.signum(turnPower) * MAX_TURN_POWER;
                        }
                        telemetry.addData("  Turn Power", "%.3f", turnPower);
                        telemetry.addData("  Error", "%.2f°", bearingDegrees);
                    }
                    telemetry.addData("", "");
                }
            }
        }
        
        telemetry.addData("", "");
        telemetry.addData("TUNING:", "");
        telemetry.addData("KP", ALIGNMENT_KP);
        telemetry.addData("Tolerance", "%.1f°", ALIGNMENT_TOLERANCE);
        telemetry.addData("Max Power", MAX_TURN_POWER);
        
        telemetry.update();
    }
}
