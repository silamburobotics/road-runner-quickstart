package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

// AprilTag imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import android.util.Size;

@Config
@TeleOp(name = "TeleOpDECODE2 (Subsystem)", group = "TeleOp")
public class TeleOpDECODE2 extends LinearOpMode {
    
    // Shooter subsystem
    private ShooterSubsystem shooterSubsystem;
    
    // Color sensors for ball detection
    private NormalizedColorSensor colorSensorIntake;
    private NormalizedColorSensor colorSensorExit;
    
    // Mecanum drive motors
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    
    // AprilTag detection system
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    
    // Button state tracking
    private boolean previousA1 = false;  // Gamepad1 A button (intake)
    private boolean previousB1 = false;  // Gamepad1 B button (shooter speed 1300)
    private boolean previousY1 = false;  // Gamepad1 Y button (shooter speed 1600)
    private boolean previousA2 = false;  // Gamepad2 A button (AprilTag alignment)
    private boolean previousB2 = false;  // Gamepad2 B button (single shot)
    private boolean previousX2 = false;  // Gamepad2 X button (advance indexer)
    private boolean previousY2 = false;  // Gamepad2 Y button (trigger)
    private boolean previousLeftBumper2 = false;  // Gamepad2 left bumper (indexer +10 degrees)
    private boolean previousLeftTrigger2 = false;  // Gamepad2 left trigger (indexer -10 degrees)
    
    // Ball detection variables
    private boolean ballDetectedIntake = false;
    private boolean previousBallDetectedIntake = false;
    private boolean ballDetectedExit = false;
    
    // Intake control variables
    private boolean intakeFromGamepad1 = false;
    private boolean intakeFromGamepad2 = false;
    
    // Color sensor settings
    public static final double COLOR_SENSOR_GAIN = 2.0;
    public static final double BALL_DETECTION_THRESHOLD = 0.04;
    
    // AprilTag settings
    public static final int TARGET_TAG_ID_PRIMARY = 20;
    public static final int TARGET_TAG_ID_SECONDARY = 24;
    public static final double OPTIMAL_SHOOTING_DISTANCE = 24.0;
    
    // Mecanum drive settings
    public static final double DRIVE_SPEED_MULTIPLIER = 0.8;
    public static final double STRAFE_SPEED_MULTIPLIER = 0.8;
    public static final double TURN_SPEED_MULTIPLIER = 0.6;
    
    // AprilTag alignment settings
    public static final double ALIGNMENT_KP = 0.015;  // Proportional gain
    public static final double ALIGNMENT_TOLERANCE = 1.0;  // Degrees
    public static final double ALIGNMENT_TIMEOUT = 1.0;  // Max alignment time in seconds
    private boolean alignmentActive = false;
    private int targetTagId = 0;
    private ElapsedTime alignmentTimer = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeShooterSubsystem();
        initializeIntakeAndDrive();
        initializeAprilTag();
        
        // Restore indexer position from autonomous if available
        if (RobotState.hasIndexerPositionSaved()) {
            shooterSubsystem.setIndexorLastSuccessfulPosition(RobotState.getSavedIndexerPosition());
            telemetry.addData("📥 Indexer Position", "Restored from Auto: %.1f ticks", 
                RobotState.getSavedIndexerPosition());
            long timeSinceSave = System.currentTimeMillis() - RobotState.getSaveTimestamp();
            telemetry.addData("⏱️ Time Since Auto", "%.1f seconds ago", timeSinceSave / 1000.0);
        } else {
            telemetry.addData("📍 Indexer Position", "Starting fresh: %.1f ticks", 
                shooterSubsystem.getIndexorLastSuccessfulPosition());
        }
        
        telemetry.addData("Status", "TeleOpDECODE2 - Using ShooterSubsystem");
        telemetry.addData("", "");
        telemetry.addData("GAMEPAD1 CONTROLS:", "");
        telemetry.addData("A", "Intake Function");
        telemetry.addData("B", "Toggle Shooter Speed 1300");
        telemetry.addData("Y", "Toggle Shooter Speed 1600");
        telemetry.addData("Left Stick", "Drive/Strafe");
        telemetry.addData("Right Stick X", "Turn");
        telemetry.addData("", "");
        telemetry.addData("GAMEPAD2 CONTROLS:", "");
        telemetry.addData("A", "Align with AprilTag");
        telemetry.addData("B", "Single Shot (Fire + Advance)");
        telemetry.addData("X", "Advance Indexer");
        telemetry.addData("Y", "Trigger Function (3-Shot)");
        telemetry.addData("Left Stick -Y", "Outtake Function");
        telemetry.addData("Left Stick +Y", "Intake Function");
        telemetry.addData("Left Bumper", "Indexer +10°");
        telemetry.addData("Left Trigger", "Indexer -10°");
        telemetry.update();
        
        waitForStart();
        
        // Main control loop
        while (opModeIsActive()) {
            readColorSensors();
            handleGamepad1Controls();
            handleGamepad2Controls();
            handleAprilTagAlignment();
            handleMecanumDrive();
            
            // Update shooter subsystem
            shooterSubsystem.update();
            
            updateTelemetry();
            sleep(20);
        }
    }
    
    private void initializeShooterSubsystem() {
        shooterSubsystem = new ShooterSubsystem();
        shooterSubsystem.init(hardwareMap);
        telemetry.addData("✅ ShooterSubsystem", "Initialized");
    }
    
    private void initializeIntakeAndDrive() {
        // Initialize mecanum drive motors
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
        
        // Initialize color sensors
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "colorSensorEntry");
        colorSensorIntake.setGain((float)COLOR_SENSOR_GAIN);
        
        colorSensorExit = hardwareMap.get(NormalizedColorSensor.class, "colorSensorExit");
        colorSensorExit.setGain((float)COLOR_SENSOR_GAIN);
        
        // Enable LED lights if available
        if (colorSensorIntake instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorIntake).enableLight(true);
        }
        if (colorSensorExit instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorExit).enableLight(true);
        }
        
        telemetry.addData("✅ Intake & Drive", "Initialized");
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
            
            telemetry.addData("✅ AprilTag System", "Initialized");
            
        } catch (Exception e) {
            telemetry.addData("❌ AprilTag Error", e.getMessage());
            visionPortal = null;
            aprilTag = null;
        }
    }
    
    private void readColorSensors() {
        NormalizedRGBA colorsIntake = colorSensorIntake.getNormalizedColors();
        previousBallDetectedIntake = ballDetectedIntake;
        ballDetectedIntake = colorsIntake.alpha > BALL_DETECTION_THRESHOLD;
        
        NormalizedRGBA colorsExit = colorSensorExit.getNormalizedColors();
        ballDetectedExit = colorsExit.alpha > BALL_DETECTION_THRESHOLD;
    }
    
    private void handleGamepad1Controls() {
        boolean currentA1 = gamepad1.a;
        boolean currentB1 = gamepad1.b;
        boolean currentY1 = gamepad1.y;
        
        if (currentA1 && !previousA1) {
            intakeFunction();
        }
        
        if (currentB1 && !previousB1) {
            shooterSubsystem.toggleShooterSpeed(ShooterSubsystem.SHOOTER_TARGET_VELOCITY_1300);
        }
        
        if (currentY1 && !previousY1) {
            shooterSubsystem.toggleShooterSpeed(ShooterSubsystem.SHOOTER_TARGET_VELOCITY_1600);
        }
        
        previousA1 = currentA1;
        previousB1 = currentB1;
        previousY1 = currentY1;
    }
    
    private void handleGamepad2Controls() {
        // Handle joystick Y-axis for outtake/intake
        double joystickY = -gamepad2.left_stick_y;
        
        if (joystickY < -0.1) {
            outtakeFunction();
        } else if (joystickY > 0.1) {
            intakeFromJoystick();
        } else {
            if (shooterSubsystem.isIntakeRunning() && shooterSubsystem.getIntakePower() < 0) {
                stopOuttake();
            }
            if (intakeFromGamepad2 && shooterSubsystem.isIntakeRunning() && 
                shooterSubsystem.getIntakePower() > 0) {
                stopIntakeFunction();
            }
        }
        
        boolean currentA2 = gamepad2.a;
        boolean currentB2 = gamepad2.b;
        boolean currentX2 = gamepad2.x;
        boolean currentY2 = gamepad2.y;
        boolean currentLeftBumper2 = gamepad2.left_bumper;
        boolean currentLeftTrigger2 = gamepad2.left_trigger > 0.5;
        
        if (currentA2 && !previousA2) {
            startAprilTagAlignment();
        }
        
        if (currentB2 && !previousB2) {
            singleShotFunction();
        }
        
        if (currentX2 && !previousX2) {
            shooterSubsystem.advanceIndexer();
        }
        
        if (currentY2 && !previousY2) {
            if (!shooterSubsystem.isShooterRunning()) {
                telemetry.addData("⚠️ Trigger", "Shooter not running - use gamepad1 B/Y first");
                telemetry.update();
            } else {
                shooterSubsystem.startTriggerSequence();
            }
        }
        
        if (currentLeftBumper2 && !previousLeftBumper2) {
            shooterSubsystem.adjustIndexerRotation(10);
        }
        
        if (currentLeftTrigger2 && !previousLeftTrigger2) {
            shooterSubsystem.adjustIndexerRotation(-10);
        }
        
        previousA2 = currentA2;
        previousB2 = currentB2;
        previousX2 = currentX2;
        previousY2 = currentY2;
        previousLeftBumper2 = currentLeftBumper2;
        previousLeftTrigger2 = currentLeftTrigger2;
    }
    
    private void intakeFunction() {
        boolean intakeRunning = shooterSubsystem.isIntakeRunning();
        
        if (!intakeRunning) {
            shooterSubsystem.startIntake();
            shooterSubsystem.startConveyor();
            
            intakeFromGamepad1 = true;
            intakeFromGamepad2 = false;
            
            telemetry.addData("✅ Intake Function", "STARTED");
        } else {
            shooterSubsystem.stopIntake();
            if (!shooterSubsystem.isIndexorMoving()) {
                shooterSubsystem.stopConveyor();
            }
            
            intakeFromGamepad1 = false;
            intakeFromGamepad2 = false;
            
            telemetry.addData("⏹️ Intake Function", "STOPPED");
        }
        
        // Auto-advance indexer when ball detected
        if (ballDetectedIntake && !previousBallDetectedIntake) {
            boolean currentIntakeRunning = shooterSubsystem.isIntakeRunning() && 
                                          shooterSubsystem.getIntakePower() > 0;
            
            if (currentIntakeRunning && !shooterSubsystem.isIndexorMoving()) {
                shooterSubsystem.advanceIndexer();
                telemetry.addData("🎾 Auto Advance", "Ball detected");
            }
        }
        
        telemetry.update();
    }
    
    private void outtakeFunction() {
        shooterSubsystem.startIntakeReverse();
        shooterSubsystem.startConveyorReverse();
        shooterSubsystem.holdIndexer();
        
        telemetry.addData("🔄 Outtake Function", "ACTIVE");
    }
    
    private void stopOuttake() {
        shooterSubsystem.stopIntake();
        shooterSubsystem.stopConveyor();
        shooterSubsystem.releaseIndexer();
        
        intakeFromGamepad1 = false;
        intakeFromGamepad2 = false;
        
        telemetry.addData("⏹️ Outtake Function", "STOPPED");
    }
    
    private void stopIntakeFunction() {
        shooterSubsystem.stopIntake();
        if (!shooterSubsystem.isIndexorMoving()) {
            shooterSubsystem.stopConveyor();
        }
        
        intakeFromGamepad1 = false;
        intakeFromGamepad2 = false;
    }
    
    private void intakeFromJoystick() {
        boolean intakeRunning = shooterSubsystem.isIntakeRunning();
        
        if (!intakeRunning) {
            shooterSubsystem.startIntake();
            shooterSubsystem.startConveyor();
            
            intakeFromGamepad2 = true;
            intakeFromGamepad1 = false;
        }
        
        // Auto-advance indexer when ball detected
        if (ballDetectedIntake && !previousBallDetectedIntake) {
            boolean currentIntakeRunning = shooterSubsystem.isIntakeRunning() && 
                                          shooterSubsystem.getIntakePower() > 0;
            
            if (currentIntakeRunning && !shooterSubsystem.isIndexorMoving()) {
                shooterSubsystem.advanceIndexer();
            }
        }
    }
    
    private void singleShotFunction() {
        if (!shooterSubsystem.isShooterRunning()) {
            telemetry.addData("⚠️ Single Shot", "Shooter not running - use gamepad1 B/Y first");
            telemetry.update();
            return;
        }
        
        // Fire trigger for single shot
        new Thread(() -> {
            try {
                // Fire
                shooterSubsystem.getTriggerServo().setPosition(ShooterSubsystem.TRIGGER_FIRE);
                Thread.sleep((long)(ShooterSubsystem.TRIGGER_FIRE_DURATION * 1000));
                
                // Return to home
                shooterSubsystem.getTriggerServo().setPosition(ShooterSubsystem.TRIGGER_HOME);
                Thread.sleep((long)(ShooterSubsystem.INDEXER_ADVANCE_WAIT * 1000));
                
                // Advance indexer
                shooterSubsystem.advanceIndexer();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }
    
    private void startAprilTagAlignment() {
        if (alignmentActive || visionPortal == null || aprilTag == null) {
            return;
        }
        
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection targetTag = null;
        
        // Find tag 20 or 24
        for (AprilTagDetection detection : detections) {
            if ((detection.id == TARGET_TAG_ID_PRIMARY || detection.id == TARGET_TAG_ID_SECONDARY) 
                && detection.ftcPose != null) {
                targetTag = detection;
                break;
            }
        }
        
        if (targetTag != null) {
            targetTagId = targetTag.id;
            alignmentActive = true;
            alignmentTimer.reset();
        }
    }
    
    private void handleAprilTagAlignment() {
        if (!alignmentActive) {
            return;
        }
        
        // Check timeout
        if (alignmentTimer.seconds() > ALIGNMENT_TIMEOUT) {
            alignmentActive = false;
            setDrivePower(0, 0, 0, 0);
            return;
        }
        
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection currentTag = null;
        
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId && detection.ftcPose != null) {
                currentTag = detection;
                break;
            }
        }
        
        if (currentTag == null) {
            alignmentActive = false;
            setDrivePower(0, 0, 0, 0);
            return;
        }
        
        double bearingDegrees = Math.toDegrees(currentTag.ftcPose.bearing);
        
        if (Math.abs(bearingDegrees) <= ALIGNMENT_TOLERANCE) {
            alignmentActive = false;
            setDrivePower(0, 0, 0, 0);
            return;
        }
        
        double turnPower = bearingDegrees * ALIGNMENT_KP;
        setDrivePower(turnPower, -turnPower, turnPower, -turnPower);
    }
    
    private void handleMecanumDrive() {
        if (alignmentActive) {
            return;
        }
        
        double drive = -gamepad1.left_stick_y * DRIVE_SPEED_MULTIPLIER;
        double strafe = gamepad1.left_stick_x * STRAFE_SPEED_MULTIPLIER;
        double turn = gamepad1.right_stick_x * TURN_SPEED_MULTIPLIER;
        
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
    
    private void setDrivePower(double lf, double rf, double lb, double rb) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }
    
    private void updateTelemetry() {
        telemetry.addData("=== SHOOTER SUBSYSTEM ===", "");
        telemetry.addData("Shooter", shooterSubsystem.isShooterRunning() ? 
            "ON (%.0f tps)" : "OFF", shooterSubsystem.getCurrentShooterVelocity());
        telemetry.addData("Indexer Position", "%.1f ticks", 
            shooterSubsystem.getIndexorLastSuccessfulPosition());
        telemetry.addData("Indexer Status", shooterSubsystem.isIndexorMoving() ? "MOVING" : "IDLE");
        telemetry.addData("Trigger Sequence", shooterSubsystem.isTriggerSequenceActive() ? "ACTIVE" : "IDLE");
        
        telemetry.addData("", "");
        telemetry.addData("=== INTAKE SYSTEM ===", "");
        telemetry.addData("Intake Power", "%.2f", shooterSubsystem.getIntakePower());
        telemetry.addData("Ball Detected (Intake)", ballDetectedIntake ? "YES" : "NO");
        telemetry.addData("Ball Detected (Exit)", ballDetectedExit ? "YES" : "NO");
        
        telemetry.addData("", "");
        telemetry.addData("=== VISION ===", "");
        if (visionPortal != null) {
            telemetry.addData("Camera", visionPortal.getCameraState());
            if (aprilTag != null) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                telemetry.addData("Tags Found", detections.size());
                
                // Show details for each detected tag
                for (AprilTagDetection detection : detections) {
                    if (detection.ftcPose != null) {
                        telemetry.addData("Tag ID", detection.id);
                        telemetry.addData("  Range", "%.2f inches", detection.ftcPose.range);
                        telemetry.addData("  Bearing", "%.1f°", Math.toDegrees(detection.ftcPose.bearing));
                        telemetry.addData("  Yaw", "%.1f°", Math.toDegrees(detection.ftcPose.yaw));
                    }
                }
            }
        }
        
        telemetry.update();
    }
}
