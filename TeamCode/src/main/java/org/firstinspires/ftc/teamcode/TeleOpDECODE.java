package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
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
@TeleOp(name = "TeleOpDECODESimple2", group = "TeleOp")
public class TeleOpDECODE extends LinearOpMode {
    
    // Declare motors
    private DcMotorEx indexor;
    private DcMotorEx intake;
    private DcMotorEx conveyor;
    private DcMotorEx shooter;
    
    // Declare servos
    private CRServo shooterServo;
    private Servo speedLight;
    private Servo triggerServo;
    
    // Declare color sensor for intake ball detection
    private NormalizedColorSensor colorSensorIntake;
    
    // Declare color sensor for exit position
    private NormalizedColorSensor colorSensorExit;
    
    // Declare mecanum drive motors
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    
    // AprilTag detection system
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    
    // Variables to track button states
    private boolean previousA1 = false;  // Gamepad1 A button (intake)
    private boolean previousB1 = false;  // Gamepad1 B button (shooter speed 1300)
    private boolean previousY1 = false;  // Gamepad1 Y button (shooter speed 1600)
    private boolean previousA2 = false;  // Gamepad2 A button (AprilTag alignment)
    private boolean previousB2 = false;  // Gamepad2 B button (trigger)
    private boolean previousX2 = false;  // Gamepad2 X button (advance indexer)
    private boolean previousY2 = false;  // Gamepad2 Y button (3-shot sequence)
    private boolean previousLeftBumper2 = false;  // Gamepad2 left bumper (indexer +10 degrees)
    private boolean previousLeftTrigger2 = false;  // Gamepad2 left trigger (indexer -10 degrees)
    
    // Ball detection variables
    private boolean ballDetectedIntake = false;
    private boolean previousBallDetectedIntake = false;
    private boolean ballDetectedExit = false;
    
    // Intake control variables
    private boolean intakeFromGamepad1 = false;  // Track if intake was started by gamepad1 A button
    private boolean intakeFromGamepad2 = false;  // Track if intake was started by gamepad2 joystick
    
    // Indexor control variables
    private double indexorLastSuccessfulPosition = 0.0;  // Last successful indexor position
    private boolean indexorMoving = false;
    private ElapsedTime indexorTimer = new ElapsedTime();
    private int indexorStartPosition = 0;
    
    // Shooter variables
    private boolean shooterRunning = false;
    private double currentShooterVelocity = 1300;  // Default velocity
    private ElapsedTime shooterStabilizationTimer = new ElapsedTime(); // Timer for speed stabilization
    private boolean shooterSpeedStable = false; // Track if speed is stable
    
    // Trigger sequence variables
    private boolean triggerSequenceActive = false;
    private boolean isThreeShotMode = false;  // Track if doing 3-shot sequence or single shot
    private ElapsedTime triggerTimer = new ElapsedTime();
    private int triggerSequenceStep = 0;  // 0=home, 1-8 for 3-shot sequence
    private int shotsFired = 0;
    private static final double INDEXER_ADVANCE_WAIT = 0.2;  // Wait time for indexer to advance

    // Shooter PID coefficients for velocity control
    public static double VELOCITY_P = 4;  // Proportional coefficient (increased for faster response on restart)
    public static double VELOCITY_I = 0.15;  // Integral coefficient
    public static double VELOCITY_D = 0.3;  // Derivative coefficient
    public static double VELOCITY_F = 13.0;  // Feedforward coefficient

    // Motor power settings
    public static final double INTAKE_POWER = 0.8;
    public static final double CONVEYOR_POWER = 1.0;
    public static final double INDEXOR_POWER = 0.5;
    public static final double SHOOTER_SERVO_POWER = 1.0;
    
    // Indexor position settings
    public static final double INDEXOR_TICKS_PER_REVOLUTION = 537.7;  // goBILDA 312 RPM motor
    public static final double INDEXOR_TICKS_PER_120_DEGREES = INDEXOR_TICKS_PER_REVOLUTION / 3.0;  // 179.23 ticks per 120°
    public static final double INDEXOR_TICKS_PER_DEGREE = INDEXOR_TICKS_PER_REVOLUTION / 360.0;  // ~1.49 ticks per degree
    public static final double INDEXOR_TICKS_PER_10_DEGREES = INDEXOR_TICKS_PER_DEGREE * 10.0;  // ~14.94 ticks per 10°
    
    // Shooter velocity settings
    public static final double SHOOTER_TARGET_VELOCITY_1300 = 1270;  // B button velocity
    public static final double SHOOTER_TARGET_VELOCITY_1600 = 1550;  // Y button velocity
    
    // Color sensor settings
    public static final double COLOR_SENSOR_GAIN = 15.0;
    public static final double BALL_DETECTION_THRESHOLD = 0.15;
    
    // Trigger servo positions
    public static final double TRIGGER_FIRE = 0.0;     // Fire position (27.0 degrees)
    public static final double TRIGGER_HOME = 0.5;     // Home position (104.4 degrees)
    public static final double TRIGGER_FIRE_DURATION = 0.5;  // Fire duration in seconds
    
    // Speed light control settings (using servo positions for LED control)
    public static final double LIGHT_OFF_POSITION = 0.0;      // Servo position for light off
    public static final double LIGHT_GREEN_POSITION = 0.5;    // Servo position for green light
    public static final double LIGHT_WHITE_POSITION = 1.0;    // Servo position for white light
    
    // Speed monitoring thresholds
    public static final double SHOOTER_SPEED_THRESHOLD = 0.95; // 95% of target speed for green light
    public static final double SHOOTER_MIN_SPEED_THRESHOLD = 0.85; // 85% minimum for white light
    public static final double SHOOTER_SPEED_TOLERANCE = 50;       // ticks/sec tolerance for "stable" speed
    public static final double SHOOTER_STABILIZATION_TIME = 0.3;   // Seconds to wait for speed stabilization (reduced from 1.0)
    
    // Indexor stuck detection
    public static final double INDEXOR_STUCK_TIMEOUT = 0.5;  // 0.5 seconds as specified
    public static final int INDEXOR_STUCK_THRESHOLD = 10;    // Minimum movement required
    
    // AprilTag settings
    public static final int TARGET_TAG_ID_PRIMARY = 20;
    public static final int TARGET_TAG_ID_SECONDARY = 24;
    public static final double OPTIMAL_SHOOTING_DISTANCE = 24.0; // inches
    
    // Mecanum drive settings
    public static final double DRIVE_SPEED_MULTIPLIER = 0.8;
    public static final double STRAFE_SPEED_MULTIPLIER = 0.8;
    public static final double TURN_SPEED_MULTIPLIER = 0.6;
    
    // AprilTag alignment settings
    public static final double ALIGNMENT_TURN_POWER = 0.3;    // Power for alignment turns
    public static final double ALIGNMENT_TOLERANCE = 2.0;      // Degrees tolerance for "aligned"
    public static final double ALIGNMENT_TIMEOUT = 1.0;        // Maximum time for alignment attempt (seconds)
    private boolean alignmentActive = false;
    private int targetTagId = 0;
    private ElapsedTime alignmentTimer = new ElapsedTime();
    private double targetBearing = 0.0;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeMotors();
        initializeAprilTag();
        
        // Restore indexor position from autonomous if available
        if (RobotState.hasIndexerPositionSaved()) {
            indexorLastSuccessfulPosition = RobotState.getSavedIndexerPosition();
            telemetry.addData("📥 Indexer Position", "Restored from Auto: %.1f ticks", indexorLastSuccessfulPosition);
            long timeSinceSave = System.currentTimeMillis() - RobotState.getSaveTimestamp();
            telemetry.addData("⏱️ Time Since Auto", "%.1f seconds ago", timeSinceSave / 1000.0);
        } else {
            // No saved position - starting fresh (encoder was reset in initializeMotors)
            indexorLastSuccessfulPosition = indexor.getCurrentPosition();
            telemetry.addData("📍 Indexer Position", "Starting fresh: %.1f ticks", indexorLastSuccessfulPosition);
        }
        
        telemetry.addData("Status", "TeleOpDECODESimple2 - Initialized");
        telemetry.addData("Indexor Position", "Preserved: %.1f ticks", indexorLastSuccessfulPosition);
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
        telemetry.addData("X", "Advance Indexer");
        telemetry.addData("B", "Trigger Function (Single Shot)");
        telemetry.addData("Y", "Three-Shot Sequence");
        telemetry.addData("Left Stick -Y", "Outtake Function");
        telemetry.addData("Left Stick +Y", "Intake Function");
        telemetry.update();
        
        waitForStart();
        
        // Main control loop
        while (opModeIsActive()) {
            readColorSensors();
            handleGamepad1Controls();
            handleGamepad2Controls();
            handleAprilTagAlignment();
            handleIndexorStuckDetection();
            handleTriggerSequence();
            updateShooterSpeedMonitoring();
            updateSpeedLight();
            handleMecanumDrive();
            updateTelemetry();
            sleep(20);
        }
    }
    
    private void initializeMotors() {
        // Initialize motors
        indexor = hardwareMap.get(DcMotorEx.class, "indexor");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Set custom PID coefficients for shooter velocity control
        shooter.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);

        // Initialize servos
        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
        speedLight = hardwareMap.get(Servo.class, "speedLight");
        triggerServo = hardwareMap.get(Servo.class, "triggerServo");
        
        // Initialize mecanum drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        
        // Set motor directions (from original)
        indexor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        conveyor.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        
        // Set servo directions
        shooterServo.setDirection(DcMotorSimple.Direction.REVERSE);
        speedLight.setDirection(Servo.Direction.FORWARD);
        triggerServo.setDirection(Servo.Direction.FORWARD);
        
        // Initialize servos to default positions
        speedLight.setPosition(LIGHT_OFF_POSITION);
        triggerServo.setPosition(TRIGGER_HOME);
        
        // Initialize color sensor
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "colorSensorEntry");
        colorSensorIntake.setGain((float)COLOR_SENSOR_GAIN);
        
        // Initialize exit color sensor
        colorSensorExit = hardwareMap.get(NormalizedColorSensor.class, "colorSensorExit");
        colorSensorExit.setGain((float)COLOR_SENSOR_GAIN);
        
        // Enable LED light if available
        if (colorSensorIntake instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorIntake).enableLight(true);
        }
        
        // Enable LED light for exit sensor if available
        if (colorSensorExit instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorExit).enableLight(true);
        }
        
        // Set mecanum drive motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        
        // Set zero power behavior
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Set motor modes
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Preserve position from auto
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Configure drive motors
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set zero power behavior for drive
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void initializeAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setLensIntrinsics(902.577, 902.577, 612.676, 364.762)  // OV9281 Arducam 1280x720
                .build();

        try {
            // Create the vision portal with MJPEG format for better performance
            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(1280, 720));
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);  // Use MJPEG for 30 FPS instead of YUY2 (10 FPS)
            builder.enableLiveView(true);
            builder.setAutoStopLiveView(false);
            builder.addProcessor(aprilTag);
            
            visionPortal = builder.build();
            
            // Wait for vision portal to initialize
            while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Vision Portal", "Initializing...");
                telemetry.update();
                sleep(20);
            }
            
            telemetry.addData("✅ AprilTag System", "Initialized successfully");
            
        } catch (Exception e) {
            telemetry.addData("❌ AprilTag Error", "Failed to initialize: %s", e.getMessage());
            telemetry.update();
            visionPortal = null;
            aprilTag = null;
        }
    }
    
    private void readColorSensors() {
        // Read intake color sensor
        NormalizedRGBA colorsIntake = colorSensorIntake.getNormalizedColors();
        
        // Update previous state
        previousBallDetectedIntake = ballDetectedIntake;
        
        // Detect ball using alpha channel
        ballDetectedIntake = colorsIntake.alpha > BALL_DETECTION_THRESHOLD;
        
        // Read exit color sensor
        NormalizedRGBA colorsExit = colorSensorExit.getNormalizedColors();
        
        // Detect ball in exit position
        ballDetectedExit = colorsExit.alpha > BALL_DETECTION_THRESHOLD;
    }
    
    private void handleGamepad1Controls() {
        // Get current button states for gamepad1
        boolean currentA1 = gamepad1.a;
        boolean currentB1 = gamepad1.b;
        boolean currentY1 = gamepad1.y;
        
        // Handle A button - Intake Function
        if (currentA1 && !previousA1) {
            intakeFunction();
        }
        
        // Handle B button - Toggle Shooter Speed 1300
        if (currentB1 && !previousB1) {
            toggleShooterSpeed(SHOOTER_TARGET_VELOCITY_1300);
        }
        
        // Handle Y button - Toggle Shooter Speed 1600
        if (currentY1 && !previousY1) {
            toggleShooterSpeed(SHOOTER_TARGET_VELOCITY_1600);
        }
        
        // Update previous states
        previousA1 = currentA1;
        previousB1 = currentB1;
        previousY1 = currentY1;
    }
    
    private void handleGamepad2Controls() {
        // Handle joystick Y-axis - Outtake Function (-Y) and Intake Function (+Y)
        double joystickY = -gamepad2.left_stick_y;
        
        if (joystickY < -0.1) {  // Negative Y (joystick pushed up) - Outtake Function
            outtakeFunction();
        } else if (joystickY > 0.1) {  // Positive Y (joystick pushed down) - Intake Function
            intakeFromJoystick();
        } else {
            // Stop functions when joystick released - only if they were started by joystick
            if (Math.abs(intake.getPower()) > 0 && intake.getPower() < 0) {
                stopOuttake();
            }
            if (intakeFromGamepad2 && Math.abs(intake.getPower()) > 0 && intake.getPower() > 0) {
                stopIntakeFunction();
            }
        }
        
        // Get current button states for gamepad2
        boolean currentA2 = gamepad2.a;
        boolean currentX2 = gamepad2.x;
        boolean currentB2 = gamepad2.b;
        boolean currentY2 = gamepad2.y;
        boolean currentLeftBumper2 = gamepad2.left_bumper;
        boolean currentLeftTrigger2 = gamepad2.left_trigger > 0.5;  // Treat trigger as button when > 50%
        
        // Handle A button - Align with AprilTag
        if (currentA2 && !previousA2) {
            startAprilTagAlignment();
        }
        
        // Handle X button - Advance Indexer
        if (currentX2 && !previousX2) {
            advanceIndexer();
        }
        
        // Handle B button - Trigger Function (single shot)
        if (currentB2 && !previousB2) {
            triggerFunction();
        }
        
        // Handle Y button - Three Shot Sequence
        if (currentY2 && !previousY2) {
            startThreeShotSequence();
        }
        
        // Handle Left Bumper - Increment indexer rotation by 10 degrees
        if (currentLeftBumper2 && !previousLeftBumper2) {
            adjustIndexerRotation(10);
        }
        
        // Handle Left Trigger - Decrement indexer rotation by 10 degrees
        if (currentLeftTrigger2 && !previousLeftTrigger2) {
            adjustIndexerRotation(-10);
        }
        
        // Update previous states
        previousA2 = currentA2;
        previousX2 = currentX2;
        previousB2 = currentB2;
        previousY2 = currentY2;
        previousLeftBumper2 = currentLeftBumper2;
        previousLeftTrigger2 = currentLeftTrigger2;
    }
    
    /**
     * Function Intake (gamepad1 A button)
     * 1) Run the intake wheels forward
     * 2) Run the conveyor forward  
     * 3) Advance the indexer forward when ball detected in the intake sensor
     * Note: Exit sensor auto-stop removed due to false detection issues
     */
    private void intakeFunction() {
        boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
        
        if (!intakeRunning) {
            // Start intake (removed exit sensor check due to false detections)
            intake.setPower(INTAKE_POWER);           // 1) Run intake wheels forward
            conveyor.setPower(CONVEYOR_POWER);       // 2) Run conveyor forward
            
            // Track that intake was started by gamepad1
            intakeFromGamepad1 = true;
            intakeFromGamepad2 = false;
            
            telemetry.addData("✅ Intake Function", "STARTED");
            telemetry.addData("Intake Power", "%.1f", INTAKE_POWER);
            telemetry.addData("Conveyor Power", "%.1f", CONVEYOR_POWER);
            telemetry.addData("Ball Detection", "Monitoring for auto-advance");
        } else {
            // Stop intake (but check if indexer is running before stopping conveyor)
            intake.setPower(0);
            if (!indexorMoving) {
                conveyor.setPower(0);  // Only stop conveyor if indexer is not running
            }
            
            // Clear intake source flags
            intakeFromGamepad1 = false;
            intakeFromGamepad2 = false;
            
            telemetry.addData("⏹️ Intake Function", "STOPPED");
        }
        
        // 3) Advance the indexer forward when ball detected in the intake sensor
        // Check for rising edge of ball detection during intake
        if (ballDetectedIntake && !previousBallDetectedIntake) {
            // Ball detected - check if intake is running
            boolean currentIntakeRunning = Math.abs(intake.getPower()) > 0.1 && intake.getPower() > 0;
            
            if (currentIntakeRunning && !indexorMoving) {
                // Auto-advance indexer when ball detected during intake
                advanceIndexer();
                telemetry.addData("🎾 Auto Advance", "Ball detected - advancing indexer");
            }
        }
        
        telemetry.update();
    }
    
    /**
     * Function Outtake (gamepad2 joystick -Y)
     * 1) Run the intake wheels rearward
     * 2) Run the conveyor rearward
     * 3) Hold the indexer
     */
    private void outtakeFunction() {
        intake.setPower(-INTAKE_POWER);          // 1) Run intake wheels rearward
        conveyor.setPower(-CONVEYOR_POWER);      // 2) Run conveyor rearward
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // 3) Hold the indexer
        indexor.setPower(0);
        
        telemetry.addData("🔄 Outtake Function", "ACTIVE");
        telemetry.addData("Intake", "REVERSE at %.1f", -INTAKE_POWER);
        telemetry.addData("Conveyor", "REVERSE at %.1f", -CONVEYOR_POWER);
        telemetry.addData("Indexor", "HOLDING (BRAKE mode)");
    }
    
    private void stopOuttake() {
        intake.setPower(0);
        conveyor.setPower(0);
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        indexor.setPower(0);  // Ensure indexer is set to float mode when outtake stops
        
        // Clear intake source flags when stopping outtake
        intakeFromGamepad1 = false;
        intakeFromGamepad2 = false;
        
        telemetry.addData("⏹️ Outtake Function", "STOPPED");
        telemetry.addData("Indexor", "Set to FLOAT mode");
    }
    
    private void stopIntakeFunction() {
        intake.setPower(0);
        if (!indexorMoving) {
            conveyor.setPower(0);  // Only stop conveyor if indexer is not running
        }
        
        // Clear intake source flags
        intakeFromGamepad1 = false;
        intakeFromGamepad2 = false;
        
        telemetry.addData("⏹️ Intake Function", "STOPPED");
    }
    
    /**
     * Intake from gamepad2 joystick - similar to intakeFunction but tracks joystick source
     */
    private void intakeFromJoystick() {
        boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
        
        if (!intakeRunning) {
            // Start intake
            intake.setPower(INTAKE_POWER);           // 1) Run intake wheels forward
            conveyor.setPower(CONVEYOR_POWER);       // 2) Run conveyor forward
            
            // Track that intake was started by gamepad2 joystick
            intakeFromGamepad2 = true;
            intakeFromGamepad1 = false;
        }
        
        // 3) Advance the indexer forward when ball detected in the intake sensor
        // Check for rising edge of ball detection during intake
        if (ballDetectedIntake && !previousBallDetectedIntake) {
            // Ball detected - check if intake is running
            boolean currentIntakeRunning = Math.abs(intake.getPower()) > 0.1 && intake.getPower() > 0;
            
            if (currentIntakeRunning && !indexorMoving) {
                // Auto-advance indexer when ball detected during intake
                advanceIndexer();
                telemetry.addData("🎾 Auto Advance", "Ball detected - advancing indexer");
            }
        }
    }
    
    /**
     * Toggle Shooter Speed (gamepad1 B/Y buttons)
     * B button: Toggle 1300 ticks/sec
     * Y button: Toggle 1600 ticks/sec
     * If shooter is off or running at different speed, start at specified speed
     * If shooter is already running at specified speed, stop shooter
     */
    private void toggleShooterSpeed(double velocity) {
        if (shooterRunning && Math.abs(currentShooterVelocity - velocity) < 50) {
            // Shooter is already running at this speed - stop it
            shooter.setVelocity(0);
            shooterServo.setPower(0);
            shooterRunning = false;
            currentShooterVelocity = 0;
            shooterSpeedStable = false;
            
            telemetry.addData("⏹️ Shooter STOPPED", "Was running at %.0f ticks/sec", velocity);
            telemetry.addData("Shooter Status", "OFF");
        } else {
            // Shooter is off or running at different speed - start at specified velocity
            shooter.setVelocity(velocity);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            shooterRunning = true;
            currentShooterVelocity = velocity;
            shooterStabilizationTimer.reset();
            shooterSpeedStable = false;
            
            telemetry.addData("🎯 Shooter STARTED", "%.0f ticks/sec", velocity);
            telemetry.addData("Shooter Status", "RUNNING");
        }
        telemetry.update();
    }
    
    /**
     * Function Advance Indexer
     * 1) Advance indexer to next 120°
     * 2) Use mod function to calculate next position
     * 3) Only advance the indexer to next level based on successful previous advancement
     */
    private void advanceIndexer() {
        // Check if indexer is already moving
        if (indexorMoving) {
            telemetry.addData("⚠️ Indexer", "Already moving - please wait");
            return;
        }
        
        // Get current actual position for telemetry only
        double currentActualPosition = (double) indexor.getCurrentPosition();
        double currentRemainder = currentActualPosition % INDEXOR_TICKS_PER_120_DEGREES;
        
        // Next position is always increment of 120 degrees from previous successful position
        double nextPosition = indexorLastSuccessfulPosition + INDEXOR_TICKS_PER_120_DEGREES;
        
        // Calculate step information for telemetry
        int lastSuccessfulStep = (int) Math.round(indexorLastSuccessfulPosition / INDEXOR_TICKS_PER_120_DEGREES) % 3;
        int nextStep = (int) Math.round(nextPosition / INDEXOR_TICKS_PER_120_DEGREES) % 3;
        
        // Set target position
        indexor.setTargetPosition((int) Math.round(nextPosition));
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(INDEXOR_POWER);
        
        // Run conveyor when indexer is running (ensure it's running regardless of intake status)
        conveyor.setPower(CONVEYOR_POWER);
        
        // Start movement tracking
        indexorMoving = true;
        indexorTimer.reset();
        indexorStartPosition = indexor.getCurrentPosition();
        
        telemetry.addData("🎯 Advance Indexer", "Moving to next 120°");
        telemetry.addData("Current Actual", "%.1f ticks (remainder: %.1f)", currentActualPosition, currentRemainder);
        telemetry.addData("Last Successful", "%.1f ticks (step %d)", indexorLastSuccessfulPosition, lastSuccessfulStep);
        telemetry.addData("Target Position", "%.1f ticks (step %d)", nextPosition, nextStep);
        telemetry.addData("120° Advancement", "Step %d → Step %d", lastSuccessfulStep, nextStep);
        telemetry.update();
    }
    
    /**
     * Handle indexer stuck detection
     * if indexer stuck for 0.5sec put the indexer in float mode
     * Only update successful position when movement completes successfully to exact target
     */
    private void handleIndexorStuckDetection() {
        if (!indexorMoving) {
            return;
        }
        
        // Check if indexer reached target
        if (!indexor.isBusy()) {
            // Check if indexer actually reached the target position (within tolerance)
            int currentPosition = indexor.getCurrentPosition();
            int targetPosition = indexor.getTargetPosition();
            int positionError = Math.abs(currentPosition - targetPosition);
            
            if (positionError <= 15) {  // Within 15 ticks tolerance for successful 120° advancement
                // Movement completed successfully to target - update successful position
                indexorLastSuccessfulPosition = targetPosition;
                telemetry.addData("✅ Indexer", "Successfully advanced to %.1f", indexorLastSuccessfulPosition);
            } else {
                // Movement completed but not at target position - DO NOT update successful position
                telemetry.addData("⚠️ Indexer", "Reached end but not at target (error: %d ticks)", positionError);
                telemetry.addData("Target", "%d, Actual: %d", targetPosition, currentPosition);
                telemetry.addData("Keeping Previous", "Successful position: %.1f", indexorLastSuccessfulPosition);
            }
            
            indexorMoving = false;
            indexor.setPower(0);
            // Only stop conveyor if intake is not running
            if (Math.abs(intake.getPower()) <= 0.1) {
                conveyor.setPower(0);
            }
            return;
        }
        
        // Check for stuck condition (0.5 seconds as specified)
        if (indexorTimer.seconds() > INDEXOR_STUCK_TIMEOUT) {
            int currentPosition = indexor.getCurrentPosition();
            int movement = Math.abs(currentPosition - indexorStartPosition);
            
            if (movement < INDEXOR_STUCK_THRESHOLD) {
                // Indexer is stuck - put in float mode
                // DO NOT update indexorLastSuccessfulPosition - keep previous successful position
                
                // First stop the motor power
                indexor.setPower(0);
                
                // Change to RUN_WITHOUT_ENCODER mode to exit RUN_TO_POSITION mode
                indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                
                // Small delay to ensure mode change takes effect
                try {
                    Thread.sleep(50);  // 50ms delay for mode change
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                
                // Now set to FLOAT behavior
                indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                
                // Ensure motor is truly floating by setting power to 0 again after mode change
                indexor.setPower(0);
                
                // Only stop conveyor if intake is not running
                if (Math.abs(intake.getPower()) <= 0.1) {
                    conveyor.setPower(0);
                }
                
                indexorMoving = false;
                
                telemetry.addData("⚠️ Indexer STUCK", "Put in FLOAT mode");
                telemetry.addData("Movement", "%d ticks in %.1f seconds", movement, indexorTimer.seconds());
                telemetry.addData("Keeping Previous", "Successful position: %.1f", indexorLastSuccessfulPosition);
                telemetry.addData("Mode", "RUN_WITHOUT_ENCODER + FLOAT");
                telemetry.update();
            }
        }
    }
    
    /**
     * Adjust indexer rotation by specified degrees
     * @param degrees positive for clockwise, negative for counter-clockwise
     */
    private void adjustIndexerRotation(double degrees) {
        // Check if indexer is already moving
        if (indexorMoving) {
            telemetry.addData("⚠️ Indexer", "Already moving - cannot adjust");
            return;
        }
        
        // Calculate ticks for the adjustment
        double adjustmentTicks = degrees * INDEXOR_TICKS_PER_DEGREE;
        
        // Get current position (use actual position for fine adjustments)
        int currentPosition = indexor.getCurrentPosition();
        double targetPosition = currentPosition + adjustmentTicks;
        
        // Set target position
        indexor.setTargetPosition((int) Math.round(targetPosition));
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(INDEXOR_POWER * 0.5);  // Use half power for fine adjustments
        
        // Start timing and tracking
        indexorTimer.reset();
        indexorMoving = true;
        indexorStartPosition = currentPosition;
        
        telemetry.addData("🎯 Fine Adjust", "%.1f° (%.1f ticks)", degrees, adjustmentTicks);
        telemetry.addData("Current Position", "%d ticks", currentPosition);
        telemetry.addData("Target Position", "%.1f ticks", targetPosition);
        telemetry.addData("Direction", degrees > 0 ? "Clockwise" : "Counter-clockwise");
        telemetry.update();
    }
    
    /**
     * Function Trigger (single shot)
     * 1) Check shooter is running at target velocity
     * 2) Put the servo in fire position for 0.5 sec
     * 3) Return to home position
     * 4) Advance indexer
     */
    private void triggerFunction() {
        if (triggerSequenceActive) {
            telemetry.addData("⚠️ Trigger", "Sequence already active");
            return;
        }
        
        // 1) Check if shooter is running at target velocity
        if (!shooterRunning) {
            telemetry.addData("⚠️ Trigger", "Shooter not running - use gamepad1 B/Y to set speed first");
            telemetry.update();
            return;
        }
        
        // 2) Start trigger sequence - move to fire position
        triggerServo.setPosition(TRIGGER_FIRE);
        
        // Put indexer in float mode while trigger is firing
        indexor.setPower(0);
        indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        triggerSequenceActive = true;
        isThreeShotMode = false;  // Single shot mode
        triggerSequenceStep = 1;  // Step 1: Single fire
        shotsFired = 0;
        triggerTimer.reset();
        
        telemetry.addData("🎯 Trigger Function", "SINGLE SHOT STARTED");
        telemetry.addData("Shooter Status", "RUNNING at %.0f ticks/sec", currentShooterVelocity);
        telemetry.addData("Trigger Position", "FIRE (%.1f seconds)", TRIGGER_FIRE_DURATION);
        telemetry.addData("Indexer Mode", "FLOAT (for firing)");
        telemetry.update();
    }
    
    /**
     * Start three-shot sequence (gamepad2 Y button)
     * Fires 3 shots with indexer advance between each shot
     */
    private void startThreeShotSequence() {
        if (triggerSequenceActive) {
            telemetry.addData("⚠️ Three-Shot", "Sequence already active");
            return;
        }
        
        if (!shooterRunning) {
            telemetry.addData("⚠️ Three-Shot", "Shooter not running - use gamepad1 B/Y to set speed first");
            telemetry.update();
            return;
        }
        
        // Start trigger sequence - move to fire position for first shot
        triggerServo.setPosition(TRIGGER_FIRE);
        
        // Put indexer in float mode while trigger is firing
        indexor.setPower(0);
        indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        triggerSequenceActive = true;
        isThreeShotMode = true;  // Three-shot mode
        triggerSequenceStep = 1;  // Step 1: First fire
        shotsFired = 0;
        triggerTimer.reset();
        
        telemetry.addData("🎯 THREE-SHOT SEQUENCE", "STARTED");
        telemetry.addData("Shooter Status", "RUNNING at %.0f ticks/sec", currentShooterVelocity);
        telemetry.addData("Shots to Fire", "3 shots with auto-advance");
        telemetry.update();
    }
    
    private void handleTriggerSequence() {
        if (!triggerSequenceActive) {
            return;
        }
        
        double elapsedTime = triggerTimer.seconds();
        
        switch (triggerSequenceStep) {
            case 1: // First shot - fire position (shorter duration for first shot)
                if (elapsedTime >= 0.3) {
                    triggerServo.setPosition(TRIGGER_HOME);
                    shotsFired = 1;
                    triggerSequenceStep = 2;
                    triggerTimer.reset();
                }
                break;
                
            case 2: // First shot complete - advance indexer
                if (elapsedTime >= INDEXER_ADVANCE_WAIT) {
                    advanceIndexer();
                    
                    // If single shot mode, end sequence here
                    if (!isThreeShotMode) {
                        triggerSequenceStep = 0;
                        triggerSequenceActive = false;
                        shotsFired = 0;
                        
                        telemetry.addData("✅ SINGLE SHOT", "COMPLETE - 1 shot fired!");
                        telemetry.update();
                    } else {
                        // Continue to shot 2 for three-shot mode
                        triggerSequenceStep = 3;
                        triggerTimer.reset();
                    }
                }
                break;
                
            case 3: // Second shot - wait before firing
                if (elapsedTime >= TRIGGER_FIRE_DURATION) {
                    triggerServo.setPosition(TRIGGER_FIRE);
                    triggerSequenceStep = 4;
                    triggerTimer.reset();
                }
                break;
                
            case 4: // Second shot hold
                if (elapsedTime >= TRIGGER_FIRE_DURATION) {
                    triggerServo.setPosition(TRIGGER_HOME);
                    shotsFired = 2;
                    triggerSequenceStep = 5;
                    triggerTimer.reset();
                }
                break;
                
            case 5: // Second shot complete - advance indexer
                if (elapsedTime >= INDEXER_ADVANCE_WAIT) {
                    advanceIndexer();
                    triggerSequenceStep = 6;
                    triggerTimer.reset();
                }
                break;
                
            case 6: // Third shot - wait before firing
                if (elapsedTime >= TRIGGER_FIRE_DURATION) {
                    triggerServo.setPosition(TRIGGER_FIRE);
                    triggerSequenceStep = 7;
                    triggerTimer.reset();
                }
                break;
                
            case 7: // Third shot hold
                if (elapsedTime >= TRIGGER_FIRE_DURATION) {
                    triggerServo.setPosition(TRIGGER_HOME);
                    shotsFired = 3;
                    triggerSequenceStep = 8;
                    triggerTimer.reset();
                }
                break;
                
            case 8: // Final advance indexer
                if (elapsedTime >= INDEXER_ADVANCE_WAIT) {
                    advanceIndexer();
                    triggerSequenceStep = 0;
                    triggerSequenceActive = false;
                    shotsFired = 0;
                    
                    telemetry.addData("✅ THREE-SHOT SEQUENCE", "COMPLETE - 3 shots fired!");
                    telemetry.update();
                }
                break;
        }
    }
    
    private void updateShooterSpeedMonitoring() {
        if (!shooterRunning) {
            return;
        }
        
        // Continuously reapply shooter velocity to maintain consistent speed
        shooter.setVelocity(currentShooterVelocity);
        
        double currentVelocity = shooter.getVelocity();
        double speedError = Math.abs(currentVelocity - currentShooterVelocity);
        double stabilizationTime = shooterStabilizationTimer.seconds();
        
        // Check if speed is within tolerance
        boolean speedWithinTolerance = speedError <= SHOOTER_SPEED_TOLERANCE;
        
        if (speedWithinTolerance && stabilizationTime >= SHOOTER_STABILIZATION_TIME) {
            shooterSpeedStable = true;
        } else if (!speedWithinTolerance) {
            // Speed drifted - reset stabilization timer
            shooterStabilizationTimer.reset();
            shooterSpeedStable = false;
        }
    }
    
    private void updateSpeedLight() {
        if (!shooterRunning) {
            // Shooter is off - speed light should be off
            speedLight.setPosition(LIGHT_OFF_POSITION);
            return;
        }
        
        // Shooter is on - check speed and stability
        double currentVelocity = shooter.getVelocity();
        double speedPercentage = currentVelocity / currentShooterVelocity;
        
        if (shooterSpeedStable && speedPercentage > SHOOTER_SPEED_THRESHOLD) {
            // Speed is optimal and stable - green light (95%+ and stable)
            speedLight.setPosition(LIGHT_GREEN_POSITION);
        } else if (currentVelocity > 50 && speedPercentage > SHOOTER_MIN_SPEED_THRESHOLD) {
            // Speed is acceptable but may not be stable - white light (85%+ of target)
            speedLight.setPosition(LIGHT_WHITE_POSITION);
        } else {
            // Speed is too low for consistent shooting - off (no light)
            speedLight.setPosition(LIGHT_OFF_POSITION);
        }
    }
    
    private void handleMecanumDrive() {
        // Skip manual drive if alignment is active
        if (alignmentActive) {
            return;
        }
        
        // Standard mecanum drive
        double drive = gamepad1.left_stick_y * DRIVE_SPEED_MULTIPLIER;
        double strafe = gamepad1.left_stick_x * STRAFE_SPEED_MULTIPLIER;
        double turn = -gamepad1.right_stick_x * TURN_SPEED_MULTIPLIER;
        
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;
        
        // Normalize powers
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                                  Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        
        leftFront.setPower(backLeftPower);
        rightFront.setPower(backRightPower);
        leftBack.setPower(frontLeftPower);
        rightBack.setPower(frontRightPower);
    }
    
    private void updateTelemetry() {
        telemetry.addData("Status", "TeleOpDECODESimple2 - RUNNING");
        telemetry.addData("", "");
        
        // Ball detection status
        String ballStatusIntake = ballDetectedIntake ? "🔴 DETECTED" : "⚪ CLEAR";
        String ballStatusExit = ballDetectedExit ? "🔴 DETECTED" : "⚪ CLEAR";
        telemetry.addData("Intake Sensor", "%s", ballStatusIntake);
        telemetry.addData("Exit Sensor", "%s", ballStatusExit);
        
        // Check for automatic indexer advancement based on ball detection
        checkAutomaticIndexerAdvance();
        
        // Motor status
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Conveyor Power", "%.2f", conveyor.getPower());
        telemetry.addData("Indexor Power", "%.2f", indexor.getPower());
        
        // Indexer position
        int currentPosition = indexor.getCurrentPosition();
        double logicalPosition = (indexorLastSuccessfulPosition % INDEXOR_TICKS_PER_REVOLUTION) / INDEXOR_TICKS_PER_120_DEGREES;
        telemetry.addData("Indexor Position", "Current: %d, Last Successful: %.1f", 
            currentPosition, indexorLastSuccessfulPosition);
        telemetry.addData("Logical Position", "%.1f (%.0f degrees)", logicalPosition, logicalPosition * 120);
        
        // Shooter status
        if (shooterRunning) {
            double currentVelocity = shooter.getVelocity();
            double speedPercentage = (currentVelocity / currentShooterVelocity) * 100;
            String stabilityStatus = shooterSpeedStable ? "STABLE" : "STABILIZING";
            String lightStatus;
            
            if (shooterSpeedStable && speedPercentage >= SHOOTER_SPEED_THRESHOLD * 100) {
                lightStatus = "🟢 GREEN (READY TO FIRE)";
            } else if (speedPercentage >= SHOOTER_MIN_SPEED_THRESHOLD * 100) {
                lightStatus = "⚪ WHITE (SPINNING UP)";
            } else {
                lightStatus = "⚫ OFF (TOO SLOW)";
            }
            
            telemetry.addData("Shooter", "RUNNING at %.0f ticks/sec (%.0f%% - %s)", 
                currentShooterVelocity, speedPercentage, stabilityStatus);
            telemetry.addData("Current Velocity", "%.0f ticks/sec", currentVelocity);
            telemetry.addData("Speed Light", "%s", lightStatus);
        } else {
            telemetry.addData("Shooter", "STOPPED");
            telemetry.addData("Speed Light", "⚫ OFF");
        }
        
        // Trigger status
        if (triggerSequenceActive) {
            String stepDescription;
            
            if (triggerSequenceStep == 1) {
                stepDescription = String.format("Shot 1: FIRING");
            } else if (triggerSequenceStep == 2) {
                stepDescription = String.format("Shot 1: Advancing indexer...");
            } else if (triggerSequenceStep == 3) {
                stepDescription = String.format("Shot 2: Preparing...");
            } else if (triggerSequenceStep == 4) {
                stepDescription = String.format("Shot 2: FIRING");
            } else if (triggerSequenceStep == 5) {
                stepDescription = String.format("Shot 2: Advancing indexer...");
            } else if (triggerSequenceStep == 6) {
                stepDescription = String.format("Shot 3: Preparing...");
            } else if (triggerSequenceStep == 7) {
                stepDescription = String.format("Shot 3: FIRING");
            } else if (triggerSequenceStep == 8) {
                stepDescription = String.format("Shot 3: Final advance...");
            } else {
                stepDescription = "Unknown step";
            }
            
            telemetry.addData("Trigger Sequence", "%s", stepDescription);
            telemetry.addData("Shots Fired", "%d / 3", shotsFired);
        } else {
            double currentTriggerPos = triggerServo.getPosition();
            String positionName;
            if (Math.abs(currentTriggerPos - TRIGGER_FIRE) < 0.05) {
                positionName = "FIRE";
            } else {
                positionName = "HOME";
            }
            telemetry.addData("Trigger Position", "%s (%.2f)", positionName, currentTriggerPos);
        }
        
        telemetry.addData("", "");
        telemetry.addData("Drive Controls", "Left stick: Drive/Strafe, Right stick X: Turn");
        
        telemetry.update();
    }
    
    /**
     * Check for automatic indexer advancement when ball is detected during intake
     * Part of Intake Function requirement: "3) Advance the indexer forward when ball detected in the intake sensor"
     */
    private void checkAutomaticIndexerAdvance() {
        // Check for rising edge of ball detection during intake
        if (ballDetectedIntake && !previousBallDetectedIntake) {
            // Ball detected - check if intake is running
            boolean intakeRunning = Math.abs(intake.getPower()) > 0.1 && intake.getPower() > 0;
            
            if (intakeRunning && !indexorMoving) {
                // Auto-advance indexer when ball detected during intake
                advanceIndexer();
                telemetry.addData("🎾 Auto Advance", "Ball detected - advancing indexer");
            }
        }
    }
    
    /**
     * Start AprilTag alignment sequence
     * Finds AprilTag #20 and aligns robot to face it
     */
    private void startAprilTagAlignment() {
        if (alignmentActive) {
            telemetry.addData("⚠️ Alignment", "Already in progress");
            telemetry.update();
            return;
        }
        
        // Check if AprilTag system is available
        if (visionPortal == null || aprilTag == null) {
            telemetry.addData("❌ Alignment", "AprilTag system not available");
            telemetry.update();
            return;
        }
        
        // Check if camera is streaming
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("❌ Alignment", "Camera not streaming");
            telemetry.update();
            return;
        }
        
        // Look for AprilTag #20 or #24
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection targetTag = null;
        
        telemetry.addData("🔍 Searching", "Found %d tags", detections.size());
        
        for (AprilTagDetection detection : detections) {
            telemetry.addData("Tag Found", "ID: %d", detection.id);
            if ((detection.id == TARGET_TAG_ID_PRIMARY || detection.id == TARGET_TAG_ID_SECONDARY) 
                && detection.ftcPose != null) {
                targetTag = detection;
                break;
            }
        }
        
        if (targetTag == null) {
            telemetry.addData("❌ Alignment", "AprilTag #%d or #%d not found", TARGET_TAG_ID_PRIMARY, TARGET_TAG_ID_SECONDARY);
            telemetry.addData("Camera State", "%s", visionPortal.getCameraState().toString());
            telemetry.update();
            return;
        }
        
        // Start alignment
        targetTagId = targetTag.id;
        targetBearing = targetTag.ftcPose.bearing;
        alignmentActive = true;
        alignmentTimer.reset();
        
        double bearingDegrees = Math.toDegrees(targetBearing);
        telemetry.addData("🎯 Alignment STARTED", "Tag #%d - Target bearing: %.1f°", targetTag.id, bearingDegrees);
        telemetry.addData("📏 Distance", "%.1f inches", targetTag.ftcPose.range);
        telemetry.update();
    }
    
    /**
     * Handle AprilTag alignment process
     * Continuously adjusts robot rotation until aligned with target
     */
    private void handleAprilTagAlignment() {
        if (!alignmentActive) {
            return;
        }
        
        // Check timeout
        if (alignmentTimer.seconds() > ALIGNMENT_TIMEOUT) {
            stopAlignment("Timeout reached");
            return;
        }
        
        // Check for manual override (any gamepad input except gamepad2.a which starts alignment)
        if (Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 || 
            Math.abs(gamepad1.right_stick_x) > 0.1) {
            stopAlignment("Manual override");
            return;
        }
        
        // Get current AprilTag detection
        if (visionPortal == null || aprilTag == null) {
            stopAlignment("Vision system lost");
            return;
        }
        
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection targetTag = null;
        
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId && detection.ftcPose != null) {
                targetTag = detection;
                break;
            }
        }
        
        if (targetTag == null) {
            stopAlignment("AprilTag lost");
            return;
        }
        
        // Calculate bearing error (convert from radians to degrees)
        double currentBearing = Math.toDegrees(targetTag.ftcPose.bearing);
        double bearingError = currentBearing;
        
        // Check if aligned (within tolerance)
        if (Math.abs(bearingError) < ALIGNMENT_TOLERANCE) {
            stopAlignment("Aligned successfully!");
            return;
        }
        
        // Calculate turn power (proportional control)
        // P = 0.015 gives full power (0.3) at maximum expected bearing error of 20 degrees
        double turnPower = bearingError * 0.015; // Proportional gain: 20° × 0.015 = 0.3 (full power)
        turnPower = Math.max(-ALIGNMENT_TURN_POWER, Math.min(ALIGNMENT_TURN_POWER, turnPower));
        
        // Apply turn movement (positive bearing = turn right)
        double frontLeftPower = turnPower;
        double frontRightPower = -turnPower;
        double backLeftPower = turnPower;
        double backRightPower = -turnPower;
        
        leftFront.setPower(backLeftPower);
        rightFront.setPower(backRightPower);
        leftBack.setPower(frontLeftPower);
        rightBack.setPower(frontRightPower);
        
        telemetry.addData("🎯 ALIGNING", "Bearing error: %.1f°", bearingError);
        telemetry.addData("🔄 Turn Power", "%.2f", turnPower);
        telemetry.addData("📏 Distance", "%.1f inches", targetTag.ftcPose.range);
        telemetry.addData("⏱️ Time", "%.1f / %.1f seconds", alignmentTimer.seconds(), ALIGNMENT_TIMEOUT);
        telemetry.update();
    }
    
    /**
     * Stop alignment sequence and return to manual control
     */
    private void stopAlignment(String reason) {
        alignmentActive = false;
        
        // Stop all drive motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        
        telemetry.addData("✅ Alignment STOPPED", "%s", reason);
        telemetry.addData("⏱️ Total Time", "%.1f seconds", alignmentTimer.seconds());
        telemetry.update();
    }
}