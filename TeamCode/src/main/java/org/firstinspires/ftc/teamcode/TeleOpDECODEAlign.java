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
@TeleOp(name = "TeleOpDECODEAlign", group = "TeleOp")
public class TeleOpDECODEAlign extends LinearOpMode {
    
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
    private ElapsedTime boostTimer = new ElapsedTime(); // Timer for voltage boost
    private boolean boostActive = false; // Track if boost is active
    
    // Trigger sequence variables
    private boolean triggerSequenceActive = false;
    private boolean isThreeShotMode = false;  // Track if doing 3-shot sequence or single shot
    private ElapsedTime triggerTimer = new ElapsedTime();
    private int triggerSequenceStep = 0;  // 0=home, 1-8 for 3-shot sequence
    private int shotsFired = 0;
    private static final double INDEXER_ADVANCE_WAIT = 0.2;  // Wait time for indexer to advance

    // Shooter velocity settings
    public static final double SHOOTER_TARGET_VELOCITY_1300 = 1330;  // B button velocity
    public static final double SHOOTER_TARGET_VELOCITY_1600 = 1530;  // Y button velocity
    
    // Shooter PID coefficients for velocity control
    public static double VELOCITY_P = 6.0;  // Proportional coefficient (increased for faster response on restart)
    public static double VELOCITY_I = 0.15;  // Integral coefficient
    public static double VELOCITY_D = 0.5;  // Derivative coefficient
    public static double VELOCITY_F = 13.0;  // Feedforward coefficient

    // Speed monitoring thresholds
    public static final double SHOOTER_SPEED_THRESHOLD = 0.90; // 90% of target speed for green light
    public static final double SHOOTER_MIN_SPEED_THRESHOLD = 0.85; // 85% minimum for white light
    public static final double SHOOTER_SPEED_TOLERANCE = 50;       // ticks/sec tolerance for "stable" speed
    public static final double SHOOTER_STABILIZATION_TIME = 0.3;   // Seconds to wait for speed stabilization
    
    // Voltage boost settings
    public static final double BOOST_VOLTAGE_MULTIPLIER = 1.45;    // 35% voltage boost for startup
    public static final double BOOST_DURATION = 0.5;                // 400 milliseconds boost duration
    
    // Motor power settings
    public static final double INTAKE_POWER = 1.0;
    public static final double CONVEYOR_POWER = 1.0;
    public static final double INDEXOR_POWER = 0.7;
    public static final double SHOOTER_SERVO_POWER = 1.0;
    
    // Indexor position settings
    public static final double INDEXOR_TICKS_PER_REVOLUTION = 537.7;  // goBILDA 312 RPM motor
    public static final double INDEXOR_TICKS_PER_120_DEGREES = INDEXOR_TICKS_PER_REVOLUTION / 3.0;  // 179.23 ticks per 120°
    public static final double INDEXOR_TICKS_PER_DEGREE = INDEXOR_TICKS_PER_REVOLUTION / 360.0;  // ~1.49 ticks per degree
    public static final double INDEXOR_TICKS_PER_10_DEGREES = INDEXOR_TICKS_PER_DEGREE * 10.0;  // ~14.94 ticks per 10°
    
    // Color sensor settings
    public static final double COLOR_SENSOR_GAIN = 15.0;
    public static final double BALL_DETECTION_THRESHOLD = 0.15;
    
    // Trigger servo positions
    public static final double TRIGGER_FIRE = 0.0;     // Fire position (27.0 degrees)
    public static final double TRIGGER_HOME = 0.5;     // Home position (104.4 degrees)
    public static final double TRIGGER_FIRE_DURATION = 0.5;  // Fire duration in seconds
    
    // Speed light control settings (using servo positions for LED control)
    public static final double LIGHT_OFF_POSITION = 0.0;      // Servo position for light off
    public static final double LIGHT_WHITE_POSITION = 0.33;   // Servo position for white light (low speed)
    public static final double LIGHT_GREEN_POSITION = 0.5;    // Servo position for green light (in tolerance)
    public static final double LIGHT_BLUE_POSITION = 0.95;    // Servo position for blue light (high speed) - adjusted for intensity
    
    // Indexor stuck detection
    public static final double INDEXOR_STUCK_TIMEOUT = 0.5;  // 0.5 seconds as specified
    public static final int INDEXOR_STUCK_THRESHOLD = 10;    // Minimum movement required
    
    // AprilTag settings
    public static final int TARGET_TAG_ID_PRIMARY = 20;
    public static final int TARGET_TAG_ID_SECONDARY = 24;
    public static final double OPTIMAL_SHOOTING_DISTANCE = 24.0; // inches
    
    // Mecanum drive settings
    public static final double DRIVE_SPEED_MULTIPLIER = 0.7;
    public static final double STRAFE_SPEED_MULTIPLIER = 0.7;
    public static final double TURN_SPEED_MULTIPLIER = 0.5;
    
    // AprilTag alignment settings
    public static final double ALIGNMENT_TURN_VELOCITY = 300.0; // Velocity (ticks/sec) for alignment turns
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
        
        telemetry.addData("Status", "TeleOpDECODEAlign - Initialized");
        telemetry.addData("", "");
        telemetry.addData("GAMEPAD1 CONTROLS:", "");
        telemetry.addData("A", "Intake Function");
        telemetry.addData("B", "Toggle Shooter Speed 1300");
        telemetry.addData("Y", "Toggle Shooter Speed 1600");
        telemetry.addData("Left Stick", "Drive/Strafe");
        telemetry.addData("Right Stick X", "Turn");
        telemetry.addData("", "");
        telemetry.addData("GAMEPAD2 CONTROLS:", "");
        telemetry.addData("A", "🎯 Align with AprilTag");
        telemetry.addData("X", "Advance Indexer");
        telemetry.addData("B", "Trigger Function (Single Shot)");
        telemetry.addData("Y", "Three-Shot Sequence");
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
        
        // Set motor directions
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
        
        // Configure drive motors for velocity control during alignment
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
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
     */
    private void intakeFunction() {
        boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
        
        if (!intakeRunning) {
            // Start intake
            intake.setPower(INTAKE_POWER);
            conveyor.setPower(CONVEYOR_POWER);
            
            // Track that intake was started by gamepad1
            intakeFromGamepad1 = true;
            intakeFromGamepad2 = false;
            
            telemetry.addData("✅ Intake Function", "STARTED");
        } else {
            // Stop intake
            intake.setPower(0);
            if (!indexorMoving) {
                conveyor.setPower(0);
            }
            
            // Clear intake source flags
            intakeFromGamepad1 = false;
            intakeFromGamepad2 = false;
            
            telemetry.addData("⏹️ Intake Function", "STOPPED");
        }
        
        // Auto-advance indexer when ball detected during intake
        if (ballDetectedIntake && !previousBallDetectedIntake) {
            boolean currentIntakeRunning = Math.abs(intake.getPower()) > 0.1 && intake.getPower() > 0;
            
            if (currentIntakeRunning && !indexorMoving) {
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
        intake.setPower(-INTAKE_POWER);
        conveyor.setPower(-CONVEYOR_POWER);
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexor.setPower(0);
    }
    
    private void stopOuttake() {
        intake.setPower(0);
        conveyor.setPower(0);
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        indexor.setPower(0);
        
        // Clear intake source flags when stopping outtake
        intakeFromGamepad1 = false;
        intakeFromGamepad2 = false;
    }
    
    private void stopIntakeFunction() {
        intake.setPower(0);
        if (!indexorMoving) {
            conveyor.setPower(0);
        }
        
        // Clear intake source flags
        intakeFromGamepad1 = false;
        intakeFromGamepad2 = false;
    }
    
    /**
     * Intake from gamepad2 joystick - similar to intakeFunction but tracks joystick source
     */
    private void intakeFromJoystick() {
        boolean intakeRunning = Math.abs(intake.getPower()) > 0.1;
        
        if (!intakeRunning) {
            // Start intake
            intake.setPower(INTAKE_POWER);
            conveyor.setPower(CONVEYOR_POWER);
            
            // Track that intake was started by gamepad2 joystick
            intakeFromGamepad2 = true;
            intakeFromGamepad1 = false;
        }
        
        // Auto-advance indexer when ball detected during intake
        if (ballDetectedIntake && !previousBallDetectedIntake) {
            boolean currentIntakeRunning = Math.abs(intake.getPower()) > 0.1 && intake.getPower() > 0;
            
            if (currentIntakeRunning && !indexorMoving) {
                advanceIndexer();
                telemetry.addData("🎾 Auto Advance", "Ball detected - advancing indexer");
            }
        }
    }
    
    /**
     * Toggle Shooter Speed (gamepad1 B/Y buttons)
     * B button: Toggle 1300 ticks/sec
     * Y button: Toggle 1600 ticks/sec
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
        } else {
            // Shooter is off or running at different speed - start at specified velocity with boost
            shooter.setVelocity(velocity * BOOST_VOLTAGE_MULTIPLIER);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            shooterRunning = true;
            currentShooterVelocity = velocity;
            shooterStabilizationTimer.reset();
            shooterSpeedStable = false;
            boostTimer.reset();
            boostActive = true;
            
            telemetry.addData("🎯 Shooter STARTED", "%.0f ticks/sec (BOOST: %.0f)", velocity, velocity * BOOST_VOLTAGE_MULTIPLIER);
        }
        telemetry.update();
    }
    
    /**
     * Function Advance Indexer
     * Advance indexer to next 120° position
     */
    private void advanceIndexer() {
        if (indexorMoving) {
            telemetry.addData("⚠️ Indexer", "Already moving - please wait");
            return;
        }
        
        // Next position is always increment of 120 degrees from previous successful position
        double nextPosition = indexorLastSuccessfulPosition + INDEXOR_TICKS_PER_120_DEGREES;
        
        // Set target position
        indexor.setTargetPosition((int) Math.round(nextPosition));
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(INDEXOR_POWER);
        
        // Run conveyor when indexer is running
        conveyor.setPower(CONVEYOR_POWER);
        
        // Start movement tracking
        indexorMoving = true;
        indexorTimer.reset();
        indexorStartPosition = indexor.getCurrentPosition();
        
        telemetry.addData("🎯 Advance Indexer", "Moving to %.1f ticks", nextPosition);
        telemetry.update();
    }
    
    /**
     * Handle indexer stuck detection
     */
    private void handleIndexorStuckDetection() {
        if (!indexorMoving) {
            return;
        }
        
        // Check if indexer reached target
        if (!indexor.isBusy()) {
            int currentPosition = indexor.getCurrentPosition();
            int targetPosition = indexor.getTargetPosition();
            int positionError = Math.abs(currentPosition - targetPosition);
            
            if (positionError <= 15) {
                // Movement completed successfully - update successful position
                indexorLastSuccessfulPosition = targetPosition;
                telemetry.addData("✅ Indexer", "Successfully advanced to %.1f", indexorLastSuccessfulPosition);
            } else {
                // Movement completed but not at target
                telemetry.addData("⚠️ Indexer", "Not at target (error: %d ticks)", positionError);
            }
            
            indexorMoving = false;
            indexor.setPower(0);
            // Only stop conveyor if intake is not running
            if (Math.abs(intake.getPower()) <= 0.1) {
                conveyor.setPower(0);
            }
            return;
        }
        
        // Check for stuck condition
        if (indexorTimer.seconds() > INDEXOR_STUCK_TIMEOUT) {
            int currentPosition = indexor.getCurrentPosition();
            int movement = Math.abs(currentPosition - indexorStartPosition);
            
            if (movement < INDEXOR_STUCK_THRESHOLD) {
                // Indexer is stuck - put in float mode
                indexor.setPower(0);
                indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                
                indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                indexor.setPower(0);
                
                if (Math.abs(intake.getPower()) <= 0.1) {
                    conveyor.setPower(0);
                }
                
                indexorMoving = false;
                
                telemetry.addData("⚠️ Indexer STUCK", "Put in FLOAT mode");
                telemetry.update();
            }
        }
    }
    
    /**
     * Adjust indexer rotation by specified degrees
     */
    private void adjustIndexerRotation(double degrees) {
        if (indexorMoving) {
            telemetry.addData("⚠️ Indexer", "Already moving - cannot adjust");
            return;
        }
        
        double adjustmentTicks = degrees * INDEXOR_TICKS_PER_DEGREE;
        int currentPosition = indexor.getCurrentPosition();
        double targetPosition = currentPosition + adjustmentTicks;
        
        indexor.setTargetPosition((int) Math.round(targetPosition));
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(INDEXOR_POWER * 0.7);
        
        indexorTimer.reset();
        indexorMoving = true;
        indexorStartPosition = currentPosition;
        
        telemetry.addData("🎯 Fine Adjust", "%.1f° (%.1f ticks)", degrees, adjustmentTicks);
        telemetry.update();
    }
    
    /**
     * Function Trigger (single shot)
     */
    private void triggerFunction() {
        if (triggerSequenceActive) {
            telemetry.addData("⚠️ Trigger", "Sequence already active");
            return;
        }
        
        // Start trigger sequence - move to fire position
        triggerServo.setPosition(TRIGGER_FIRE);
        
        // Put indexer in float mode while trigger is firing
        indexor.setPower(0);
        indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        triggerSequenceActive = true;
        isThreeShotMode = false;
        triggerSequenceStep = 1;
        shotsFired = 0;
        triggerTimer.reset();
        
        telemetry.addData("🎯 Trigger Function", "SINGLE SHOT STARTED");
        telemetry.update();
    }
    
    /**
     * Start three-shot sequence
     */
    private void startThreeShotSequence() {
        if (triggerSequenceActive) {
            telemetry.addData("⚠️ Three-Shot", "Sequence already active");
            return;
        }
        
        if (!shooterRunning) {
            telemetry.addData("⚠️ Three-Shot", "Shooter not running");
            telemetry.update();
            return;
        }
        
        // Start trigger sequence
        triggerServo.setPosition(TRIGGER_FIRE);
        
        // Put indexer in float mode while trigger is firing
        indexor.setPower(0);
        indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        triggerSequenceActive = true;
        isThreeShotMode = true;
        triggerSequenceStep = 1;
        shotsFired = 0;
        triggerTimer.reset();
        
        telemetry.addData("🎯 THREE-SHOT SEQUENCE", "STARTED");
        telemetry.update();
    }
    
    private void handleTriggerSequence() {
        if (!triggerSequenceActive) {
            return;
        }
        
        double elapsedTime = triggerTimer.seconds();
        
        switch (triggerSequenceStep) {
            case 1: // First shot - fire position
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
                    
                    if (!isThreeShotMode) {
                        triggerSequenceStep = 0;
                        triggerSequenceActive = false;
                        shotsFired = 0;
                    } else {
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
                }
                break;
        }
    }
    
    private void updateShooterSpeedMonitoring() {
        if (!shooterRunning) {
            return;
        }
        
        // Handle voltage boost transition
        if (boostActive && boostTimer.seconds() >= BOOST_DURATION) {
            boostActive = false;
            shooter.setVelocity(currentShooterVelocity);
        }
        
        double currentVelocity = shooter.getVelocity();
        double speedError = Math.abs(currentVelocity - currentShooterVelocity);
        double stabilizationTime = shooterStabilizationTimer.seconds();
        
        boolean speedWithinTolerance = speedError <= SHOOTER_SPEED_TOLERANCE;
        
        if (speedWithinTolerance && stabilizationTime >= SHOOTER_STABILIZATION_TIME) {
            shooterSpeedStable = true;
        } else if (!speedWithinTolerance) {
            shooterSpeedStable = false;
            shooterStabilizationTimer.reset();
        }
    }
    
    private void updateSpeedLight() {
        if (!shooterRunning) {
            speedLight.setPosition(LIGHT_OFF_POSITION);
            return;
        }
        
        double currentVelocity = shooter.getVelocity();
        double speedPercentage = currentVelocity / currentShooterVelocity;
        
        // Green: Speed is stable and within tolerance range
        if (shooterSpeedStable && speedPercentage > SHOOTER_SPEED_THRESHOLD) {
            speedLight.setPosition(LIGHT_GREEN_POSITION);
        }
        // Blue: High speed mode (1600 target velocity)
        else if (currentShooterVelocity >= SHOOTER_TARGET_VELOCITY_1600) {
            speedLight.setPosition(LIGHT_BLUE_POSITION);
        }
        // White: Low speed mode (1300 target velocity)
        else if (currentShooterVelocity >= SHOOTER_TARGET_VELOCITY_1300) {
            speedLight.setPosition(LIGHT_WHITE_POSITION);
        }
        // Off: Speed too low
        else {
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
        double strafe = -gamepad1.left_stick_x * STRAFE_SPEED_MULTIPLIER;  // Inverted for correct left/right
        double turn = -gamepad1.right_stick_x * TURN_SPEED_MULTIPLIER;
        
        // Check if all joysticks are centered
        boolean joysticksIdle = Math.abs(gamepad1.left_stick_y) < 0.05 && 
                                Math.abs(gamepad1.left_stick_x) < 0.05 && 
                                Math.abs(gamepad1.right_stick_x) < 0.05;
        
        if (joysticksIdle) {
            // Stop all motors
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        } else {
            // Calculate motor powers
            double leftFrontPower = drive + strafe + turn;
            double rightFrontPower = drive - strafe - turn;
            double leftBackPower = drive - strafe + turn;
            double rightBackPower = drive + strafe - turn;
            
            // Normalize powers
            double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                                      Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightBackPower /= maxPower;
            }
            
            // Set motor powers
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
        }
    }
    
    private void updateTelemetry() {
        telemetry.addData("Status", "TeleOpDECODEAlign");
        telemetry.addData("", "");
        
        // Shooter status
        if (shooterRunning) {
            double currentVelocity = shooter.getVelocity();
            double speedPercentage = (currentVelocity / currentShooterVelocity) * 100;
            telemetry.addData("🎯 Shooter", "%.0f / %.0f ticks/sec (%.0f%%)", 
                            currentVelocity, currentShooterVelocity, speedPercentage);
            if (boostActive) {
                telemetry.addData("⚡ Boost", "ACTIVE (%.1f sec remaining)", BOOST_DURATION - boostTimer.seconds());
            }
            if (shooterSpeedStable) {
                telemetry.addData("✅ Speed", "STABLE");
            }
        } else {
            telemetry.addData("🎯 Shooter", "OFF");
        }
        
        // Indexer status
        telemetry.addData("📍 Indexer Position", "%.1f ticks", (double)indexor.getCurrentPosition());
        telemetry.addData("✅ Last Successful", "%.1f ticks", indexorLastSuccessfulPosition);
        if (indexorMoving) {
            telemetry.addData("⚙️ Indexer", "MOVING to %d", indexor.getTargetPosition());
        }
        
        // Ball detection
        telemetry.addData("🎾 Ball Detected", ballDetectedIntake ? "YES (Intake)" : "No");
        if (ballDetectedExit) {
            telemetry.addData("🎾 Exit Sensor", "Ball Present");
        }
        
        // Alignment status
        if (alignmentActive) {
            telemetry.addData("🎯 ALIGNMENT", "ACTIVE");
        }
        
        telemetry.update();
    }
    
    /**
     * Start AprilTag alignment process
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
        
        // Check for manual override
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
        
        // Calculate turn velocity (proportional control)
        double velocityScale = Math.abs(bearingError) / 20.0; // Scale from 0 to 1.0 based on 20° max expected error
        velocityScale = Math.min(velocityScale, 1.0); // Cap at 1.0
        velocityScale = Math.max(velocityScale, 0.2); // Minimum 20% velocity for small errors
        
        double turnVelocity = ALIGNMENT_TURN_VELOCITY * velocityScale;
        
        // Apply direction based on bearing error
        if (bearingError < 0) {
            turnVelocity = -turnVelocity;
        }
        
        // Apply turn movement using velocity control
        leftFront.setVelocity(turnVelocity);
        rightFront.setVelocity(-turnVelocity);
        leftBack.setVelocity(turnVelocity);
        rightBack.setVelocity(-turnVelocity);
        
        telemetry.addData("🎯 ALIGNING", "Bearing error: %.1f°", bearingError);
        telemetry.addData("🔄 Turn Velocity", "%.0f ticks/sec (%.0f%% of max)", turnVelocity, velocityScale * 100);
        telemetry.addData("📏 Distance", "%.1f inches", targetTag.ftcPose.range);
        telemetry.addData("⏱️ Time", "%.1f / %.1f seconds", alignmentTimer.seconds(), ALIGNMENT_TIMEOUT);
        telemetry.update();
    }
    
    /**
     * Stop alignment sequence and return to manual control
     */
    private void stopAlignment(String reason) {
        alignmentActive = false;
        
        // Stop all drive motors using velocity control
        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
        leftBack.setVelocity(0);
        rightBack.setVelocity(0);
        
        telemetry.addData("✅ Alignment STOPPED", "%s", reason);
        telemetry.addData("⏱️ Total Time", "%.1f seconds", alignmentTimer.seconds());
        telemetry.update();
    }
}
