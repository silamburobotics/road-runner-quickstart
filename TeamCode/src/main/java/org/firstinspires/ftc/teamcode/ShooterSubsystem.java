package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shooter Subsystem - Manages trigger, shooter, conveyor, and indexer
 * This class can be used in both Autonomous and TeleOp programs
 */
@Config
public class ShooterSubsystem {
    
    // Hardware components
    private DcMotorEx indexor;
    private DcMotorEx intake;
    private DcMotorEx conveyor;
    private DcMotorEx shooter;
    private CRServo shooterServo;
    private Servo speedLight;
    private Servo triggerServo;
    
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
    public static final double SHOOTER_TARGET_VELOCITY_1300 = 1300;  // B button velocity
    public static final double SHOOTER_TARGET_VELOCITY_1600 = 1600;  // Y button velocity
    
    // Trigger servo positions
    public static final double TRIGGER_FIRE = 0.0;     // Fire position (27.0 degrees)
    public static final double TRIGGER_INTERMITTENT = 0.25;  // Intermittent position (65.7 degrees)
    public static final double TRIGGER_HOME = 0.5;     // Home position (104.4 degrees)
    public static final double TRIGGER_FIRE_DURATION = 0.5;  // Fire duration in seconds
    
    // Speed light control settings
    public static final double LIGHT_OFF_POSITION = 0.0;      // Servo position for light off
    public static final double LIGHT_GREEN_POSITION = 0.5;    // Servo position for green light
    public static final double LIGHT_WHITE_POSITION = 1.0;    // Servo position for white light
    
    // Speed monitoring thresholds
    public static final double SHOOTER_SPEED_THRESHOLD = 0.95; // 95% of target speed for green light
    public static final double SHOOTER_MIN_SPEED_THRESHOLD = 0.85; // 85% minimum for white light
    public static final double SHOOTER_SPEED_TOLERANCE = 50;       // ticks/sec tolerance for "stable" speed
    public static final double SHOOTER_STABILIZATION_TIME = 1.0;   // Seconds to wait for speed stabilization
    
    // Indexor stuck detection
    public static final double INDEXOR_STUCK_TIMEOUT = 0.5;  // 0.5 seconds as specified
    public static final int INDEXOR_STUCK_THRESHOLD = 10;    // Minimum movement required
    
    // State variables
    private double indexorLastSuccessfulPosition = 0.0;  // Last successful indexor position
    private boolean indexorMoving = false;
    private ElapsedTime indexorTimer = new ElapsedTime();
    private int indexorStartPosition = 0;
    
    private boolean shooterRunning = false;
    private double currentShooterVelocity = 1300;  // Default velocity
    private ElapsedTime shooterStabilizationTimer = new ElapsedTime();
    private boolean shooterSpeedStable = false;
    
    private boolean triggerSequenceActive = false;
    private ElapsedTime triggerTimer = new ElapsedTime();
    private int triggerSequenceStep = 0;  // 0=home, 1=first fire, 2=intermittent, 3=second fire, 4=complete
    private boolean firstFireComplete = false;
    
    /**
     * Initialize the shooter subsystem
     * @param hardwareMap The hardware map from the OpMode
     */
    public void init(HardwareMap hardwareMap) {
        // Initialize motors
        indexor = hardwareMap.get(DcMotorEx.class, "indexor");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        
        // Initialize servos
        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
        speedLight = hardwareMap.get(Servo.class, "speedLight");
        triggerServo = hardwareMap.get(Servo.class, "triggerServo");
        
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
        
        // Set zero power behavior
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        // Set motor modes
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Preserve indexor position from previous run
        indexorLastSuccessfulPosition = indexor.getCurrentPosition();
    }
    
    /**
     * Toggle shooter on/off at specified velocity
     * @param velocity Target velocity in ticks/sec
     */
    public void toggleShooterSpeed(double velocity) {
        if (shooterRunning && Math.abs(currentShooterVelocity - velocity) < 50) {
            // Shooter is already running at this speed - stop it
            shooter.setVelocity(0);
            shooterServo.setPower(0);
            shooterRunning = false;
            currentShooterVelocity = 0;
            shooterSpeedStable = false;
        } else {
            // Shooter is off or running at different speed - start at specified velocity
            shooter.setVelocity(velocity);
            shooterServo.setPower(SHOOTER_SERVO_POWER);
            shooterRunning = true;
            currentShooterVelocity = velocity;
            shooterStabilizationTimer.reset();
            shooterSpeedStable = false;
        }
    }
    
    /**
     * Start shooter at specified velocity
     * @param velocity Target velocity in ticks/sec
     */
    public void startShooter(double velocity) {
        shooter.setVelocity(velocity);
        shooterServo.setPower(SHOOTER_SERVO_POWER);
        shooterRunning = true;
        currentShooterVelocity = velocity;
        shooterStabilizationTimer.reset();
        shooterSpeedStable = false;
    }
    
    /**
     * Stop the shooter
     */
    public void stopShooter() {
        shooter.setVelocity(0);
        shooterServo.setPower(0);
        shooterRunning = false;
        currentShooterVelocity = 0;
        shooterSpeedStable = false;
    }
    
    /**
     * Advance indexer to next 120° position
     */
    public void advanceIndexer() {
        if (indexorMoving) {
            return; // Already moving
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
    }
    
    /**
     * Adjust indexer rotation by specified degrees
     * @param degrees Positive for clockwise, negative for counter-clockwise
     */
    public void adjustIndexerRotation(double degrees) {
        if (indexorMoving) {
            return; // Already moving
        }
        
        // Calculate ticks for the adjustment
        double adjustmentTicks = degrees * INDEXOR_TICKS_PER_DEGREE;
        
        // Get current position
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
    }
    
    /**
     * Start trigger sequence (double-fire with intermittent position)
     */
    public void startTriggerSequence() {
        if (triggerSequenceActive) {
            return; // Sequence already active
        }
        
        if (!shooterRunning) {
            return; // Shooter must be running
        }
        
        // Start trigger sequence - move to fire position
        triggerServo.setPosition(TRIGGER_FIRE);
        
        // Put indexer in float mode while trigger is firing
        indexor.setPower(0);
        indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        triggerSequenceActive = true;
        triggerSequenceStep = 1;  // Step 1: First fire
        firstFireComplete = false;
        triggerTimer.reset();
    }
    
    /**
     * Update all subsystem states (call this in loop)
     */
    public void update() {
        handleIndexorStuckDetection();
        handleTriggerSequence();
        updateShooterSpeedMonitoring();
        updateSpeedLight();
    }
    
    /**
     * Handle indexer stuck detection and position tracking
     */
    private void handleIndexorStuckDetection() {
        if (!indexorMoving) {
            return;
        }
        
        // Check if indexer reached target
        if (!indexor.isBusy()) {
            // Check if indexer actually reached the target position
            int currentPosition = indexor.getCurrentPosition();
            int targetPosition = indexor.getTargetPosition();
            int positionError = Math.abs(currentPosition - targetPosition);
            
            if (positionError <= 15) {  // Within 15 ticks tolerance
                // Movement completed successfully - update successful position
                indexorLastSuccessfulPosition = targetPosition;
            }
            
            indexorMoving = false;
            indexor.setPower(0);
            conveyor.setPower(0);
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
                    Thread.sleep(50);  // 50ms delay for mode change
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                
                indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                indexor.setPower(0);
                conveyor.setPower(0);
                indexorMoving = false;
            }
        }
    }
    
    /**
     * Handle trigger sequence state machine
     */
    private void handleTriggerSequence() {
        if (!triggerSequenceActive) {
            return;
        }
        
        double elapsedTime = triggerTimer.seconds();
        
        switch (triggerSequenceStep) {
            case 1: // First fire position
                if (elapsedTime >= TRIGGER_FIRE_DURATION) {
                    // Move to intermittent position
                    triggerServo.setPosition(TRIGGER_INTERMITTENT);
                    triggerSequenceStep = 2;
                    firstFireComplete = true;
                    triggerTimer.reset();
                }
                break;
                
            case 2: // Intermittent position (brief pause)
                if (elapsedTime >= 0.2) {  // 0.2 second pause at intermittent
                    // Move to second fire position
                    triggerServo.setPosition(TRIGGER_FIRE);
                    triggerSequenceStep = 3;
                    triggerTimer.reset();
                }
                break;
                
            case 3: // Second fire position
                if (elapsedTime >= TRIGGER_FIRE_DURATION) {
                    // Return to home position
                    triggerServo.setPosition(TRIGGER_HOME);
                    triggerSequenceStep = 4;
                }
                break;
                
            case 4: // Sequence complete - advance indexer
                triggerSequenceActive = false;
                triggerSequenceStep = 0;
                firstFireComplete = false;
                
                // Advance indexer after double-fire sequence
                advanceIndexer();
                break;
        }
    }
    
    /**
     * Update shooter speed monitoring
     */
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
    
    /**
     * Update speed light based on shooter speed
     */
    private void updateSpeedLight() {
        if (!shooterRunning) {
            speedLight.setPosition(LIGHT_OFF_POSITION);
            return;
        }
        
        double currentVelocity = shooter.getVelocity();
        double speedPercentage = currentVelocity / currentShooterVelocity;
        
        if (shooterSpeedStable && speedPercentage > SHOOTER_SPEED_THRESHOLD) {
            // Speed is optimal and stable - green light
            speedLight.setPosition(LIGHT_GREEN_POSITION);
        } else if (currentVelocity > 50 && speedPercentage > SHOOTER_MIN_SPEED_THRESHOLD) {
            // Speed is acceptable but may not be stable - white light
            speedLight.setPosition(LIGHT_WHITE_POSITION);
        } else {
            // Speed is too low - off
            speedLight.setPosition(LIGHT_OFF_POSITION);
        }
    }
    
    // Getters for state information
    public boolean isShooterRunning() { return shooterRunning; }
    public double getCurrentShooterVelocity() { return currentShooterVelocity; }
    public boolean isShooterSpeedStable() { return shooterSpeedStable; }
    public boolean isIndexorMoving() { return indexorMoving; }
    public double getIndexorLastSuccessfulPosition() { return indexorLastSuccessfulPosition; }
    public boolean isTriggerSequenceActive() { return triggerSequenceActive; }
    public int getTriggerSequenceStep() { return triggerSequenceStep; }
    public int getIndexorCurrentPosition() { return indexor.getCurrentPosition(); }
    public double getShooterCurrentVelocity() { return shooter.getVelocity(); }
    
    // Conveyor control
    public void startConveyor() {
        conveyor.setPower(CONVEYOR_POWER);
    }
    
    public void stopConveyor() {
        conveyor.setPower(0);
    }
    
    public void startConveyorReverse() {
        conveyor.setPower(-CONVEYOR_POWER);
    }
    
    public void setConveyorPower(double power) {
        conveyor.setPower(power);
    }
    
    // Indexer position setter (for restoring from autonomous)
    public void setIndexorLastSuccessfulPosition(double position) {
        indexorLastSuccessfulPosition = position;
    }
    
    // Indexer hold/release for outtake function
    public void holdIndexer() {
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexor.setPower(0);
    }
    
    public void releaseIndexer() {
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        indexor.setPower(0);
    }
    
    // Intake control
    public void startIntake() {
        intake.setPower(INTAKE_POWER);
    }
    
    public void stopIntake() {
        intake.setPower(0);
    }
    
    public void startIntakeReverse() {
        intake.setPower(-INTAKE_POWER);
    }
    
    public void setIntakePower(double power) {
        intake.setPower(power);
    }
    
    public double getIntakePower() {
        return intake.getPower();
    }
    
    public boolean isIntakeRunning() {
        return Math.abs(intake.getPower()) > 0.1;
    }
}
