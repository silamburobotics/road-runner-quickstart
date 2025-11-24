package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shooter Speed Test OpMode - Tests consistent speed maintenance during firing
 * Tunable parameters for maintaining shooter velocity during 3-shot sequence
 */
@Config
@TeleOp(name = "Shooter Speed Test", group = "Test")
public class ShooterSpeedTest extends LinearOpMode {
    
    // Hardware
    private DcMotorEx shooter;
    private CRServo shooterServo;
    private Servo triggerServo;
    private Servo speedLight;
    
    // Tunable speed control parameters
    public static double TARGET_VELOCITY = 1300;  // Target shooter speed (ticks/sec)
    public static double VELOCITY_GAIN = 0.0003;  // Proportional gain for velocity correction
    public static double FEEDFORWARD_GAIN = 0.00065;  // Feedforward term for velocity
    public static double MIN_POWER = 0.3;  // Minimum motor power
    public static double MAX_POWER = 1.0;  // Maximum motor power
    
    // Tunable timing parameters
    public static double FIRE_DURATION = 0.3;  // How long trigger stays in fire position
    public static double ADVANCE_WAIT = 0.2;  // Wait time between shots
    public static int UPDATE_RATE_MS = 20;  // Speed update frequency (milliseconds)
    
    // Servo positions
    public static double TRIGGER_FIRE = 0.0;
    public static double TRIGGER_HOME = 0.5;
    public static double SHOOTER_SERVO_POWER = 1.0;
    
    // Speed monitoring
    public static double SPEED_TOLERANCE = 50;  // Acceptable speed deviation
    public static double STABILIZATION_TIME = 0.3;  // Time to reach stable speed
    
    // State tracking
    private boolean testActive = false;
    private int shotsFired = 0;
    private int testStep = 0;
    private ElapsedTime testTimer = new ElapsedTime();
    private ElapsedTime stabilizationTimer = new ElapsedTime();
    private ElapsedTime updateTimer = new ElapsedTime();
    
    // Speed tracking
    private double[] speedHistory = new double[3];  // Track speed for each shot
    private int historyIndex = 0;
    private boolean isStable = false;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
        triggerServo = hardwareMap.get(Servo.class, "triggerServo");
        speedLight = hardwareMap.get(Servo.class, "speedLight");
        
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        triggerServo.setPosition(TRIGGER_HOME);
        speedLight.setPosition(0.0);
        
        telemetry.addLine("=== SHOOTER SPEED TEST ===");
        telemetry.addLine("Gamepad1 B: Start 3-shot test");
        telemetry.addLine("Gamepad1 A: Stop shooter");
        telemetry.addLine("Tune parameters in FTC Dashboard");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Control inputs
            if (gamepad1.b && !testActive) {
                startTest();
            }
            
            if (gamepad1.a) {
                stopTest();
            }
            
            // Update shooter speed control
            if (testActive || shooter.getPower() > 0) {
                maintainShooterSpeed();
            }
            
            // Handle test sequence
            if (testActive) {
                runTestSequence();
            }
            
            // Update telemetry
            updateTelemetry();
        }
        
        // Cleanup
        shooter.setPower(0);
        shooterServo.setPower(0);
    }
    
    private void startTest() {
        // Start shooter and servo
        shooterServo.setPower(SHOOTER_SERVO_POWER);
        
        // Reset test state
        testActive = true;
        shotsFired = 0;
        testStep = 0;
        historyIndex = 0;
        testTimer.reset();
        stabilizationTimer.reset();
        updateTimer.reset();
        
        // Clear history
        for (int i = 0; i < speedHistory.length; i++) {
            speedHistory[i] = 0;
        }
    }
    
    private void stopTest() {
        testActive = false;
        shooter.setPower(0);
        shooterServo.setPower(0);
        triggerServo.setPosition(TRIGGER_HOME);
        speedLight.setPosition(0.0);
        testStep = 0;
        shotsFired = 0;
    }
    
    private void maintainShooterSpeed() {
        // Only update at specified rate
        if (updateTimer.milliseconds() < UPDATE_RATE_MS) {
            return;
        }
        updateTimer.reset();
        
        // Get current velocity
        double currentVel = shooter.getVelocity();
        
        // Calculate error
        double error = TARGET_VELOCITY - currentVel;
        
        // Calculate correction using proportional + feedforward
        double proportionalTerm = error * VELOCITY_GAIN;
        double feedforwardTerm = TARGET_VELOCITY * FEEDFORWARD_GAIN;
        double motorPower = proportionalTerm + feedforwardTerm;
        
        // Clamp power
        motorPower = Math.max(MIN_POWER, Math.min(MAX_POWER, motorPower));
        
        // Apply power
        shooter.setPower(motorPower);
        
        // Check stability
        boolean withinTolerance = Math.abs(error) < SPEED_TOLERANCE;
        if (withinTolerance && stabilizationTimer.seconds() >= STABILIZATION_TIME) {
            isStable = true;
            speedLight.setPosition(0.5);  // Green
        } else if (!withinTolerance) {
            stabilizationTimer.reset();
            isStable = false;
            speedLight.setPosition(1.0);  // White
        }
    }
    
    private void runTestSequence() {
        double elapsed = testTimer.seconds();
        
        switch (testStep) {
            case 0: // Wait for stabilization
                if (isStable) {
                    testStep = 1;
                    testTimer.reset();
                }
                break;
                
            case 1: // Shot 1 - Fire
                triggerServo.setPosition(TRIGGER_FIRE);
                if (elapsed >= FIRE_DURATION) {
                    triggerServo.setPosition(TRIGGER_HOME);
                    speedHistory[historyIndex++] = shooter.getVelocity();
                    shotsFired = 1;
                    testStep = 2;
                    testTimer.reset();
                }
                break;
                
            case 2: // Wait between shots
                if (elapsed >= ADVANCE_WAIT) {
                    testStep = 3;
                    testTimer.reset();
                }
                break;
                
            case 3: // Shot 2 - Fire
                triggerServo.setPosition(TRIGGER_FIRE);
                if (elapsed >= FIRE_DURATION) {
                    triggerServo.setPosition(TRIGGER_HOME);
                    speedHistory[historyIndex++] = shooter.getVelocity();
                    shotsFired = 2;
                    testStep = 4;
                    testTimer.reset();
                }
                break;
                
            case 4: // Wait between shots
                if (elapsed >= ADVANCE_WAIT) {
                    testStep = 5;
                    testTimer.reset();
                }
                break;
                
            case 5: // Shot 3 - Fire
                triggerServo.setPosition(TRIGGER_FIRE);
                if (elapsed >= FIRE_DURATION) {
                    triggerServo.setPosition(TRIGGER_HOME);
                    speedHistory[historyIndex++] = shooter.getVelocity();
                    shotsFired = 3;
                    testStep = 6;
                    testTimer.reset();
                }
                break;
                
            case 6: // Test complete
                testActive = false;
                break;
        }
    }
    
    private void updateTelemetry() {
        telemetry.addData("=== STATUS ===", "");
        telemetry.addData("Test Active", testActive);
        telemetry.addData("Shots Fired", "%d / 3", shotsFired);
        telemetry.addData("Speed Stable", isStable);
        
        telemetry.addData("\n=== SPEED ===", "");
        telemetry.addData("Target", "%.0f ticks/sec", TARGET_VELOCITY);
        telemetry.addData("Current", "%.0f ticks/sec", shooter.getVelocity());
        telemetry.addData("Error", "%.0f ticks/sec", TARGET_VELOCITY - shooter.getVelocity());
        telemetry.addData("Motor Power", "%.3f", shooter.getPower());
        
        telemetry.addData("\n=== TUNING ===", "");
        telemetry.addData("Velocity Gain", "%.6f", VELOCITY_GAIN);
        telemetry.addData("Feedforward", "%.6f", FEEDFORWARD_GAIN);
        telemetry.addData("Update Rate", "%d ms", UPDATE_RATE_MS);
        
        if (shotsFired > 0) {
            telemetry.addData("\n=== RESULTS ===", "");
            for (int i = 0; i < shotsFired; i++) {
                double deviation = speedHistory[i] - TARGET_VELOCITY;
                telemetry.addData("Shot " + (i+1), "%.0f (%.0f)", speedHistory[i], deviation);
            }
            
            if (shotsFired == 3) {
                double avgSpeed = (speedHistory[0] + speedHistory[1] + speedHistory[2]) / 3.0;
                double maxDev = Math.max(
                    Math.abs(speedHistory[0] - avgSpeed),
                    Math.max(
                        Math.abs(speedHistory[1] - avgSpeed),
                        Math.abs(speedHistory[2] - avgSpeed)
                    )
                );
                telemetry.addData("Average", "%.0f ticks/sec", avgSpeed);
                telemetry.addData("Max Deviation", "%.0f ticks/sec", maxDev);
                telemetry.addData("Consistency", maxDev < SPEED_TOLERANCE ? "GOOD ✓" : "NEEDS TUNING");
            }
        }
        
        telemetry.update();
    }
}
