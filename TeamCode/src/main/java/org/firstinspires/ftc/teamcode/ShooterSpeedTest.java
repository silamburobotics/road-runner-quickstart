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
    public static double VELOCITY_P = 1.2;  // Proportional coefficient for internal PID
    public static double VELOCITY_I = 0.08;  // Integral coefficient for internal PID (reduced to prevent wind-up)
    public static double VELOCITY_D = 0.15;  // Derivative coefficient for internal PID (dampen oscillation)
    public static double VELOCITY_F = 11.8;  // Feedforward coefficient (reduced to prevent overshoot)
    
    // Tunable timing parameters
    public static double FIRE_DURATION = 0.3;  // How long trigger stays in fire position
    public static double ADVANCE_WAIT = 0.2;  // Wait time between shots
    
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
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set initial PID coefficients
        shooter.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
        
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
            
            // Update PID coefficients if changed via dashboard
            shooter.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
            
            // Maintain shooter speed using internal PID
            if (testActive || shooter.getVelocity() > 0) {
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
        shooter.setVelocity(0);
        shooterServo.setPower(0);
    }
    
    private void startTest() {
        // Start shooter and servo with internal PID velocity control
        shooter.setVelocity(TARGET_VELOCITY);
        shooterServo.setPower(SHOOTER_SERVO_POWER);
        
        // Reset test state
        testActive = true;
        shotsFired = 0;
        testStep = 0;
        historyIndex = 0;
        testTimer.reset();
        stabilizationTimer.reset();
        
        // Clear history
        for (int i = 0; i < speedHistory.length; i++) {
            speedHistory[i] = 0;
        }
    }
    
    private void stopTest() {
        testActive = false;
        shooter.setVelocity(0);
        shooterServo.setPower(0);
        triggerServo.setPosition(TRIGGER_HOME);
        speedLight.setPosition(0.0);
        testStep = 0;
        shotsFired = 0;
    }
    
    private void maintainShooterSpeed() {
        // Motor controller handles PID internally - just maintain target velocity
        shooter.setVelocity(TARGET_VELOCITY);
        
        // Get current velocity
        double currentVel = shooter.getVelocity();
        
        // Calculate error
        double error = Math.abs(TARGET_VELOCITY - currentVel);
        
        // Check stability
        boolean withinTolerance = error < SPEED_TOLERANCE;
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
        
        telemetry.addData("\n=== PID TUNING ===", "");
        telemetry.addData("P (Proportional)", "%.2f", VELOCITY_P);
        telemetry.addData("I (Integral)", "%.2f", VELOCITY_I);
        telemetry.addData("D (Derivative)", "%.2f", VELOCITY_D);
        telemetry.addData("F (Feedforward)", "%.2f", VELOCITY_F);
        
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
