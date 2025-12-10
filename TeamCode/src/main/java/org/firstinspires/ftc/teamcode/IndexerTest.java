package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Indexer Test", group="Testing")
public class IndexerTest extends LinearOpMode {
    
    // Declare motor
    private DcMotorEx indexor;
    
    // Indexor position settings
    public static final double INDEXOR_TICKS = 537.7/3;              // goBILDA 312 RPM motor: 120 degrees = 179 ticks
    public static final double AUTO_INDEXOR_POWER = 0.3;             // Power for automatic indexor movement
    public static final double INDEXOR_MOVE_TIMEOUT = 3.0;           // Maximum time to wait for indexor movement
    public static final double FREE_SPIN_DURATION = 3.0;             // Seconds to spin freely
    public static final double FREE_SPIN_POWER = 0.5;                // Power for free spinning
    
    public double IndexerPreviousPosition = 0.0;
    
    @Override
    public void runOpMode() {
        // Initialize indexor motor
        indexor = hardwareMap.get(DcMotorEx.class, "indexor");
        
        // Set motor direction
        indexor.setDirection(DcMotor.Direction.REVERSE);
        
        // Set zero power behavior
        indexor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Reset encoder
        indexor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Initialize previous position
        IndexerPreviousPosition = 0.0;
        
        telemetry.addData("Status", "Indexer Test - Ready");
        telemetry.addData("", "");
        telemetry.addData("Test Sequence", "");
        telemetry.addData("1.", "Advance indexer 3 times (120° each)");
        telemetry.addData("2.", "Free spin for 3 seconds");
        telemetry.addData("3.", "Advance indexer 3 times (120° each)");
        telemetry.addData("", "");
        telemetry.addData("Initial Position", "%.1f ticks", IndexerPreviousPosition);
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            executeTestSequence();
        }
    }
    
    private void executeTestSequence() {
        telemetry.addData("🤖 TEST STARTED", "Beginning indexer test sequence");
        telemetry.update();
        sleep(1000);
        
        // PHASE 1: Advance indexer 3 times
        telemetry.addData("🚀 PHASE 1", "Advancing indexer 3 times...");
        telemetry.update();
        
        for (int i = 1; i <= 3; i++) {
            telemetry.addData("📍 Advancement", "%d of 3", i);
            telemetry.update();
            moveIndexorToNextPosition();
            sleep(500); // Brief pause between advancements
        }
        
        telemetry.addData("✅ PHASE 1", "Complete - 3 advancements done");
        telemetry.addData("Position After Phase 1", "%d ticks", indexor.getCurrentPosition());
        telemetry.update();
        sleep(2000);
        
        // PHASE 2: Free spin for 3 seconds
        telemetry.addData("🚀 PHASE 2", "Free spinning for 3 seconds...");
        telemetry.update();
        
        freeSpinIndexor();
        
        // WAIT FOR BUTTON PRESS: Display current state and wait for user input
        telemetry.addData("✅ PHASE 2", "Complete - Free spin finished");
        telemetry.addData("", "");
        telemetry.addData("=== CURRENT STATE ===", "");
        telemetry.addData("📍 Current Position", "%d ticks", indexor.getCurrentPosition());
        telemetry.addData("📌 Indexer Previous Position", "%.1f ticks", IndexerPreviousPosition);
        telemetry.addData("📊 Position Difference", "%d ticks", indexor.getCurrentPosition() - (int)IndexerPreviousPosition);
        telemetry.addData("", "");
        telemetry.addData("⏸️ WAITING", "Press A button to continue with Phase 3");
        telemetry.update();
        
        // Wait for A button press
        while (opModeIsActive() && !gamepad1.a) {
            // Update display in case position changes
            telemetry.addData("✅ PHASE 2", "Complete - Free spin finished");
            telemetry.addData("", "");
            telemetry.addData("=== CURRENT STATE ===", "");
            telemetry.addData("📍 Current Position", "%d ticks", indexor.getCurrentPosition());
            telemetry.addData("📌 Indexer Previous Position", "%.1f ticks", IndexerPreviousPosition);
            telemetry.addData("📊 Position Difference", "%d ticks", indexor.getCurrentPosition() - (int)IndexerPreviousPosition);
            telemetry.addData("", "");
            telemetry.addData("⏸️ WAITING", "Press A button to continue with Phase 3");
            telemetry.update();
            sleep(100);
        }
        
        // Wait for button release to avoid double-trigger
        while (opModeIsActive() && gamepad1.a) {
            sleep(50);
        }
        
        telemetry.addData("▶️ RESUMING", "Starting Phase 3...");
        telemetry.update();
        sleep(1000);
        
        // PHASE 3: Advance indexer 3 times again
        telemetry.addData("🚀 PHASE 3", "Advancing indexer 3 times again...");
        telemetry.update();
        
        for (int i = 1; i <= 3; i++) {
            telemetry.addData("📍 Advancement", "%d of 3", i);
            telemetry.update();
            moveIndexorToNextPosition();
            sleep(500); // Brief pause between advancements
        }
        
        telemetry.addData("✅ PHASE 3", "Complete - 3 more advancements done");
        telemetry.addData("Position After Phase 3", "%d ticks", indexor.getCurrentPosition());
        telemetry.update();
        sleep(2000);
        
        // FINAL SUMMARY
        int finalPosition = indexor.getCurrentPosition();
        telemetry.addData("✅ TEST COMPLETE", "All phases finished!");
        telemetry.addData("", "");
        telemetry.addData("=== FINAL RESULTS ===", "");
        telemetry.addData("Final Position", "%d ticks", finalPosition);
        telemetry.addData("Expected Position", "%.1f ticks (6 × 120° = 720°)", INDEXOR_TICKS * 6);
        telemetry.addData("Position Error", "%d ticks", Math.abs(finalPosition - (int)(INDEXOR_TICKS * 6)));
        telemetry.addData("", "");
        telemetry.addData("Total Advancements", "6 (3 + 3)");
        telemetry.addData("Total Rotation", "%.0f degrees", (finalPosition / 537.7) * 360);
        telemetry.update();
        
        // Keep displaying results
        while (opModeIsActive()) {
            sleep(100);
        }
    }
    
    private void moveIndexorToNextPosition() {
        telemetry.addData("🔄 INDEXOR", "Moving to next position...");
        telemetry.update();
        
        // Get current position and calculate target
        double currentPosition = indexor.getCurrentPosition();
        double targetPosition = IndexerPreviousPosition + INDEXOR_TICKS;
        if (currentPosition > targetPosition + 5) {
            double indexerCorrection = INDEXOR_TICKS - (currentPosition % INDEXOR_TICKS);
            targetPosition = currentPosition + indexerCorrection;
        }

        IndexerPreviousPosition = targetPosition;

        // Set indexor to run to position
        indexor.setTargetPosition((int)targetPosition);
        indexor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexor.setPower(AUTO_INDEXOR_POWER);
        
        ElapsedTime indexorTimer = new ElapsedTime();
        indexorTimer.reset();
        
        // Wait for indexor to reach position
        while (opModeIsActive() && indexor.isBusy() && indexorTimer.seconds() < INDEXOR_MOVE_TIMEOUT) {
            telemetry.addData("🎯 Target Position", "%d ticks", (int)targetPosition);
            telemetry.addData("📍 Current Position", "%d ticks", indexor.getCurrentPosition());
            telemetry.addData("🔄 Indexor Status", indexor.isBusy() ? "Moving..." : "Complete");
            telemetry.addData("⏱️ Elapsed", "%.1f / %.1f seconds", indexorTimer.seconds(), INDEXOR_MOVE_TIMEOUT);
            telemetry.update();
            sleep(50);
        }

        // Stop indexor
        indexor.setPower(0);
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("✅ Indexor", "Position advanced to %d ticks", indexor.getCurrentPosition());
        telemetry.update();
    }
    
    private void freeSpinIndexor() {
        telemetry.addData("🔄 FREE SPIN", "Starting 3-second free spin...");
        telemetry.update();
        
        // Record position before free spin
        int positionBeforeSpin = indexor.getCurrentPosition();
        
        // Set to RUN_WITHOUT_ENCODER mode for free spinning
        indexor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexor.setPower(FREE_SPIN_POWER);
        
        ElapsedTime spinTimer = new ElapsedTime();
        spinTimer.reset();
        
        // Spin freely for specified duration
        while (opModeIsActive() && spinTimer.seconds() < FREE_SPIN_DURATION) {
            telemetry.addData("🌀 FREE SPINNING", "%.1f / %.1f seconds", spinTimer.seconds(), FREE_SPIN_DURATION);
            telemetry.addData("⚡ Power", "%.1f", FREE_SPIN_POWER);
            telemetry.addData("📍 Current Position", "%d ticks", indexor.getCurrentPosition());
            telemetry.addData("📊 Drift from Start", "%d ticks", indexor.getCurrentPosition() - positionBeforeSpin);
            telemetry.update();
            sleep(100);
        }
        
        // Stop spinning
        indexor.setPower(0);
        indexor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        int positionAfterSpin = indexor.getCurrentPosition();
        int positionDrift = positionAfterSpin - positionBeforeSpin;
        
        telemetry.addData("✅ Free Spin", "Complete!");
        telemetry.addData("Position Before", "%d ticks", positionBeforeSpin);
        telemetry.addData("Position After", "%d ticks", positionAfterSpin);
        telemetry.addData("Position Drift", "%d ticks (%.0f degrees)", positionDrift, (positionDrift / 537.7) * 360);
        telemetry.update();
        sleep(1000);
    }
}
