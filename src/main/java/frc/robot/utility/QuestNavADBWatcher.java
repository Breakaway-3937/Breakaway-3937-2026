// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.io.File;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Vision;

/**
 * QuestNavADBWatcher — translated from FRC 2429's gui/ui_updater.py
 *
 * <p>
 * Monitors the {@code /QuestNav/quest_in_passthrough} NetworkTables flag
 * published by {@link Vision} and fires an ADB command to bring the QuestNav
 * app back to the foreground when a double-tap passthrough event is detected.
 *
 * <h3>⚠️ Where to run this</h3>
 * FRC 2429 explicitly ran their ADB script on the <b>Driver Station laptop</b>
 * instead of the roboRIO to avoid spawning processes on a real-time OS.
 * You have two options:
 * <ol>
 * <li><b>On the roboRIO (simple)</b> — instantiate in {@code Robot.java}
 * and call {@link #start()}. Spawning a subprocess is technically
 * allowed on the Rio's Linux OS, but it can introduce brief jitter.
 * In practice most teams find this acceptable.</li>
 * <li><b>On the DS laptop (recommended)</b> — copy this file into a small
 * standalone Java project on the DS, point it at the roboRIO's NT server
 * ({@code NetworkTableInstance.getDefault().startClient4("10.39.37.2")}),
 * and run it separately. Use the same ADB path constants below.</li>
 * </ol>
 *
 * <h3>One-time setup (do this before a match)</h3>
 * Run the following on the laptop that will execute ADB to pair the Quest:
 * 
 * <pre>
 *   adb tcpip 5802
 *   adb connect 10.39.37.200:5802
 * </pre>
 * 
 * Port 5802 is within the FMS-allowed range. The Quest's DHCP address is
 * typically {@code 10.TE.AM.200} because it is the first device to request an
 * address from the robot radio. Adjust {@link #QUEST_ADB_ADDRESS} if yours
 * differs.
 *
 * <h3>Usage in Robot.java</h3>
 * 
 * <pre>
 * private final QuestNavADBWatcher questADB = new QuestNavADBWatcher();
 *
 * public Robot() {
 *     robotContainer = new RobotContainer();
 *     questADB.start();
 * }
 * </pre>
 */
public class QuestNavADBWatcher {

    // -------------------------------------------------------------------------
    // Configuration — adjust these for your team
    // -------------------------------------------------------------------------

    /**
     * IP:port of the QuestNav headset on the robot network.
     * Format: 10.TE.AM.200:5802 (replace TE.AM with your team number).
     * Team 3937 → 10.39.37.200:5802
     */
    private static final String QUEST_ADB_ADDRESS = "10.39.37.200:5802";

    /**
     * Path to the adb executable.
     * On the roboRIO (Linux): install adb via opkg or deploy it to /home/lvuser/adb
     * On a Windows DS laptop: point this to wherever adb.exe lives, e.g.:
     * "C:/Users/YourName/AppData/Local/Android/Sdk/platform-tools/adb.exe"
     * On a macOS/Linux DS laptop: usually just "adb" if it's on the PATH.
     */
    private static final String ADB_PATH = "/home/lvuser/adb"; // roboRIO default

    /** Seconds the passthrough flag must be true before we attempt ADB recovery. */
    private static final double ADB_TRIGGER_DELAY_S = 0.3;

    /** Minimum seconds between successive ADB attempts (cooldown). */
    private static final double ADB_COOLDOWN_S = 2.0;

    /** Maximum number of ADB recovery attempts per passthrough event. */
    private static final int ADB_MAX_RETRIES = 5;

    /** How often (ms) the watcher thread polls the NT flag. */
    private static final int POLL_INTERVAL_MS = 100;

    // -------------------------------------------------------------------------
    // Internal state — mirrors ui_updater.py's dtap_* fields
    // -------------------------------------------------------------------------
    private final BooleanSubscriber passthroughSub;
    private final ScheduledExecutorService executor;

    private double dtapStartTime = -1.0; // -1 = not currently in passthrough
    private double dtapLastFixTime = -1.0; // timestamp of last successful ADB call
    private int dtapRetries = 0;

    public QuestNavADBWatcher() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        passthroughSub = inst.getBooleanTopic(Vision.NT_PASSTHROUGH_KEY).subscribe(false);
        executor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "QuestNavADBWatcher");
            t.setDaemon(true); // won't prevent JVM exit
            return t;
        });
    }

    /**
     * Start the background polling thread.
     * Call once from {@code Robot()} constructor.
     */
    public void start() {
        executor.schedule(this::connectADB, 0, TimeUnit.SECONDS); // connect to Quest first
        executor.scheduleAtFixedRate(this::poll, 0, POLL_INTERVAL_MS, TimeUnit.MILLISECONDS);
        System.out.println("[QuestNavADBWatcher] Started. Watching " + Vision.NT_PASSTHROUGH_KEY);
    }

    /**
     * Stop the background thread (e.g. in a test teardown).
     * Not normally needed in robot code.
     */
    public void stop() {
        executor.shutdown();
    }

    // -------------------------------------------------------------------------
    // ADB TCP connection — called once at startup
    // -------------------------------------------------------------------------

    /**
     * Establishes the ADB TCP connection to the Quest.
     * Must run before any -s commands will work.
     * Requires {@code adb tcpip 5802} to have been run on the Quest via USB first.
     */
    private void connectADB() {
        try {
            File adbFile = new File(ADB_PATH);
            if (!adbFile.exists()) {
                System.err.println("[QuestNavADBWatcher] adb not found at: " + ADB_PATH
                        + " — update ADB_PATH in QuestNavADBWatcher.java");
                return;
            }

            Process p = new ProcessBuilder(ADB_PATH, "connect", QUEST_ADB_ADDRESS)
                    .redirectErrorStream(true)
                    .start();
            p.waitFor(3, TimeUnit.SECONDS);
            System.out.println("[QuestNavADBWatcher] ADB connect attempt to " + QUEST_ADB_ADDRESS + " complete.");
        } catch (Exception e) {
            System.err.println("[QuestNavADBWatcher] ADB connect failed: " + e.getMessage());
        }
    }

    // -------------------------------------------------------------------------
    // Core polling loop — translated from _check_questnav_passthrough_issue()
    // -------------------------------------------------------------------------
    private void poll() {
        try {
            boolean isInPassthrough = passthroughSub.get();
            double now = nowSeconds();

            if (isInPassthrough) {
                // Arm the timer the first time we see the flag go true
                if (dtapStartTime < 0) {
                    dtapStartTime = now;
                    System.out.printf("[%.1f] QuestNavADBWatcher: passthrough condition detected.%n", now);
                }

                double elapsedSinceTrigger = now - dtapStartTime;
                double elapsedSinceLastFix = (dtapLastFixTime < 0) ? Double.MAX_VALUE : (now - dtapLastFixTime);

                // Only act after the trigger delay to avoid reacting to brief blips
                if (elapsedSinceTrigger > ADB_TRIGGER_DELAY_S) {
                    if (elapsedSinceLastFix > ADB_COOLDOWN_S) {
                        if (dtapRetries < ADB_MAX_RETRIES) {
                            dtapRetries++;
                            System.out.printf(
                                    "[%.1f] QuestNavADBWatcher: passthrough > %.2fs — ADB fix attempt %d/%d%n",
                                    now, ADB_TRIGGER_DELAY_S, dtapRetries, ADB_MAX_RETRIES);
                            fireADBRelaunch();
                            dtapLastFixTime = now;
                            // Reset trigger so it needs another full delay before trying again
                            dtapStartTime = now;

                        } else if (dtapRetries == ADB_MAX_RETRIES) {
                            System.out.printf(
                                    "[%.1f] QuestNavADBWatcher: max retries (%d) reached. Giving up.%n",
                                    now, ADB_MAX_RETRIES);
                            dtapRetries++; // increment once more so this message only prints once
                        }
                    }
                }

            } else {
                // Flag cleared — passthrough resolved
                if (dtapStartTime >= 0) {
                    if (dtapRetries > 0) {
                        System.out.printf("[%.1f] QuestNavADBWatcher: passthrough resolved after %d attempt(s).%n",
                                now, dtapRetries);
                    }
                    dtapStartTime = -1.0;
                    dtapLastFixTime = -1.0;
                    dtapRetries = 0;
                }
            }

        } catch (Exception e) {
            // Never let an exception kill the scheduled thread
            System.err.println("[QuestNavADBWatcher] Unexpected error in poll(): " + e.getMessage());
        }
    }

    // -------------------------------------------------------------------------
    // ADB relaunch command
    // -------------------------------------------------------------------------

    /**
     * Fires the ADB shell command that brings QuestNav back to the foreground.
     *
     * <p>
     * Equivalent to 2429's:
     * 
     * <pre>
     *   adb -s 10.39.37.200:5802 shell am start -n \
     *     gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity
     * </pre>
     */
    private void fireADBRelaunch() {
        try {
            File adbFile = new File(ADB_PATH);
            if (!adbFile.exists()) {
                System.err.println("[QuestNavADBWatcher] adb not found at: " + ADB_PATH);
                return;
            }
            
            new ProcessBuilder(ADB_PATH, "-s", QUEST_ADB_ADDRESS,
                    "shell", "am", "start",
                    "-n", "gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity")
                    .redirectErrorStream(true)
                    .start();

            System.out.printf("[%.1f] QuestNavADBWatcher: ADB relaunch command sent to %s%n",
                    nowSeconds(), QUEST_ADB_ADDRESS);
        } catch (Exception e) {
            System.err.println("[QuestNavADBWatcher] Failed to execute ADB command: " + e.getMessage());
        }
    }

    /** Returns the current time in seconds from the system monotonic clock. */
    private static double nowSeconds() {
        return System.nanoTime() / 1e9;
    }
}