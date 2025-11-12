package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="WebcamServoAutoAim")
public class WebcamServoAutoAim extends OpMode {

    private Servo webcamServo;
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // --- Linear mapping: 0..300 deg  <->  0.0..1.0 position ---
    private static final double SERVO_TOTAL_DEG = 300.0;
    private static final double CENTER          = 0.50;    // 150° (middle of travel)
    private static final double DEAD_BAND_DEG   = 1.0;     // ignore tiny wobble
    private static final int    TARGET_ID       = 22;      // change to your tag ID

    // Flip this if it still looks away from the tag:
    private static final int SIGN = -1; // try +1 first; if wrong, set to -1

    // live state
    private double position = CENTER;

    @Override
    public void init() {
        webcamServo = hardwareMap.get(Servo.class, "WebcamServo");

        // Try FORWARD first. If tag-right still pans left after testing, flip SIGN to -1.
        webcamServo.setDirection(Servo.Direction.FORWARD);

        aprilTagWebcam.init(hardwareMap, telemetry);

        position = clamp(CENTER, 0.0, 1.0);
        webcamServo.setPosition(position);

        telemetry.addLine("WebcamServoAutoAim_Linear300 ready");
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TARGET_ID);

        if (tag != null) {
            double bearing = tag.ftcPose.bearing;   // degrees: +right, -left
            double absErr  = Math.abs(bearing);

            if (absErr < DEAD_BAND_DEG) {
                // hold steady to kill jitter around center
                webcamServo.setPosition(position);
                telemetry.addLine("Centered (deadband)");
            } else {
                // Direct linear mapping: CENTER +/- (bearing / 300)
                double target = CENTER + SIGN * (bearing / SERVO_TOTAL_DEG);

                // Keep inside 0..1 (full travel)
                position = clamp(target, 0.0, 1.0);
                webcamServo.setPosition(position);

                telemetry.addLine("Tag detected");
                telemetry.addData("Bearing (deg)", "%.2f", bearing);
            }
            telemetry.addData("Servo pos", "%.3f", position);
        } else {
            telemetry.addLine("No tag visible");
        }
        telemetry.update();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}