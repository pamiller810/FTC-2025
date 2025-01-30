package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Go")
public class Android_Studio_Test extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor BL_Motor;
    public DcMotor BR_Motor;
    public DcMotor FL_Motor;
    public DcMotor FR_Motor;
    public DcMotor UPPArm_Motor;
    public DcMotor Vertical_Rack;

    public CRServo Rack_Servo;
    public Servo Grabber_Servo;

    private final int maxVerticalRackHeight = 1818;
    private final int minVerticalRackHeight = 0;

    private final double GRABBER_CLOSED = 0.0; // Closed position
    private final double GRABBER_OPEN = 1.0; // Open position

    private final double gearTurnAmount = 0.05; //Amount to move the servo per button press (adjust as needed)

    private final double maxGearPosition = 1.0; // Maximum position for the servo (full turn, fully clockwise)
    private final double minGearPosition = 0.0; // Minimum position for the servo (full turn, fully counter-clockwise)

    public Android_Studio_Test(HardwareMap hardwareMap, Telemetry telemetry) {
    }

    @Override
    public void runOpMode() {
        // Hardware Mapping
        BR_Motor = hardwareMap.get(DcMotor.class, "BR Wheel Motor");
        if (BR_Motor == null) {
            telemetry.addData("Error", "BR Wheel Motor not found.");
            return;
        }

        FL_Motor = hardwareMap.get(DcMotor.class, "FL Wheel Motor");
        BL_Motor = hardwareMap.get(DcMotor.class, "BL Wheel Motor");
        FR_Motor = hardwareMap.get(DcMotor.class, "FR Wheel Motor");
        UPPArm_Motor = hardwareMap.get(DcMotor.class, "Upper Arm Motor");
        Vertical_Rack = hardwareMap.get(DcMotor.class, "Vertical Rack");
        Grabber_Servo = hardwareMap.get(Servo.class, "Grabber_Servo");
        Rack_Servo = hardwareMap.get(CRServo.class, "Rack_Servo");

        // Reset and configure vertical rack encoder
        Vertical_Rack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Vertical_Rack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Motor direction configuration
        BR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        FR_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        BL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        FL_Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure that the UPPArm_Motor direction is correct
        UPPArm_Motor.setDirection(DcMotorSimple.Direction.FORWARD); // Or REVERSE depending on your setup

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Mecanum drive calculations
            double axial = gamepad1.left_stick_y; // Forward/backward
            double lateral = -gamepad1.left_stick_x; // Left/right
            double yaw = -gamepad1.right_stick_x; // Rotation

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the powers
            // double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            // maxPower = Math.max(maxPower, Math.abs(leftBackPower));
            // maxPower = Math.max(maxPower, Math.abs(rightBackPower));
            double maxPower = 0.5;

            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightBackPower /= maxPower;
            }

            // Set motor powers
            FL_Motor.setPower(leftFrontPower);
            FR_Motor.setPower(rightFrontPower);
            BL_Motor.setPower(leftBackPower);
            BR_Motor.setPower(rightBackPower);

            // Vertical Rack Control with Limits and Adjusted Speed
            double verticalRackSpeed = (gamepad2.right_trigger - gamepad2.left_trigger); // Full range of -1 to 1

            // Ensure correct movement direction for the Vertical Rack Motor
            if (Vertical_Rack.getCurrentPosition() <= minVerticalRackHeight && verticalRackSpeed < 0) {
                verticalRackSpeed = 0; // Prevent going below minimum
            } else if (Vertical_Rack.getCurrentPosition() >= maxVerticalRackHeight && verticalRackSpeed > 0) {
                verticalRackSpeed = 0; // Prevent going above maximum
            }

            Vertical_Rack.setPower(verticalRackSpeed);

            // UPPArm Control with Limits
            double uppArmSpeed = -gamepad2.right_stick_y * 0.75; // Adjust speed and invert Y-axis for correct movement direction

            // Optional: Add position limits for the UPPArm_Motor (if needed)
            if (UPPArm_Motor.getCurrentPosition() <= 0 && uppArmSpeed < 0) {
                uppArmSpeed = 0; // Prevent moving below 0 (down limit)
            } else if (UPPArm_Motor.getCurrentPosition() >= 388 && uppArmSpeed > 0) { // Replace 1000 with your own upper limit
                uppArmSpeed = 0; // Prevent moving above max height (up limit)
            }

            UPPArm_Motor.setPower(uppArmSpeed);

            // Gradual Grabber Control (Open/Close with Triggers)
            double grabberPosition = GRABBER_CLOSED + (gamepad2.right_trigger * (GRABBER_OPEN - GRABBER_CLOSED));
//            Grabber_Servo.setPosition(grabberPosition);
//
//            // Gradual Gear Turn Control with Rack Servo
//            double rackServoPosition = Rack_Servo.getPosition(); //Get the current position of the servo

            //Control rack servo using triggers or buttons for incremental movement

            if (gamepad2.left_stick_y > 0.1) {
                //move the gear clockwise (incremental movements)
                Rack_Servo.setPower(0.5);

            } else if (gamepad2.left_stick_y < -0.1) {
                //move the gear counter-clockwise (decremental movements)
                Rack_Servo.setPower(-0.5);
//                rackServoPosition = Math.max(minGearPosition, rackServoPosition - gearTurnAmount);
            }
            else {
                Rack_Servo.setPower(0);
            }

            if(gamepad2.a) {
                // Open Grabber servo
                Grabber_Servo.setPosition(0.55);
            }
            else if(gamepad2.y) {
                // Close Grabber Servo
                Grabber_Servo.setPosition(1);
            }

//            Rack_Servo.setPosition(rackServoPosition); // Set the servo to the new position

            // Telemetry for debugging
            telemetry.addData("Rack Position", Vertical_Rack.getCurrentPosition());
            telemetry.addData("Arm Position", UPPArm_Motor.getCurrentPosition()); // Display arm position
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update();
        }
    }
}
