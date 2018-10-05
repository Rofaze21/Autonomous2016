package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(name="Gyro 7006", group="Pushbot")
public class Gyro7006 extends LinearOpMode {


    HardwarePushbot1         robot   = new HardwarePushbot1();
    ModernRoboticsI2cGyro   gyro    = null;
    private double PREFERRED_LIGHT_VALUE = .05d;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = .5 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.2;

    static final double     HEADING_THRESHOLD       = 1 ;
    static final double     P_TURN_COEFF            = 0.1;
    static final double     P_DRIVE_COEFF           = 0.15;

    ColorSensor colorSensor;
    OpticalDistanceSensor odsSensor;
    @Override
    public void runOpMode() {

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        boolean bLedOn = true;
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");

        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");


        robot.LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        robot.LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }
        gyro.resetZAxisIntegrator();

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
        telemetry.addData("Path", "Complete");
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.update();

        int start = 1;
        robot.Arm.setPosition(.5);
        while (opModeIsActive()) {


            switch (start) {
                case 1:
                    Shoot(1200);
                    sleep(300);// Shoot initial ball into center vortex
                    telemetry.addData("I am a Baller!", "Complete");
                    telemetry.update();
                   /* LauncherDown();
                    sleep(500);//Put Launcher down into position
                    Shoot(500);
                    telemetry.addData("I am a Baller 2!", "Complete");
                    telemetry.addData("State", start);
                    telemetry.update();*/

                    gyroDrive(DRIVE_SPEED, -6.5, 0);
                    start++;
                    break;
                case 2:
                    telemetry.addData("State", start);
                    telemetry.update();
                    gyroTurn(TURN_SPEED, -45);
                    gyroHold(TURN_SPEED, -45, 1.5);
                    telemetry.addData("Turning -45 degrees", "Complete");
                    telemetry.update();
                    sleep(50);
                    start++;
                    break;
                case 3:
                    gyroDrive(1.0, -35, -45);
                    start++;
                    break;
                case 4:
                    telemetry.addData("State", start);
                    telemetry.addData("Detecting light underneath", start);
                    telemetry.update();

                    if (odsSensor.getRawLightDetected() <= .055) {
                        robot.RightWheel.setPower(-.3);
                        robot.LeftWheel.setPower(-.3);
                    } else {
                        robot.RightWheel.setPower(0);
                        robot.LeftWheel.setPower(0);
                        start = 6;
                        break;
                    }
                    break;

                case 5:// do not go to case 5
                    followTapeUsingODS(5.0);
                    start++;
                    break;
                case 6:
                    telemetry.addData("State", start);
                    telemetry.update();
                    gyroTurn(TURN_SPEED, -90);
                    gyroHold(TURN_SPEED, -90, 1);
                    telemetry.addData("Turning -90 degrees", "Complete");
                    telemetry.update();
                    telemetry.addData("State", start);
                    telemetry.update();
                    gyroDrive(DRIVE_SPEED, -2, -90);
                    start++;
                    break;
                case 7:
                    telemetry.addData("State", start);
                    telemetry.update();
                    if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
                        telemetry.addData("Blue Detected!!", "Going straight!");
                        telemetry.update();
                        /*gyroDrive(DRIVE_SPEED, 4, -90);
                        gyroTurn(TURN_SPEED, 0);
                        gyroHold(TURN_SPEED, 0, 2);
                        gyroDrive(DRIVE_SPEED, -9, 0);
                        gyroTurn(TURN_SPEED, -90);
                        gyroHold(TURN_SPEED, -90, 1);*/
                        robot.Arm.setPosition(-1.5);
                        sleep(500);
                        robot.RightWheel.setPower(-.2);
                        robot.LeftWheel.setPower(-.2);
                        sleep(1500);
                        robot.RightWheel.setPower(.2);
                        robot.LeftWheel.setPower(.2);
                        sleep(500);
                        robot.RightWheel.setPower(-.2);
                        robot.LeftWheel.setPower(-.2);
                        sleep(1500);
                        robot.RightWheel.setPower(0);
                        robot.LeftWheel.setPower(0);
                        start = 8;
                        break;
                    } else {
                        telemetry.addData("Red Detected!", "Turning 90 Degrees");
                        telemetry.update();
                        robot.Arm.setPosition(1);
                        sleep(500);
                        robot.RightWheel.setPower(-.2);
                        robot.LeftWheel.setPower(-.2);
                        sleep(1500);
                        robot.RightWheel.setPower(.2);
                        robot.LeftWheel.setPower(.2);
                        sleep(500);
                        robot.RightWheel.setPower(-.2);
                        robot.LeftWheel.setPower(-.2);
                        sleep(1500);
                        robot.RightWheel.setPower(0);
                        robot.LeftWheel.setPower(0);
                        gyroDrive(DRIVE_SPEED, 5, -90);
                        gyroTurn(TURN_SPEED, 0);
                        gyroHold(TURN_SPEED, 0, 1);
                        start++;
                        break;
                    }
                case 8:
                    robot.RightWheel.setPower(0);
                    robot.LeftWheel.setPower(0);
                    telemetry.addData("DONE", "Cool");
                default:
                    telemetry.addData("Complete", "Good Job!");
                    telemetry.update();
            }
        }
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        if (opModeIsActive()) {

            telemetry.update();
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.LeftWheel.getCurrentPosition() + moveCounts;
            newRightTarget = robot.RightWheel.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.LeftWheel.setTargetPosition(newLeftTarget);
            robot.RightWheel.setTargetPosition(newRightTarget);

            robot.LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.LeftWheel.setPower(speed);
            robot.RightWheel.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.LeftWheel.isBusy() && robot.RightWheel.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.LeftWheel.setPower(leftSpeed);
                robot.RightWheel.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.LeftWheel.getCurrentPosition(),
                        robot.RightWheel.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.LeftWheel.setPower(0);
            robot.RightWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.LeftWheel.setPower(0);
        robot.RightWheel.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.LeftWheel.setPower(leftSpeed);
        robot.RightWheel.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void LauncherDown (){
        robot.Launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Launcher.setTargetPosition(270);
        robot.Launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Launcher.setPower(.4);
        robot.Collector.setPower(-1);
        sleep(2000);
        robot.Launcher.setPower(0);
        robot.Collector.setPower(0);
    }
    public void Shoot (int time){
        robot.Launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Launcher.setPower(1);
        sleep(time);
        robot.Launcher.setPower(0);
    }
    public void followTapeUsingODS(double holdTime) {
        double currentODSReading;
        double odsDelta;
        double powerToLeftWheel = -.1d;
        double powerToRightWheel = -.1d;
        boolean lineFollowerActivated = false;

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            currentODSReading = odsSensor.getLightDetected();
            odsDelta = PREFERRED_LIGHT_VALUE - currentODSReading;

            if (odsDelta < 0)
                lineFollowerActivated = true;

            if (lineFollowerActivated) {
                // odsDelta will be either positive or negative
                odsDelta *= 0.20;
                powerToLeftWheel = -.1d + odsDelta;
                powerToRightWheel = -.1d - odsDelta;
            }

            telemetry.addData("currentODSReading", currentODSReading);
            telemetry.addData("odsDelta", odsDelta);
            telemetry.addData("powerToLeftWheel", powerToLeftWheel);
            telemetry.addData("powerToRightWheel", powerToRightWheel);
            telemetry.update();

            robot.LeftWheel.setPower(powerToLeftWheel);
            robot.RightWheel.setPower(powerToRightWheel);

        }



    }
    }
