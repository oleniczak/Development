package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Commands
{
    LinearOpMode opMode;

    private HardRobot   robot   = new HardRobot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public void encoderMove(double speed, double inches, String action,
                            //double leftInches, double rightInches,
                            double timeoutS) {
        int targetLeftFront;
        int targetRightFront;
        int targetLeftBack;
        int targetRightBack;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive())
        {
            if (action.equals("DFW"))  //Drive Forward
            {
                // Determine new target position, and pass to motor controller
                targetLeftFront = robot.leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                targetRightFront = robot.rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                targetLeftBack = robot.leftBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                targetRightBack = robot.rightBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            }
            else if (action.equals("DRV")) //Drive Reverse
            {
                targetLeftFront = robot.leftFront.getCurrentPosition() + (int) (inches*(-1) * COUNTS_PER_INCH);
                targetRightFront = robot.rightFront.getCurrentPosition() + (int) (inches*(-1) * COUNTS_PER_INCH);
                targetLeftBack = robot.leftBack.getCurrentPosition() + (int) (inches*(-1) * COUNTS_PER_INCH);
                targetRightBack = robot.rightBack.getCurrentPosition() + (int) (inches*(-1) * COUNTS_PER_INCH);
            }
            else if (action.equals("DLT")) //Drive Left
            {
                targetLeftFront = robot.leftFront.getCurrentPosition() + (int) (inches*(-1) * COUNTS_PER_INCH);
                targetRightFront = robot.rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                targetLeftBack = robot.leftBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                targetRightBack = robot.rightBack.getCurrentPosition() + (int) (inches*(-1) * COUNTS_PER_INCH);
            }
            else if (action.equals("DRT")) //Drive Right
            {
                targetLeftFront = robot.leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                targetRightFront = robot.rightFront.getCurrentPosition() + (int) (inches*(-1) * COUNTS_PER_INCH);
                targetLeftBack = robot.leftBack.getCurrentPosition() + (int) (inches*(-1) * COUNTS_PER_INCH);
                targetRightBack = robot.rightBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            }
            else if (action.equals("TLT")) //Turn Left
            {
                targetLeftFront = robot.leftFront.getCurrentPosition() + (int) (inches*(-1) * COUNTS_PER_INCH);
                targetRightFront = robot.rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                targetLeftBack = robot.leftBack.getCurrentPosition() + (int) (inches*(-1) * COUNTS_PER_INCH);
                targetRightBack = robot.rightBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            }
            else //(action.equals("TRT"))  //Turn Right
            {
                targetLeftFront = robot.leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                targetRightFront = robot.rightFront.getCurrentPosition() + (int) (inches*(-1) * COUNTS_PER_INCH);
                targetLeftBack = robot.leftBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                targetRightBack = robot.rightBack.getCurrentPosition() + (int) (inches*(-1) * COUNTS_PER_INCH);
            }

            robot.leftFront.setTargetPosition(targetLeftFront);
            robot.rightFront.setTargetPosition(targetRightFront);
            robot.leftBack.setTargetPosition(targetLeftBack);
            robot.rightBack.setTargetPosition(targetRightBack);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));
            robot.leftBack.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy()))
            {
                // Display it for the driver.
                opMode.telemetry.addData("EncoderMove",  "Running to %7d :%7d :%7d :%7d", targetLeftFront,  targetRightFront, targetLeftBack, targetRightBack);
                opMode.telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
