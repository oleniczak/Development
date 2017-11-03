package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Practice - RED/LEFT", group="Autonomous")
public class AutoWithCommands extends LinearOpMode
{
    private  HardRobot  robot   = new HardRobot();   // Use a Pushbot's hardware
    private  Commands   cmd   = new Commands();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    private static String strAlliance;  // = "RED";
    private static String strPosition;  // = "LEFT";

    private static int distJewel = 3;
    private static int distColAdj = 0;
    private static int colNo=0;

    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.5;

    private static boolean blueFound = false;
    private static boolean redFound = false;

    private RelicRecoveryVuMark image;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Initialization", "Starting...");
        telemetry.update();

        // Initialize the hardware
        robot.init(hardwareMap);

        sleep(500);     // pause for servos to move

        while (opModeIsActive() && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad2.left_bumper && !gamepad2.right_bumper)
        {
            if (gamepad1.left_bumper)
            {
                strAlliance = "RED";
                strPosition = "LEFT";
            }
            else if (gamepad1.right_bumper)
            {
                strAlliance = "RED";
                strPosition = "RIGHT";
            }
            else if (gamepad2.left_bumper)
            {
                strAlliance = "BLUE";
                strPosition = "LEFT";
            }
            else
            {
                strAlliance = "BLUE";
                strPosition = "RIGHT";
            }
        }

        telemetry.addData("Initialization", "Initialization Complete!");
        telemetry.addData("","");
        telemetry.addData("Configuration","Configured for " + strAlliance + " alliance " + strPosition + "position!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //*******************************************************************************
        // 1) Drop Arm
        //*******************************************************************************
        telemetry.addData("Action", "Dropping Arm...");
        telemetry.update();

        robot.bigAss.setPosition(0.65);

        telemetry.addData("Action", "Arm Drop Complete!");
        telemetry.update();


        //*******************************************************************************
        // 2) Sense Jewel Color
        //*******************************************************************************
        telemetry.addData("Action", "Sensing Jewel Color...");
        telemetry.update();

        robot.Color.enableLed(true);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 5)
        {
            telemetry.addData("SenseJewel", "Red:" + robot.Color.red());
            telemetry.addData("SenseJewel", "Blue:" + robot.Color.blue());
            telemetry.update();
        }

        if (robot.Color.red() >= 1 && robot.Color.red() <= 100)
        {
            redFound = true;
        }
        if (robot.Color.blue() >= 1 && robot.Color.blue() <= 100)
        {
            blueFound = true;
        }

        robot.Color.enableLed(false);

        telemetry.addData("Action", "Sensing Jewel Color Complete!");
        telemetry.update();


        //*******************************************************************************
        // Sense Crypto Key Image
        //*******************************************************************************
        telemetry.addData("Action", "Sensing Crypto Key...");
        telemetry.update();

        //robot.targets.activate();
        //image = RelicRecoveryVuMark.from(robot.target);

        //if (image == RelicRecoveryVuMark.LEFT)
        {
            colNo = 1;
        }
        //else if (image == RelicRecoveryVuMark.CENTER)
        //{
        //    colNo=2;
        //}
        //else if (image == RelicRecoveryVuMark.RIGHT)
        //{
        //    colNo = 3;
        //}

        if (strAlliance.equals("RED"))
        {
            distColAdj = 24 - (colNo * 8) + 4;
        }
        else  //BLUE
        {
            distColAdj = (colNo * 8) - 4;
        }
        //robot.targets.deactivate();   //??

        telemetry.addData("Action", "Sensing Crypto Key Complete!");
        telemetry.addData("","");
        telemetry.addData("Detected", "Key found for column " + colNo);
        telemetry.update();


        //*******************************************************************************
        // 3) Lift Block
        //*******************************************************************************
        telemetry.addData("Action", "Lifting Glyph...");
        telemetry.update();

        robot.leftClaw.setPosition(0.0);
        robot.rightClaw.setPosition(1.0);

        runtime.reset();
        robot.liftMotor.setPower(0.2);
        while (opModeIsActive() && runtime.milliseconds() < 5)
        {
            telemetry.addData("LiftBlock", runtime.toString());
            telemetry.update();
        }

        telemetry.addData("Action", "Glyph Lift Complete!");
        telemetry.update();


        //*******************************************************************************
        // 4) Hit Jewel
        //*******************************************************************************
        telemetry.addData("Action", "Hitting Jewel...");
        telemetry.update();

        if ((strAlliance.equals("RED") && redFound) || (strAlliance.equals("BLUE") && blueFound))
        {
            cmd.encoderMove(DRIVE_SPEED, distJewel, "DRV", 2);
            //distJewel = distJewel;
        }
        else
        {
            cmd.encoderMove(DRIVE_SPEED, distJewel, "DFW", 2);
            distJewel = distJewel * (-1);
        }

        telemetry.addData("Action", "Jewel Hit Complete!");
        telemetry.update();


        //*******************************************************************************
        // 5) Retract Arm
        //*******************************************************************************
        telemetry.addData("Action", "Retracting Arm...");
        telemetry.update();

        robot.bigAss.setPosition(0.0);

        telemetry.addData("Action", "Arm Retraction Complete!");
        telemetry.update();


        //*******************************************************************************
        // 6) Drive off stone, align with crypto, approach crypto
        //*******************************************************************************
        telemetry.addData("Action", "Driving to Crypto...");
        telemetry.update();

        if (strAlliance.equals("RED") && strPosition.equals("LEFT"))
        {
            //Drive off stone
            cmd.encoderMove(DRIVE_SPEED, distJewel + 24 + distColAdj, "DFW", 2);
            //Align with Crypto
            cmd.encoderMove(TURN_SPEED, 9, "TRT", 2);
        }
        else if (strAlliance.equals("RED") && strPosition.equals("RIGHT"))
        {
            //Drive off stone
            cmd.encoderMove(DRIVE_SPEED, distJewel + 24, "DFW", 2);
            //Align with Crypto
            cmd.encoderMove(DRIVE_SPEED, distColAdj, "DLT", 2);
        }
        else if (strAlliance.equals("BLUE") && strPosition.equals("LEFT"))
        {
            //Drive off stone
            cmd.encoderMove(DRIVE_SPEED, distJewel + 24, "DRV", 2);
            //Align with Crypto
            cmd.encoderMove(TURN_SPEED, 18, "TRT", 2);
            cmd.encoderMove(DRIVE_SPEED, distColAdj, "DRT", 2);
        }
        else if (strAlliance.equals("BLUE") && strPosition.equals("RIGHT"))
        {
            //Drive off stone
            cmd.encoderMove(DRIVE_SPEED, distJewel + 24 + distColAdj, "DRV", 2);
            //Align with Crypto
            cmd.encoderMove(TURN_SPEED, 9, "TRT", 2);
        }

        // Approach crypto
        cmd.encoderMove(DRIVE_SPEED, 12, "DFW", 2);

        telemetry.addData("Action", "Drive to Crypto Complete!");
        telemetry.update();


        //*******************************************************************************
        // 7) Place Block
        //*******************************************************************************
        telemetry.addData("Action", "Placing Glyph...");
        telemetry.update();

        runtime.reset();
        robot.liftMotor.setPower(-0.2);
        while (opModeIsActive() && runtime.milliseconds() < 5)
        {
            telemetry.addData("LiftBlock", runtime.toString());
            telemetry.update();
        }

        robot.leftClaw.setPosition(1.0);
        robot.rightClaw.setPosition(0.0);

        telemetry.addData("Action", "Glyph Placement Complete!");
        telemetry.update();
    }
}
