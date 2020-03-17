#include "corridor_state.h"
#include "state_manager.h"
#include "roomba.h"
#include "sonar.h"

#if ROOMBA_NUM == 1

MoveState stateAfterRotation = Forward;


unsigned int lastPeakTime = 250;
int lastRight = 0;

bool firstTime = true;


int getRightPeak(int minimum)
{
    int sonarRight = sonar_get(0);
    int delta = sonarRight - lastRight; // TODO: try to replace lastSonarRight with minimum
    lastRight = sonarRight;

    if (abs(delta) > MIN_WALL_THICKNESS && millis() - lastPeakTime > MAX_TIME_PASS_WALL)
    {
        // Purge first (from zero pos)
        if (firstTime)
        {
            firstTime = false;
            return 0;
        }

        lastPeakTime = millis();

        return delta;
    }

    return 0;
}

unsigned long nextDebugTime = millis();

void corridor_state_loop()
{
    if (millis() >= nextDebugTime)
    {
        nextDebugTime = millis() + 200;
        print_f("Journey: %i ; Target distance: %i ; Right: %i ; Min: %i\n", (int)lastCoveredDistance, (int)untilDistance, sonar_get(0), (int)minimum);
    }

    // Set initial PID min
    if (minimum <= 0)
    {
        minimum = sonar_get(0);
    }

    // Get center-right distance from about 5 to 300 cm
    int frontDistance = sonar_get(2) < 1 ? 300 : sonar_get(2);

    if (moveState == AwaitObstacleLeave)
    {
        awaitObstacleLeave(); //меняем состояние назад при удачном стечении обстоятельств
    }
    else if (moveState == Stop)
    {
        driveDirect(0, 0);
    }
    else if (moveState == AlongWallUntilDistance) // 0, 2
    {
        drivePIDWall(minimum);

        ///  |
        ///  |
        ///  |
        ///  |____       <-->
        ///^     |
        // ()    |

        minimum += getRightPeak(minimum); // обратно меняем

        awaitObstacle(untilDistance, 30 + 2 * minimum);

        if (areWeAtDestination() || doSeeVirtualWall())
        {
            //movementType = Stop;
            moveState = Left;
            stateAfterRotation = Forward;
            untilDistance = roomWidth;
        }
    }
    else if (moveState == Forward)
    {
        driveForwardWithRegulation(100, encoderLeft - lastLeftEnc, encoderRight - lastRightEnc);
        moveState = GoingForward;
    }
    else if (moveState == GoingForward)
    {
        driveForwardWithRegulation(100, encoderLeft - lastLeftEnc, encoderRight - lastRightEnc);
        awaitObstacle(untilDistance, 35 + 2 * minimum);

        if (frontDistance < minimum + 10 && areWeAtDestination())
        {
            stateAfterRotation = AlongWallUntilDistance;
            untilDistance = roomLength;
            moveState = Left;
        }
    }
    else if (moveState == Left)
    {
        extraRot = (encoderLeft - encoderRight) / TICKS_ONE_TURN * PI;
        resetInitEncoders();
        moveState = RotatingLeft;
        driveDirect(0, 0);
    }
    else if (moveState == RotatingLeft)
    {
        driveRotateWithRegulation(100 * clamp(howCloseIsAngle(), 0.5f, 1), 1, encoderLeft - lastLeftEnc, encoderRight - lastRightEnc);

        if (areWeAtAngle())
        {
            minimum = sonar_get(0);

            driveDirect(0, 0);
            //minimum = sonar_get(0);
            moveState = stateAfterRotation;
            resetInitEncoders();
            wallIndex = (wallIndex + 1) % 4;
        }
    }
    // end long corridor
}

#endif