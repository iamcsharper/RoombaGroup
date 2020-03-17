#include "group_move.h"
#include "roomba.h"

float target_x = 0;
float target_y = 0;

#define TARGET_MOVE_TIMER 15
unsigned long target_move_timer = 0;
float x = 0, y = 0;
float old_x = 0, old_y = 0;

float calc_angle_to_target()
{
    float dir_x = target_x - x;
    float dir_y = target_y - y;

    float dir_len = sqrtf(dir_x * dir_x + dir_y * dir_y);

    dir_y = dir_y / dir_len;

    return acos(dir_y) * sign(dir_x);
}

float old_rotation = 0;
float rotation = 0;
float rotation_target = 0;
float extra_rotation = 0;
int rotation_dir = 1;

int lastTimeMovingForward = 0;
bool rotEpsilonParity = true;

float journeyLength = 0;

void slave_move_loop()
{
    int packet_size = udp_get();

    if (packet_size)
    {
        if (incomingPacket[0] == 0)
        {
            moveState = Stop;
            print("Stop\n");
        }
        else if (incomingPacket[0] == 1)
        {
            moveState = RotateToTarget;

            target_x = (signed short)((signed short)(incomingPacket[1]) << 8 | incomingPacket[2]);
            target_y = (signed short)((signed short)(incomingPacket[3]) << 8 | incomingPacket[4]);

            print_f("Move to %f %f", target_x, target_y);
        }

        // Reset
        for (int i = 0; i < packet_size; i++)
        {
            incomingPacket[i] = 255;
        }
    }

    //float distance = (encoderLeft + encoderRight) / 2.0f * TICKS_TO_CM;

    float sqrDistanceToTarget = (target_x - x) * (target_x - x) + (target_y - y) * (target_y - y);

    if (moveState == Stop)
    {
        driveDirect(0, 0);
    }
    else if (moveState == RotateToTarget)
    {
        // Вычисляем поворот
        rotation_target = calc_angle_to_target();

        rotation_dir = 1;
        if (rotation_target < rotation)
        {
            rotation_dir = -1;
        }

        driveRotateWithRegulation(30, rotation_dir, abs(encoderLeft - lastLeftEnc), abs(encoderRight - lastRightEnc));

        resetInitEncoders();
        old_rotation = rotation; // Сохраняем текущее вращение
        moveState = RotatingToTarget;
    }
    else if (moveState == RotatingToTarget)
    {
        lastTimeMovingForward = millis();

        // На сколько радиан мы повернули
        ///////////////////// <------------------------------------------>   -   сколько полных оборотов мы совершили (в виде дроби)

        float rotError = (rotation - rotation_target);

        driveRotateWithRegulation(120 * clamp(abs(rotError), 0.2f, 1) + 5, rotation_dir, abs(encoderLeft - lastLeftEnc), abs(encoderRight - lastRightEnc));
        rotation = old_rotation + rotation_dir * lastRotationDelta; // Текущее вращение есть то что было плюс то что "набежало" к текущему моменту
        bool shouldWeStopRotating = false;

        if (rotEpsilonParity)
        {
            // Паритет в сторону повернуть больше чем надо
            shouldWeStopRotating = rotation_dir * rotError > 0.001f;
        }
        else
        {
            // Паритет в сторону повернуть меньше чем надо
            shouldWeStopRotating = abs(rotError) < 0.001f;
        }
        rotEpsilonParity = !rotEpsilonParity;

        if (shouldWeStopRotating)
        {
            driveDirect(0, 0);
            moveState = Forward;

            resetInitEncoders();
            old_rotation = rotation; // Сохраняем текущее вращение
        }
    }
    else if (moveState == Forward)
    {
        driveForwardWithRegulation(250, abs(encoderLeft - lastLeftEnc), abs(encoderRight - lastRightEnc));

        old_x = x;
        old_y = y;

        journeyLength = sqrDistanceToTarget;

        resetInitEncoders();

        moveState = GoingForward;
    }
    else if (moveState == GoingForward)
    {
        // Условие выхода - мы подошли вплотную к точке
        if (sqrDistanceToTarget < 12 * 12)
        {
            moveState = Stop;
        }

        float speedCoef = 1;

        if (sqrDistanceToTarget / journeyLength < 0.01f)
            speedCoef = clamp(sqrDistanceToTarget / journeyLength, 0.4f, 1);

        driveForwardWithRegulation(250 * speedCoef, encoderLeft - lastLeftEnc, encoderRight - lastRightEnc);

        // float dx = distance * (float)sin(rotation + (0.785398163f - 0.02f));
        // float dy = distance * (float)cos(rotation + (0.785398163f - 0.02f));
        float dx = lastCoveredDistance * (float)sin(rotation);
        float dy = lastCoveredDistance * (float)cos(rotation);

        x = old_x + dx;
        y = old_y + dy;
    }
}

void master_move_loop()
{
    if (millis() - target_move_timer > TARGET_MOVE_TIMER)
    {
    }
}