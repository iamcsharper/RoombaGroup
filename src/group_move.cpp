#include "group_move.h"

enum MSState
{
    Stop,
    Rotate,
    Rotating,
    Go_Forward,
    Going_Forward
} _state,
    _prevState;

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

void slave_move_loop()
{
    int packet_size = 0;
    if (packet_size = udp_get())
    {
        if (incomingPacket[0] == 0)
        {
            _state = Stop;
            print("Stop\n");
        }
        else if (incomingPacket[0] == 1)
        {
            _state = Rotate;

            target_x = (signed short)((signed short)(incomingPacket[1]) << 8 | incomingPacket[2]);
            target_y = (signed short)((signed short)(incomingPacket[3]) << 8 | incomingPacket[4]);

            print_f("Move to %f %f", target_x, target_y);
        }
    }
    for (int i = 0; i < packet_size; i++)
    {
        incomingPacket[i] = 255;
    }

    if (millis() - target_move_timer > TARGET_MOVE_TIMER)
    {
        //delta encoders and distance
        int encoderLeft = encoderCountsLeft - initEncoderLeft;
        int encoderRight = encoderCountsRight - initEncoderRight;
        float distance = (encoderLeft + encoderRight) / 2.0f * TICKS_TO_CM;

        float sqrDistanceToTarget = (target_x - x) * (target_x - x) + (target_y - y) * (target_y - y);

        bool iSeeLegs = (sonar_get(3) > 0 && sonar_get(3) < 31);
        iSeeLegs |= (sonar_get(2) > 0 && sonar_get(2) < 31);
        iSeeLegs |= lbCenterLeft > 40;
        iSeeLegs |= lbCenterRight > 40;

        if (_state == Stop)
        {
            driveDirect(0, 0);
        }
        else if (_state == Rotate)
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
            _state = Rotating;
        }
        else if (_state == Rotating)
        {
            lastTimeMovingForward = millis();

            // На сколько радиан мы повернули
            ///////////////////// <------------------------------------------>   -   сколько полных оборотов мы совершили (в виде дроби)
            float deltaRot = (float)(abs(encoderLeft - encoderRight)) / TICKS_ONE_TURN * PI;
            float rotError = (rotation - rotation_target);

            driveRotateWithRegulation(120 * clamp(abs(rotError), 0.2f, 1) + 5, rotation_dir, abs(encoderLeft - lastLeftEnc), abs(encoderRight - lastRightEnc));
            rotation = old_rotation + rotation_dir * deltaRot; // Текущее вращение есть то что было плюс то что "набежало" к текущему моменту
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
                _state = Go_Forward;

                resetInitEncoders();
                old_rotation = rotation; // Сохраняем текущее вращение
            }
        }
        else if (_state == Go_Forward)
        {
            driveForwardWithRegulation(250, abs(encoderLeft - lastLeftEnc), abs(encoderRight - lastRightEnc));

            old_x = x;
            old_y = y;

            resetInitEncoders();

            _state = Going_Forward;
        }
        else if (_state == Going_Forward)
        {
            // Условие выхода - мы подошли вплотную к точке
            if (sqrDistanceToTarget < 12 * 12)
            {
                _state = Stop;
            }
        }

        resetInitEncoders();
    }
}

void master_move_loop()
{
}