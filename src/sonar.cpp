#include "Sonar.h"

long duration;

uint8_t Sonar[] = {15, 2, 13, 12, 14, 27, 26, 25, 33, 32, 21, 34}; // pin_trig,pin_echo ESP32

int lastIndexes[6] = {0, 0, 0, 0, 0, 0};
bool isFull[6] = {0, 0, 0, 0, 0, 0};

#define US_OFFSET 18

unsigned long timers[6];

int us_cm[6][3];
int sonar_values[6];

uint16_t PERIOD;

void sonar_init(uint16_t period)
{
    PERIOD = period;

    for (int i = 0; i < sizeof(Sonar); i += 2)
    {
        pinMode(Sonar[i], OUTPUT);
        pinMode(Sonar[i + 1], INPUT);
    }

    for (int i = 0; i < 6; i++)
    {
        if (i > 0)
        {
            timers[i] = timers[i - 1] + US_OFFSET;
        }
        else
        {
            timers[i] = millis();
        }

        for (int j = 0; j < 3; j++)
        {
            us_cm[i][j] = 0;
        }
    }
}

float median_of_three(int a, int b, int c)
{
    if (((a <= b) && (b <= c)) || ((c <= b) && (b <= a)))
        return b;
    else if (((a <= c) && (c <= b)) || ((b <= c) && (c <= a)))
        return c;
    else if (((b <= a) && (a <= c)) || ((c <= a) && (a <= b)))
        return a;
    else
        return b;
}

int _readCmSonar(uint8_t sonar_num)
{
    uint8_t trig = Sonar[sonar_num * 2];
    uint8_t echo = Sonar[sonar_num * 2 + 1];

    // Сначала генерируем короткий импульс длительностью 2-5 микросекунд.
    digitalWrite(trig, LOW);
    delayMicroseconds(5);
    digitalWrite(trig, HIGH);
    // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    //  Время задержки акустического сигнала на эхолокаторе.
    duration = pulseIn(echo, HIGH, 18000); // 25 мс на прохождение луча туда обратно
    // Теперь осталось преобразовать время в расстояние

    return duration / 57;
}

int i = 0;

unsigned int lastPeakTime = 250;
int lastRight = 0;

int lastRightVals[6];

int getLastMin()
{
    int min = lastRightVals[0];

    for (int i = 1; i < 6; i++)
    {
        if (lastRightVals[i] < min) {
            min = lastRightVals[i];
        }
    }

    return min;
}

int getLastMax()
{
    int max = lastRightVals[0];

    for (int i = 1; i < 6; i++)
    {
        if (lastRightVals[i] > max) {
            max = lastRightVals[i];
        }
    }

    return max;
}

bool firstTime = true;

int getRightPeak(int minimum)
{
    int sonarRight = sonar_get(0);
    int delta = sonarRight - lastRight; // TODO: try to replace lastSonarRight with minimum
    lastRight = sonarRight;

    if (abs(delta) > MIN_WALL_THICKNESS && millis() - lastPeakTime > MAX_TIME_PASS_WALL)
    {
        // Purge first (from zero pos)
        if (firstTime) {
            firstTime = false;
            return 0;
        }

        lastPeakTime = millis();

        return delta;
    }

    return 0;
}

void sonar_loop()
{
    if (millis() >= timers[i])
    {
        timers[i] += PERIOD;

        int distance = _readCmSonar(i);
        int lastIndex = lastIndexes[i];

        // Out Of Range
        if (distance >= 200)
        {
            distance = 0;
        }

        // 30, 28, 27
        // 28, 27, -
        // Индекс двa => заполнили
        if (lastIndex == 2)
        {
            isFull[i] = true;
        }

        // Если заполнено - медиана
        if (isFull[i])
        {
            us_cm[i][0] = us_cm[i][1];
            us_cm[i][1] = us_cm[i][2];
            us_cm[i][2] = distance;

            sonar_values[i] = median_of_three(us_cm[i][0], us_cm[i][1], us_cm[i][2]);
        }
        else
        {
            us_cm[i][lastIndex] = distance;
            lastIndexes[i] = (lastIndex + 1) % 3;

            // return GetSonar(sonar_num);
            sonar_values[i] = distance;
        }
    }

    lastRightVals[0] = lastRightVals[1];
    lastRightVals[1] = lastRightVals[2];
    lastRightVals[2] = lastRightVals[3];
    lastRightVals[3] = lastRightVals[4];
    lastRightVals[4] = lastRightVals[5];
    lastRightVals[5] = sonar_get(0);

    i = (i + 1) % 6;
}

// Возвращает ПОСЛЕДНЮЮ ПОСЧИТАННУЮ дистанцию С ФИЛЬТРАЦИЕЙ. Нумерация @sonar_num справа налево от 0 до 5
int sonar_get(uint8_t sonar_num)
{
    return sonar_values[sonar_num];
}