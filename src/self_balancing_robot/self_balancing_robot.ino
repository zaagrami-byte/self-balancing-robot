/*
 * Self-balancing robot firmware
 */

 #include <Arduino.h>
#include <Wire.h>
#include <math.h>

// =====================================================
//                  MPU6050
// =====================================================

#define MPU6050_ADDR 0x68

#define SDA_MPU 21
#define SCL_MPU 22

int16_t ax, ay, az;
int16_t gx, gy, gz;

float gyroOffsetX = 0.0;

// =====================================================
//                  L298N
// =====================================================

// Moteur gauche
#define ENA 25
#define IN1 26
#define IN2 27

// Moteur droit
#define ENB 13
#define IN3 14
#define IN4 12

// =====================================================
//                  PWM ESP32
// =====================================================

#define PWM_FREQUENCY 20000
#define PWM_RESOLUTION 8

#define PWM_MAX 230

// PWM minimum pour démarrer les moteurs
#define PWM_MIN 60

// =====================================================
//                     PID
// =====================================================

// Réglage initial
float Kp = 60.0;
float Ki = 5.0;
float Kd = 4.0;

// Erreur PID
float erreur = 0.0;
float erreurPrecedente = 0.0;

// Intégrale
float integrale = 0.0;

// Sortie PID
float pidOutput = 0.0;

// =====================================================
//              TERMES PID POUR PLOTTER
// =====================================================

float termeP = 0.0;
float termeI = 0.0;
float termeD = 0.0;

// =====================================================
//                  ANGLE
// =====================================================

float angleAccel = 0.0;
float angleFinal = 0.0;

float angleConsigne = 0.0;

// =====================================================
//                  TEMPS
// =====================================================

unsigned long previousMicros = 0;

float dt = 0.0;

// =====================================================
//              FILTRE COMPLEMENTAIRE
// =====================================================

#define ALPHA 0.99

// =====================================================
//                  SECURITE
// =====================================================

#define ANGLE_SECURITE 35.0

// =====================================================
//              INVERSION MOTEURS
// =====================================================

// false = sens normal
// true  = inversion du sens

bool INVERT_MOTORS = false;

// =====================================================
//              SERIAL PLOTTER
// =====================================================

// true = Serial Plotter
// false = affichage texte normal

#define SERIAL_PLOTTER true

// =====================================================
//              AFFICHAGE
// =====================================================

String dernierSens = "";

unsigned long dernierAffichage = 0;

// =====================================================
//                  PROTOTYPES
// =====================================================

void lireMPU6050();

float calculAngleAccel();

void calculAngle();

void calibrerGyroscope();

void calculPID();

void appliquerPID();

void moteurGauche(int vitesse);

void moteurDroit(int vitesse);

void stopMotors();

void afficherSensMoteur(int vitesse);

void afficherPIDPlotter();

// =====================================================
//                     SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    // =================================================
    // I2C
    // =================================================

    Wire.begin(
        SDA_MPU,
        SCL_MPU
    );

    Wire.setClock(400000);

    // =================================================
    // INITIALISATION MPU6050
    // =================================================

    Wire.beginTransmission(
        MPU6050_ADDR
    );

    Wire.write(0x6B);
    Wire.write(0x00);

    byte error =
        Wire.endTransmission();

    if (error != 0)
    {
        Serial.println(
            "ERREUR : MPU6050 NON DETECTE !"
        );

        while (1)
        {
            delay(1000);
        }
    }

    // =================================================
    // CONFIGURATION GYROSCOPE
    // =================================================

    // ±250 deg/s

    Wire.beginTransmission(
        MPU6050_ADDR
    );

    Wire.write(0x1B);
    Wire.write(0x00);

    Wire.endTransmission();

    // =================================================
    // CONFIGURATION ACCELEROMETRE
    // =================================================

    // ±2g

    Wire.beginTransmission(
        MPU6050_ADDR
    );

    Wire.write(0x1C);
    Wire.write(0x00);

    Wire.endTransmission();

    // =================================================
    // MOTEURS
    // =================================================

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // =================================================
    // PWM ESP32
    // =================================================

    ledcAttach(
        ENA,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );

    ledcAttach(
        ENB,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );

    // =================================================
    // ARRET MOTEURS
    // =================================================

    stopMotors();

    // =================================================
    // CALIBRATION GYROSCOPE
    // =================================================

    if (!SERIAL_PLOTTER)
    {
        Serial.println();
        Serial.println(
            "======================================"
        );

        Serial.println(
            "       ROBOT AUTO-EQUILIBRE"
        );

        Serial.println(
            " ESP32 + MPU6050 + PID + L298N"
        );

        Serial.println(
            "======================================"
        );

        Serial.println();

        Serial.println(
            "Calibration du gyroscope"
        );

        Serial.println(
            "NE BOUGEZ PAS LE ROBOT !"
        );
    }

    calibrerGyroscope();

    // =================================================
    // ANGLE INITIAL
    // =================================================

    lireMPU6050();

    angleAccel =
        calculAngleAccel();

    angleFinal =
        angleAccel;

    previousMicros =
        micros();

    if (!SERIAL_PLOTTER)
    {
        Serial.print(
            "Angle initial : "
        );

        Serial.print(
            angleFinal,
            2
        );

        Serial.println(
            " deg"
        );

        Serial.println();

        Serial.println(
            "======================================"
        );

        Serial.println(
            "             ROBOT PRET"
        );

        Serial.println(
            "======================================"
        );

        Serial.println();

        Serial.println(
            "PARAMETRES PID"
        );

        Serial.print("Kp = ");
        Serial.println(Kp);

        Serial.print("Ki = ");
        Serial.println(Ki);

        Serial.print("Kd = ");
        Serial.println(Kd);

        Serial.print("ALPHA = ");
        Serial.println(ALPHA);

        Serial.print("PWM_MAX = ");
        Serial.println(PWM_MAX);

        Serial.print("PWM_MIN = ");
        Serial.println(PWM_MIN);

        Serial.println();
    }
}

// =====================================================
//                     LOOP
// =====================================================

void loop()
{
    // =================================================
    // LECTURE MPU6050
    // =================================================

    lireMPU6050();

    // =================================================
    // CALCUL ANGLE
    // =================================================

    calculAngle();

    // =================================================
    // SECURITE
    // =================================================

    if (fabs(angleFinal) > ANGLE_SECURITE)
    {
        stopMotors();

        integrale = 0.0;

        erreurPrecedente = 0.0;

        termeP = 0.0;
        termeI = 0.0;
        termeD = 0.0;

        pidOutput = 0.0;

        dernierSens = "STOP";

        if (!SERIAL_PLOTTER)
        {
            Serial.println();
            Serial.println(
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            );

            Serial.println(
                "SECURITE : ROBOT TROP INCLINE !"
            );

            Serial.println(
                "MOTEURS ARRETES"
            );

            Serial.println(
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            );
        }

        delay(100);

        return;
    }

    // =================================================
    // CALCUL PID
    // =================================================

    calculPID();

    // =================================================
    // APPLICATION PID
    // =================================================

    appliquerPID();

    // =================================================
    // SERIAL PLOTTER
    // =================================================

    if (SERIAL_PLOTTER)
    {
        afficherPIDPlotter();
    }

    // =================================================
    // SERIAL NORMAL
    // =================================================

    else
    {
        if (
            millis() - dernierAffichage
            >= 200
        )
        {
            Serial.print("Angle: ");
            Serial.print(
                angleFinal,
                2
            );

            Serial.print(
                " | Consigne: "
            );

            Serial.print(
                angleConsigne,
                2
            );

            Serial.print(
                " | Erreur: "
            );

            Serial.print(
                erreur,
                2
            );

            Serial.print(
                " | P: "
            );

            Serial.print(
                termeP,
                2
            );

            Serial.print(
                " | I: "
            );

            Serial.print(
                termeI,
                2
            );

            Serial.print(
                " | D: "
            );

            Serial.print(
                termeD,
                2
            );

            Serial.print(
                " | PID: "
            );

            Serial.print(
                pidOutput,
                2
            );

            Serial.print(
                " | Sens: "
            );

            Serial.print(
                dernierSens
            );

            Serial.print(
                " | PWM: "
            );

            Serial.println(
                abs(
                    (int)pidOutput
                )
            );

            dernierAffichage =
                millis();
        }
    }

    delay(2);
}

// =====================================================
//              LECTURE MPU6050
// =====================================================

void lireMPU6050()
{
    Wire.beginTransmission(
        MPU6050_ADDR
    );

    Wire.write(0x3B);

    Wire.endTransmission(false);

    Wire.requestFrom(
        MPU6050_ADDR,
        14,
        true
    );

    if (Wire.available() >= 14)
    {
        // -----------------------------
        // ACCELEROMETRE X
        // -----------------------------

        ax =
            Wire.read() << 8;

        ax |= Wire.read();

        // -----------------------------
        // ACCELEROMETRE Y
        // -----------------------------

        ay =
            Wire.read() << 8;

        ay |= Wire.read();

        // -----------------------------
        // ACCELEROMETRE Z
        // -----------------------------

        az =
            Wire.read() << 8;

        az |= Wire.read();

        // -----------------------------
        // TEMPERATURE
        // -----------------------------

        Wire.read();
        Wire.read();

        // -----------------------------
        // GYROSCOPE X
        // -----------------------------

        gx =
            Wire.read() << 8;

        gx |= Wire.read();

        // -----------------------------
        // GYROSCOPE Y
        // -----------------------------

        gy =
            Wire.read() << 8;

        gy |= Wire.read();

        // -----------------------------
        // GYROSCOPE Z
        // -----------------------------

        gz =
            Wire.read() << 8;

        gz |= Wire.read();
    }
}

// =====================================================
//              ANGLE ACCELEROMETRE
// =====================================================

float calculAngleAccel()
{
    float angle;

    angle =
        atan2(
            (float)ay,
            (float)az
        );

    angle =
        angle * 180.0 / PI;

    return angle;
}

// =====================================================
//                  CALCUL ANGLE
// =====================================================

void calculAngle()
{
    unsigned long currentMicros =
        micros();

    dt =
        (currentMicros - previousMicros)
        / 1000000.0;

    previousMicros =
        currentMicros;

    // Protection dt

    if (
        dt <= 0.0 ||
        dt > 0.1
    )
    {
        dt = 0.005;
    }

    // =================================================
    // ANGLE ACCELEROMETRE
    // =================================================

    angleAccel =
        calculAngleAccel();

    // =================================================
    // GYROSCOPE
    // =================================================

    float gyroX =
        (
            (float)gx -
            gyroOffsetX
        )
        / 131.0;

    // =================================================
    // INTEGRATION GYROSCOPE
    // =================================================

    float angleGyro =
        angleFinal +
        gyroX * dt;

    // =================================================
    // FILTRE COMPLEMENTAIRE
    // =================================================

    angleFinal =
        ALPHA * angleGyro
        +
        (1.0 - ALPHA)
        * angleAccel;
}

// =====================================================
//             CALIBRATION GYROSCOPE
// =====================================================

void calibrerGyroscope()
{
    const int nombreEchantillons =
        1000;

    long somme = 0;

    for (
        int i = 0;
        i < nombreEchantillons;
        i++
    )
    {
        lireMPU6050();

        somme += gx;

        delay(2);
    }

    gyroOffsetX =
        (float)somme /
        nombreEchantillons;

    if (!SERIAL_PLOTTER)
    {
        Serial.print(
            "Offset gyro X = "
        );

        Serial.println(
            gyroOffsetX
        );
    }
}

// =====================================================
//                     PID
// =====================================================

void calculPID()
{
    // =================================================
    // ERREUR
    // =================================================

    erreur =
        angleConsigne -
        angleFinal;

    // =================================================
    // INTEGRALE
    // =================================================

    integrale +=
        erreur * dt;

    // =================================================
    // ANTI-WINDUP
    // =================================================

    // Limite volontairement faible
    // pour éviter la saturation

    integrale =
        constrain(
            integrale,
            -2.0,
            2.0
        );

    // =================================================
    // DERIVEE
    // =================================================

    float derivee = 0.0;

    if (dt > 0.0)
    {
        derivee =
            (
                erreur -
                erreurPrecedente
            )
            / dt;
    }

    // =================================================
    // TERMES PID
    // =================================================

    termeP =
        Kp * erreur;

    termeI =
        Ki * integrale;

    termeD =
        Kd * derivee;

    // =================================================
    // PID
    // =================================================

    pidOutput =
        termeP
        +
        termeI
        +
        termeD;

    // =================================================
    // LIMITATION PID
    // =================================================

    pidOutput =
        constrain(
            pidOutput,
            -PWM_MAX,
            PWM_MAX
        );

    // =================================================
    // MEMORISATION ERREUR
    // =================================================

    erreurPrecedente =
        erreur;
}

// =====================================================
//             APPLICATION PID
// =====================================================

void appliquerPID()
{
    int vitesse =
        (int)pidOutput;

    // =================================================
    // INVERSION MOTEURS
    // =================================================

    if (INVERT_MOTORS)
    {
        vitesse =
            -vitesse;
    }

    // =================================================
    // COMPENSATION DEMARRAGE
    // =================================================

    if (
        vitesse > 0 &&
        vitesse < PWM_MIN
    )
    {
        vitesse =
            PWM_MIN;
    }

    else if (
        vitesse < 0 &&
        vitesse > -PWM_MIN
    )
    {
        vitesse =
            -PWM_MIN;
    }

    // =================================================
    // MOTEURS
    // =================================================

    afficherSensMoteur(
        vitesse
    );

    moteurGauche(
        vitesse
    );

    moteurDroit(
        vitesse
    );
}

// =====================================================
//          AFFICHAGE SENS MOTEUR
// =====================================================

void afficherSensMoteur(
    int vitesse
)
{
    String sens;

    if (vitesse > 0)
    {
        sens = "AVANT";
    }

    else if (vitesse < 0)
    {
        sens = "ARRIERE";
    }

    else
    {
        sens = "STOP";
    }

    // En mode Plotter,
    // ne rien envoyer d'autre
    // que les données du graphe.

    if (SERIAL_PLOTTER)
    {
        dernierSens =
            sens;

        return;
    }

    if (sens != dernierSens)
    {
        Serial.print(
            ">>> Moteur : "
        );

        Serial.print(
            sens
        );

        Serial.print(
            " | PWM = "
        );

        Serial.println(
            abs(vitesse)
        );

        dernierSens =
            sens;
    }
}

// =====================================================
//                MOTEUR GAUCHE
// =====================================================

void moteurGauche(
    int vitesse
)
{
    vitesse =
        constrain(
            vitesse,
            -PWM_MAX,
            PWM_MAX
        );

    // =================================================
    // AVANT
    // =================================================

    if (vitesse > 0)
    {
        digitalWrite(
            IN1,
            HIGH
        );

        digitalWrite(
            IN2,
            LOW
        );

        ledcWrite(
            ENA,
            vitesse
        );
    }

    // =================================================
    // ARRIERE
    // =================================================

    else if (vitesse < 0)
    {
        digitalWrite(
            IN1,
            LOW
        );

        digitalWrite(
            IN2,
            HIGH
        );

        ledcWrite(
            ENA,
            -vitesse
        );
    }

    // =================================================
    // STOP
    // =================================================

    else
    {
        digitalWrite(
            IN1,
            LOW
        );

        digitalWrite(
            IN2,
            LOW
        );

        ledcWrite(
            ENA,
            0
        );
    }
}

// =====================================================
//                 MOTEUR DROIT
// =====================================================

void moteurDroit(
    int vitesse
)
{
    vitesse =
        constrain(
            vitesse,
            -PWM_MAX,
            PWM_MAX
        );

    // =================================================
    // AVANT
    // =================================================

    if (vitesse > 0)
    {
        digitalWrite(
            IN3,
            HIGH
        );

        digitalWrite(
            IN4,
            LOW
        );

        ledcWrite(
            ENB,
            vitesse
        );
    }

    // =================================================
    // ARRIERE
    // =================================================

    else if (vitesse < 0)
    {
        digitalWrite(
            IN3,
            LOW
        );

        digitalWrite(
            IN4,
            HIGH
        );

        ledcWrite(
            ENB,
            -vitesse
        );
    }

    // =================================================
    // STOP
    // =================================================

    else
    {
        digitalWrite(
            IN3,
            LOW
        );

        digitalWrite(
            IN4,
            LOW
        );

        ledcWrite(
            ENB,
            0
        );
    }
}

// =====================================================
//                  STOP MOTEURS
// =====================================================

void stopMotors()
{
    digitalWrite(
        IN1,
        LOW
    );

    digitalWrite(
        IN2,
        LOW
    );

    digitalWrite(
        IN3,
        LOW
    );

    digitalWrite(
        IN4,
        LOW
    );

    ledcWrite(
        ENA,
        0
    );

    ledcWrite(
        ENB,
        0
    );
}

// =====================================================
//              SERIAL PLOTTER
// =====================================================

void afficherPIDPlotter()
{
    // Angle réel
    Serial.print("Angle:");
    Serial.print(
        angleFinal,
        2
    );

    // Consigne
    Serial.print(
        "\tConsigne:"
    );

    Serial.print(
        angleConsigne,
        2
    );

    // Erreur
    Serial.print(
        "\tErreur:"
    );

    Serial.print(
        erreur,
        2
    );

    // Terme P
    Serial.print(
        "\tP:"
    );

    Serial.print(
        termeP,
        2
    );

    // Terme I
    Serial.print(
        "\tI:"
    );

    Serial.print(
        termeI,
        2
    );

    // Terme D
    Serial.print(
        "\tD:"
    );

    Serial.print(
        termeD,
        2
    );

    // Sortie PID
    Serial.print(
        "\tPID:"
    );

    Serial.println(
        pidOutput,
        2
    );
}
