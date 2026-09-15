// ============================================================
// CHOCADEIRA - CONTROLE V1.5
// ESP32-WROOM-32E
//
// V1.5
// - SHT31
// - DS3231 RTC
// - OLED SSD1306 128x64
// - Perfil CODORNA
// - Perfil GALINHA
// - Perfil PAVAO
// - Perfil PATO
// - Perfil CUSTOM
// - Fases de incubacao
// - Temperatura alvo por fase
// - Umidade alvo por fase
// - Histerese de umidade
// - Pulso da bomba
// - Lockout entre pulsos
// - RH maxima de seguranca
// - Calculo automatico do dia pelo DS3231
// - Inicio / parada / reset da incubacao
// - Configuracao CUSTOM via Serial
// - Inicio da incubacao com data/hora informada
//
// IMPORTANTE:
// Nesta V1.5:
//
// GPIO25 continua sendo a BOMBA.
//
// O aquecedor, ventilador e motor de viragem
// ainda NAO sao acionados fisicamente.
//
// A temperatura alvo e a viragem apenas fazem
// parte da configuracao da fase e ficam prontas
// para a proxima versao.
//
// ATENCAO:
// A data de inicio da incubacao ainda nao e salva
// permanentemente na NVS. Um reset/desligamento do
// ESP32 perde o estado da incubacao.
// O DS3231 continua mantendo a data/hora.
//
// ============================================================


// ============================================================
// BIBLIOTECAS
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>
#include <Preferences.h>


// ============================================================
// PINOS
// ============================================================

#define I2C_SDA 21
#define I2C_SCL 22

// Mantido GPIO25 para a bomba nesta V1.5.
#define PUMP_PIN 25


// ============================================================
// ENDERECOS I2C
// ============================================================

#define SHT31_ADDRESS 0x44
#define RTC_ADDRESS   0x68
#define OLED_ADDRESS  0x3C


// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1


// ============================================================
// OBJETOS
// ============================================================

Adafruit_SHT31 sht31 = Adafruit_SHT31();

RTC_DS3231 rtc;

Adafruit_SSD1306 display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    OLED_RESET
);

Preferences preferences;

// ============================================================
// STATUS DOS DISPOSITIVOS
// ============================================================

bool sht31Available = false;
bool rtcAvailable = false;
bool oledAvailable = false;


// ============================================================
// LIMITES
// ============================================================

const float MIN_RH_TARGET = 30.0;
const float MAX_RH_TARGET = 90.0;

const float MIN_RH_HYST = 0.5;
const float MAX_RH_HYST = 10.0;

const float MIN_RH_MAX = 60.0;
const float MAX_RH_MAX = 95.0;

const float MIN_TEMP = 30.0;
const float MAX_TEMP = 40.0;

const unsigned long MIN_PULSE = 100;
const unsigned long MAX_PULSE = 10000;

const unsigned long MIN_LOCKOUT = 5000;
const unsigned long MAX_LOCKOUT = 600000;


// ============================================================
// TEMPORIZACAO
// ============================================================

const unsigned long SENSOR_INTERVAL_MS = 2000;

// Atualizacao interna do OLED.
// Mantemos 500 ms para permitir atualizar
// rapidamente informacoes como tempo restante da bomba.
const unsigned long DISPLAY_INTERVAL_MS = 500;

// Troca automatica de tela a cada 5 segundos.
const unsigned long OLED_PAGE_INTERVAL_MS = 5000;

unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastOLEDPageChange = 0;

// Pagina atualmente exibida.
// 0 = Principal
// 1 = Controle RH
// 2 = Bomba
// 3 = Incubacao
uint8_t oledPage = 0;

// ============================================================
// ESTRUTURA DE FASE
// ============================================================

struct IncubationPhase
{
    uint16_t startDay;
    uint16_t endDay;

    float temperatureTarget;

    float rhTarget;
    float rhHysteresis;
    float rhMax;

    unsigned long pumpPulseMs;
    unsigned long pumpLockoutMs;

    bool turningEnabled;
};


// ============================================================
// ESTRUTURA DE PERFIL
// ============================================================

struct IncubationProfile
{
    const char* name;

    uint16_t totalDays;

    uint8_t phaseCount;

    const IncubationPhase* phases;
};

enum Profile
{
    PROFILE_GALINHA = 0,
    PROFILE_CODORNA = 1,
    PROFILE_PAVAO = 2,
    PROFILE_PATO = 3,
    PROFILE_CUSTOM = 4
};


// ============================================================
// PERFIL CODORNA
// ============================================================

const IncubationPhase quailPhases[] =
{
    {
        1,
        7,

        37.5,

        55.0,
        2.0,
        65.0,

        2000,
        60000,

        true
    },

    {
        8,
        14,

        37.5,

        50.0,
        2.0,
        80.0,

        2000,
        60000,

        true
    },

    {
        15,
        18,

        37.2,

        68.0,
        2.0,
        80.0,

        2000,
        60000,

        false
    }
};


// ============================================================
// PERFIL GALINHA
// ============================================================

const IncubationPhase chickenPhases[] =
{
    {
        1,
        7,

        37.5,

        55.0,
        2.0,
        65.0,

        2000,
        60000,

        true
    },

    {
        8,
        17,

        37.5,

        50.0,
        2.0,
        65.0,

        2000,
        60000,

        true
    },

    {
        18,
        21,

        37.2,

        68.0,
        2.0,
        80.0,

        2000,
        60000,

        false
    }
};


// ============================================================
// PERFIL PAVAO
// ============================================================

const IncubationPhase peacockPhases[] =
{
    {
        1,
        7,

        37.5,

        55.0,
        2.0,
        65.0,

        2000,
        60000,

        true
    },

    {
        8,
        25,

        37.5,

        50.0,
        2.0,
        65.0,

        2000,
        60000,

        true
    },

    {
        26,
        28,

        37.2,

        68.0,
        2.0,
        80.0,

        2000,
        60000,

        false
    }
};


// ============================================================
// PERFIL PATO
// ============================================================

const IncubationPhase duckPhases[] =
{
    {
        1,
        7,

        37.5,

        55.0,
        2.0,
        65.0,

        2000,
        60000,

        true
    },

    {
        8,
        25,

        37.5,

        50.0,
        2.0,
        65.0,

        2000,
        60000,

        true
    },

    {
        26,
        28,

        37.2,

        68.0,
        2.0,
        80.0,

        2000,
        60000,

        false
    }
};


// ============================================================
// PERFIS
// ============================================================

const IncubationProfile profileQuail =
{
    "CODORNA",
    18,
    sizeof(quailPhases) / sizeof(quailPhases[0]),
    quailPhases
};

const IncubationProfile profileChicken =
{
    "GALINHA",
    21,
    sizeof(chickenPhases) / sizeof(chickenPhases[0]),
    chickenPhases
};

const IncubationProfile profilePeacock =
{
    "PAVAO",
    28,
    sizeof(peacockPhases) / sizeof(peacockPhases[0]),
    peacockPhases
};

const IncubationProfile profileDuck =
{
    "PATO",
    28,
    sizeof(duckPhases) / sizeof(duckPhases[0]),
    duckPhases
};


// ============================================================
// PERFIL CUSTOMIZADO
// ============================================================

const uint8_t MAX_CUSTOM_PHASES = 8;

IncubationPhase customPhases[MAX_CUSTOM_PHASES];

IncubationProfile customProfile =
{
    "CUSTOM",
    21,
    0,
    customPhases
};


// ============================================================
// PERFIL ATUAL
// ============================================================

const IncubationProfile* currentProfile =
    &profileChicken;


// ============================================================
// CONTROLE DA INCUBACAO
// ============================================================

bool incubationActive = false;

bool postIncubationMode = false;

DateTime incubationStart;

int currentPhaseIndex = -1;


// ============================================================
// PARAMETROS DA FASE ATUAL
// ============================================================

float temperatureTarget = 37.5;

float rhTarget = 55.0;

float rhHysteresis = 2.0;

float rhMax = 70.0;

unsigned long pumpPulseMs = 2000;

unsigned long pumpLockoutMs = 60000;

bool turningEnabled = true;


// ============================================================
// LEITURAS
// ============================================================

float currentTemperature = NAN;

float currentHumidity = NAN;


// ============================================================
// BOMBA
// ============================================================

bool pumpState = false;

unsigned long currentPumpDuration = 0;

unsigned long pumpStartTime = 0;

unsigned long lockoutStartTime = 0;


// ============================================================
// CONTROLE AUTOMATICO
// ============================================================

bool automaticControl = true;


// ============================================================
// ESTADOS
// ============================================================

enum ControlState
{
    STATE_NORMAL,
    STATE_DOSING,
    STATE_LOCKOUT,
    STATE_HIGH_RH,
    STATE_SENSOR_ERROR
};

ControlState controlState = STATE_NORMAL;


// ============================================================
// NOME DO ESTADO
// ============================================================

const char* getStateName()
{
    switch (controlState)
    {
        case STATE_NORMAL:
            return "NORMAL";

        case STATE_DOSING:
            return "DOSANDO";

        case STATE_LOCKOUT:
            return "LOCKOUT";

        case STATE_HIGH_RH:
            return "RH ALTA";

        case STATE_SENSOR_ERROR:
            return "SENSOR ERRO";

        default:
            return "DESCONHECIDO";
    }
}


// ============================================================
// DATA/HORA SERIAL
// ============================================================

void printDateTimeSerial(const DateTime& dt)
{
    if (dt.day() < 10)
        Serial.print("0");

    Serial.print(dt.day());

    Serial.print("/");

    if (dt.month() < 10)
        Serial.print("0");

    Serial.print(dt.month());

    Serial.print("/");

    Serial.print(dt.year());

    Serial.print(" ");

    if (dt.hour() < 10)
        Serial.print("0");

    Serial.print(dt.hour());

    Serial.print(":");

    if (dt.minute() < 10)
        Serial.print("0");

    Serial.print(dt.minute());

    Serial.print(":");

    if (dt.second() < 10)
        Serial.print("0");

    Serial.println(dt.second());
}


// ============================================================
// CONVERTE TEXTO PARA DATETIME
// ============================================================
//
// Formato:
//
// DD/MM/YYYY HH:MM:SS
//
// Exemplo:
//
// 03/09/2026 08:00:00
//
// ============================================================

bool parseDateTime(
    const String& text,
    DateTime& result
)
{
    int day;
    int month;
    int year;

    int hour;
    int minute;
    int second;

    int n =
        sscanf(
            text.c_str(),
            "%d/%d/%d %d:%d:%d",
            &day,
            &month,
            &year,
            &hour,
            &minute,
            &second
        );

    if (n != 6)
        return false;

    if (year < 2000 || year > 2099)
        return false;

    if (month < 1 || month > 12)
        return false;

    if (day < 1 || day > 31)
        return false;

    if (hour < 0 || hour > 23)
        return false;

    if (minute < 0 || minute > 59)
        return false;

    if (second < 0 || second > 59)
        return false;

    result =
        DateTime(
            year,
            month,
            day,
            hour,
            minute,
            second
        );

    // Evita aceitar datas invalidas que o RTClib possa normalizar.
    if (result.year() != year ||
        result.month() != month ||
        result.day() != day ||
        result.hour() != hour ||
        result.minute() != minute ||
        result.second() != second)
    {
        return false;
    }

    return true;
}


// ============================================================
// DIA DA INCUBACAO
// ============================================================
//
// O instante em que "INCUBACAO INICIAR" foi executado
// corresponde ao DIA 1.
//
// Exemplo:
//
// Inicio: 03/09 09:00
//
// 03/09 09:00 -> Dia 1
// 04/09 09:00 -> Dia 2
// 05/09 09:00 -> Dia 3
//
// ============================================================

int getIncubationDay()
{
    if (!incubationActive)
        return 0;

    if (!rtcAvailable)
        return 0;

    DateTime now =
        rtc.now();

    int64_t elapsed =
        now.unixtime() -
        incubationStart.unixtime();

    if (elapsed < 0)
        return 0;

    return
        (int)(elapsed / 86400LL) + 1;
}


// ============================================================
// ENCONTRA FASE
// ============================================================

int findPhaseForDay(int day)
{
    if (day <= 0)
        return -1;

    for (
        uint8_t i = 0;
        i < currentProfile->phaseCount;
        i++
    )
    {
        const IncubationPhase& phase =
            currentProfile->phases[i];

        if (day >= phase.startDay &&
            day <= phase.endDay)
        {
            return i;
        }
    }

    return -1;
}


// ============================================================
// APLICA FASE
// ============================================================

void applyCurrentPhase()
{
    if (!incubationActive)
        return;

    int day =
        getIncubationDay();

    int newPhase =
        findPhaseForDay(day);

    if (newPhase < 0)
        return;

    if (newPhase == currentPhaseIndex)
        return;

    currentPhaseIndex =
        newPhase;

    const IncubationPhase& phase =
        currentProfile->phases[currentPhaseIndex];

    temperatureTarget =
        phase.temperatureTarget;

    rhTarget =
        phase.rhTarget;

    rhHysteresis =
        phase.rhHysteresis;

    rhMax =
        phase.rhMax;

    pumpPulseMs =
        phase.pumpPulseMs;

    pumpLockoutMs =
        phase.pumpLockoutMs;

    turningEnabled =
        phase.turningEnabled;

    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        "[INCUBACAO] NOVA FASE"
    );

    Serial.println(
        "========================================"
    );

    Serial.print("Perfil       : ");
    Serial.println(currentProfile->name);

    Serial.print("Dia          : ");
    Serial.print(day);

    Serial.print("/");
    Serial.println(currentProfile->totalDays);

    Serial.print("Fase         : ");
    Serial.print(currentPhaseIndex + 1);

    Serial.print("/");
    Serial.println(currentProfile->phaseCount);

    Serial.print("Temperatura  : ");
    Serial.print(temperatureTarget, 1);

    Serial.println(" C");

    Serial.print("RH alvo      : ");
    Serial.print(rhTarget, 1);

    Serial.println(" %");

    Serial.print("Histerese RH : ");
    Serial.print(rhHysteresis, 1);

    Serial.println(" %");

    Serial.print("RH maxima    : ");
    Serial.print(rhMax, 1);

    Serial.println(" %");

    Serial.print("Pulso bomba  : ");
    Serial.print(pumpPulseMs);

    Serial.println(" ms");

    Serial.print("Lockout      : ");
    Serial.print(pumpLockoutMs);

    Serial.println(" ms");

    Serial.print("Viragem      : ");

    if (turningEnabled)
        Serial.println("ON");
    else
        Serial.println("OFF");

    Serial.println(
        "========================================"
    );

    Serial.println();
}

// ============================================================
// RESTAURA PARAMETROS DA FASE 1
// ============================================================
//
// Utilizada quando a incubacao termina.
//
// O sistema deixa de estar em incubacao,
// mas os parametros permanecem preparados
// conforme a Fase 1 do perfil.
//
// ============================================================

void restorePhase1Parameters()
{
    if (currentProfile->phaseCount == 0)
    {
        Serial.println(
            "[ERRO] Perfil sem fases."
        );

        return;
    }

    const IncubationPhase& phase =
        currentProfile->phases[0];

    temperatureTarget =
        phase.temperatureTarget;

    rhTarget =
        phase.rhTarget;

    rhHysteresis =
        phase.rhHysteresis;

    rhMax =
        phase.rhMax;

    pumpPulseMs =
        phase.pumpPulseMs;

    pumpLockoutMs =
        phase.pumpLockoutMs;

    turningEnabled =
        phase.turningEnabled;

    Serial.println();
    Serial.println(
        "[INCUBACAO] Parametros restaurados para FASE 1."
    );

    Serial.print(
        "Temperatura: "
    );

    Serial.print(
        temperatureTarget,
        1
    );

    Serial.println(
        " C"
    );

    Serial.print(
        "RH alvo: "
    );

    Serial.print(
        rhTarget,
        1
    );

    Serial.println(
        " %"
    );

    Serial.print(
        "RH maxima: "
    );

    Serial.print(
        rhMax,
        1
    );

    Serial.println(
        " %"
    );

    Serial.print(
        "Pulso bomba: "
    );

    Serial.print(
        pumpPulseMs
    );

    Serial.println(
        " ms"
    );
}

void configurePostIncubation()
{
    temperatureTarget = 37.5;
    rhTarget = 55.0;
}

// ============================================================
// BOMBA OFF
// ============================================================

void pumpOff()
{
    digitalWrite(
        PUMP_PIN,
        LOW
    );

    if (pumpState)
    {
        unsigned long elapsed =
            millis() -
            pumpStartTime;

        Serial.print(
            "[BOMBA] OFF | Tempo: "
        );

        Serial.print(elapsed);

        Serial.println(" ms");
    }

    pumpState = false;
}


// ============================================================
// BOMBA ON
// ============================================================

void pumpOn(unsigned long duration = 0)
{
    if (pumpState)
        return;

    if (!isnan(currentHumidity) &&
        currentHumidity >= rhMax)
    {
        Serial.println(
            "[SEGURANCA] RH maxima atingida."
        );

        Serial.println(
            "[SEGURANCA] Bomba bloqueada."
        );

        return;
    }

    if (duration == 0)
        duration = pumpPulseMs;

    if (duration > MAX_PULSE)
        duration = MAX_PULSE;

    if (duration < MIN_PULSE)
        duration = MIN_PULSE;

    currentPumpDuration =
        duration;

    digitalWrite(
        PUMP_PIN,
        HIGH
    );

    pumpState = true;

    pumpStartTime =
        millis();

    controlState =
        STATE_DOSING;

    Serial.println();

    Serial.println(
        "[BOMBA] ON"
    );

    Serial.print(
        "[BOMBA] Pulso: "
    );

    Serial.print(
        currentPumpDuration
    );

    Serial.println(" ms");

    Serial.println();
}


// ============================================================
// ATUALIZA BOMBA
// ============================================================

void updatePump()
{
    if (!pumpState)
        return;

    unsigned long elapsed =
        millis() -
        pumpStartTime;

    if (elapsed >= MAX_PULSE)
    {
        Serial.println(
            "[SEGURANCA] TIMEOUT BOMBA!"
        );

        pumpOff();

        lockoutStartTime =
            millis();

        controlState =
            STATE_LOCKOUT;

        return;
    }

    if (elapsed >= currentPumpDuration)
    {
        pumpOff();

        lockoutStartTime =
            millis();

        controlState =
            STATE_LOCKOUT;

        Serial.print(
            "[CONTROLE] Lockout: "
        );

        Serial.print(
            pumpLockoutMs / 1000
        );

        Serial.println(" s");
    }
}


// ============================================================
// ATUALIZA LOCKOUT
// ============================================================

void updateLockout()
{
    if (controlState != STATE_LOCKOUT)
        return;

    unsigned long elapsed =
        millis() -
        lockoutStartTime;

    if (elapsed >= pumpLockoutMs)
    {
        controlState =
            STATE_NORMAL;

        Serial.println(
            "[CONTROLE] Lockout terminado."
        );
    }
}


// ============================================================
// LEITURA SHT31
// ============================================================

bool readSensor()
{
    if (!sht31Available)
    {
        currentTemperature = NAN;
        currentHumidity = NAN;

        return false;
    }

    float temperature =
        sht31.readTemperature();

    float humidity =
        sht31.readHumidity();

    if (isnan(temperature) ||
        isnan(humidity))
    {
        currentTemperature = NAN;
        currentHumidity = NAN;

        return false;
    }

    currentTemperature =
        temperature;

    currentHumidity =
        humidity;

    return true;
}


// ============================================================
// CONTROLE DE UMIDADE
// ============================================================

void humidityControl()
{
    if (!automaticControl)
        return;

    if (!incubationActive && !postIncubationMode)
        return;

    if (isnan(currentHumidity))
    {
        pumpOff();

        controlState =
            STATE_SENSOR_ERROR;

        return;
    }

    // --------------------------------------------------------
    // RH MAXIMA
    // --------------------------------------------------------

    if (currentHumidity >= rhMax)
    {
        if (pumpState)
            pumpOff();

        if (controlState != STATE_HIGH_RH)
        {
            Serial.println();

            Serial.println(
                "[ALERTA] UMIDADE ALTA!"
            );

            Serial.print("RH = ");

            Serial.print(
                currentHumidity,
                2
            );

            Serial.println(" %");

            Serial.println(
                "[SEGURANCA] Bomba OFF."
            );

            Serial.println();
        }

        controlState =
            STATE_HIGH_RH;

        return;
    }

    // --------------------------------------------------------
    // SAIDA DO ESTADO RH ALTA
    // --------------------------------------------------------

    if (controlState == STATE_HIGH_RH)
    {
        Serial.println(
            "[CONTROLE] RH voltou ao normal."
        );

        controlState =
            STATE_NORMAL;
    }

    if (pumpState)
        return;

    if (controlState == STATE_LOCKOUT)
        return;

    float lowerLimit =
        rhTarget -
        rhHysteresis;

    if (currentHumidity <= lowerLimit)
    {
        Serial.print(
            "[CONTROLE] RH = "
        );

        Serial.print(
            currentHumidity,
            2
        );

        Serial.print(
            " % | Limite = "
        );

        Serial.print(
            lowerLimit,
            2
        );

        Serial.println(" %");

        pumpOn();
    }
}


// ============================================================
// OLED - DATA/HORA
// ============================================================

void drawDateTime()
{
    if (!rtcAvailable)
    {
        display.print(
            "RTC: ERRO"
        );

        return;
    }

    DateTime now =
        rtc.now();

    if (now.day() < 10)
        display.print("0");

    display.print(now.day());

    display.print("/");

    if (now.month() < 10)
        display.print("0");

    display.print(now.month());

    display.print("/");

    display.print(now.year());

    display.print(" ");

    if (now.hour() < 10)
        display.print("0");

    display.print(now.hour());

    display.print(":");

    if (now.minute() < 10)
        display.print("0");

    display.print(now.minute());
}


// ============================================================
// OLED
//
// LAYOUT PRESERVADO DA V1.4-A
// ============================================================

// ============================================================
// OLED - TELA 1
// ============================================================
//
// Tela principal.
// O layout original da V1.4-A foi preservado.
//
// ============================================================

void drawOLEDPage1()
{
    display.clearDisplay();

    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    // --------------------------------------------------------
    // TITULO
    // --------------------------------------------------------

    display.setCursor(0, 0);
    display.println("CHOCADEIRA");

    // --------------------------------------------------------
    // DATA / HORA
    // --------------------------------------------------------

    display.setCursor(0, 8);
    drawDateTime();

    // --------------------------------------------------------
    // TEMPERATURA
    // --------------------------------------------------------

    display.setCursor(0, 24);
    display.print("TEMP: ");

    if (isnan(currentTemperature))
    {
        display.println("ERRO");
    }
    else
    {
        display.print(currentTemperature, 1);
        display.println(" C");
    }

    // --------------------------------------------------------
    // UMIDADE
    // --------------------------------------------------------

    display.setCursor(0, 34);
    display.print("RH  : ");

    if (isnan(currentHumidity))
    {
        display.println("ERRO");
    }
    else
    {
        display.print(currentHumidity, 1);
        display.println(" %");
    }

    // --------------------------------------------------------
    // ALVO
    // --------------------------------------------------------

    display.setCursor(0, 44);
    display.print("ALVO: ");
    display.print(rhTarget, 1);
    display.println(" %");

    // --------------------------------------------------------
    // BOMBA
    // --------------------------------------------------------

    display.setCursor(0, 54);
    display.print("BOMBA:");

    if (pumpState)
        display.print("ON");
    else
        display.print("OFF");

    display.display();
}


// ============================================================
// OLED - TELA 2
// ============================================================
//
// Informacoes do controle de umidade.
//
// ============================================================

void drawOLEDPage2()
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.println("CONTROLE RH");

    display.setCursor(0, 16);
    display.print("ATUAL: ");

    if (isnan(currentHumidity))
        display.println("ERRO");
    else
    {
        display.print(currentHumidity, 1);
        display.println(" %");
    }

    display.setCursor(0, 26);
    display.print("ALVO : ");
    display.print(rhTarget, 1);
    display.println(" %");

    display.setCursor(0, 36);
    display.print("MIN  : ");
    display.print(rhTarget - rhHysteresis, 1);
    display.println(" %");

    display.setCursor(0, 46);
    display.print("MAX  : ");
    display.print(rhMax, 1);
    display.println(" %");

    display.setCursor(0, 56);
    display.print("AUTO:");

    if (automaticControl)
        display.print(" ON");
    else
        display.print(" OFF");

    display.display();
}


// ============================================================
// OLED - TELA 3
// ============================================================
//
// Informacoes detalhadas da bomba.
//
// ============================================================

void drawOLEDPage3()
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.println("CONTROLE BOMBA");

    display.setCursor(0, 16);
    display.print("ESTADO: ");

    if (pumpState)
        display.println("ON");
    else
        display.println("OFF");

    display.setCursor(0, 26);
    display.print("AUTO  : ");

    if (automaticControl)
        display.println("ON");
    else
        display.println("OFF");

    display.setCursor(0, 36);
    display.print("PULSO : ");
    display.print(pumpPulseMs / 1000.0, 1);
    display.println(" s");

    display.setCursor(0, 46);

    if (pumpState)
    {
        unsigned long elapsed =
            millis() - pumpStartTime;

        long remaining =
            (long)currentPumpDuration -
            (long)elapsed;

        if (remaining < 0)
            remaining = 0;

        display.print("REST  : ");
        display.print(remaining / 1000.0, 1);
        display.println(" s");
    }
    else
    {
        display.print("LOCK  : ");

        if (controlState == STATE_LOCKOUT)
        {
            unsigned long elapsed =
                millis() - lockoutStartTime;

            long remaining =
                (long)pumpLockoutMs -
                (long)elapsed;

            if (remaining < 0)
                remaining = 0;

            display.print(remaining / 1000);
            display.println(" s");
        }
        else
        {
            display.println("0 s");
        }
    }

    display.setCursor(0, 56);
    display.print("VIRAGEM: ");

    if (turningEnabled)
        display.print("ON");
    else
        display.print("OFF");

    display.display();
}


// ============================================================
// OLED - TELA 4
// ============================================================
//
// Informacoes da incubacao.
//
// ============================================================

void drawOLEDPage4()
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.println("INCUBACAO");

    display.setCursor(0, 16);
    display.print("PERFIL: ");
    display.println(currentProfile->name);

    display.setCursor(0, 26);

    if (incubationActive)
    {
        int day = getIncubationDay();

        display.print("DIA: ");
        display.print(day);
        display.print(" / ");
        display.println(currentProfile->totalDays);
    }
    else
    {
        display.println("DIA: --");
    }

    display.setCursor(0, 36);
    display.print("FASE: ");

    if (incubationActive &&
        currentPhaseIndex >= 0)
    {
        display.print(currentPhaseIndex + 1);
        display.print(" / ");
        display.println(currentProfile->phaseCount);
    }
    else
    {
        display.println("--");
    }

    display.setCursor(0, 46);
    display.print("TEMP: ");
    display.print(temperatureTarget, 1);
    display.println(" C");

    display.setCursor(0, 56);
    display.print("RH  : ");
    display.print(rhTarget, 1);
    display.println(" %");

    display.display();
}

// ============================================================
// OLED - ATUALIZA PAGINA
// ============================================================
//
// Chama a tela correspondente a oledPage.
//
// ============================================================

void updateDisplay()
{
    if (!oledAvailable)
        return;

    switch (oledPage)
    {
        case 0:
            drawOLEDPage1();
            break;

        case 1:
            drawOLEDPage2();
            break;

        case 2:
            drawOLEDPage3();
            break;

        case 3:
            drawOLEDPage4();
            break;

        default:
            oledPage = 0;
            drawOLEDPage1();
            break;
    }
}


// ============================================================
// OLED - TROCA DE PAGINA
// ============================================================
//
// Troca automaticamente a cada 3 segundos.
//
// NAO utiliza delay(), portanto nao interfere no:
//
// - SHT31
// - bomba
// - lockout
// - RTC
// - incubacao
// - comandos Serial
//
// ============================================================

void updateOLEDPage()
{
    if (!oledAvailable)
        return;

    unsigned long now =
        millis();

    if (now - lastOLEDPageChange <
        OLED_PAGE_INTERVAL_MS)
    {
        return;
    }

    lastOLEDPageChange =
        now;

    oledPage++;

    if (oledPage >= 4)
    {
        oledPage = 0;
    }
}

// ============================================================
// STATUS
// ============================================================

void printStatus()
{
    Serial.println();

    Serial.println(
        "===== CHOCADEIRA V1.5 ====="
    );

    Serial.println();

    // --------------------------------------------------------
    // PERFIL
    // --------------------------------------------------------

    Serial.print(
        "Perfil: "
    );

    Serial.println(
        currentProfile->name
    );

    Serial.print(
        "Duracao: "
    );

    Serial.print(
        currentProfile->totalDays
    );

    Serial.println(
        " dias"
    );

    // --------------------------------------------------------
    // INCUBACAO
    // --------------------------------------------------------

    Serial.print(
        "Incubacao: "
    );

    Serial.println(
        incubationActive
        ? "ATIVA"
        : "PARADA"
    );

    if (incubationActive)
    {
        int day =
            getIncubationDay();

        Serial.print(
            "Dia: "
        );

        Serial.print(day);

        Serial.print("/");

        Serial.println(
            currentProfile->totalDays
        );

        Serial.print(
            "Fase: "
        );

        if (currentPhaseIndex >= 0)
        {
            Serial.print(
                currentPhaseIndex + 1
            );

            Serial.print("/");

            Serial.println(
                currentProfile->phaseCount
            );
        }
        else
        {
            Serial.println(
                "N/A"
            );
        }

        Serial.print(
            "Inicio: "
        );

        printDateTimeSerial(
            incubationStart
        );
    }

    Serial.println();

    // --------------------------------------------------------
    // RTC
    // --------------------------------------------------------

    Serial.println(
        "RTC:"
    );

    if (!rtcAvailable)
    {
        Serial.println(
            "ERRO"
        );
    }
    else
    {
        Serial.print(
            "Data/Hora: "
        );

        printDateTimeSerial(
            rtc.now()
        );
    }

    Serial.println();

    // --------------------------------------------------------
    // I2C
    // --------------------------------------------------------

    Serial.println(
        "I2C:"
    );

    Serial.print(
        "SHT31 0x44: "
    );

    Serial.println(
        sht31Available
        ? "OK"
        : "ERRO"
    );

    Serial.print(
        "DS3231 0x68: "
    );

    Serial.println(
        rtcAvailable
        ? "OK"
        : "ERRO"
    );

    Serial.print(
        "OLED 0x3C: "
    );

    Serial.println(
        oledAvailable
        ? "OK"
        : "ERRO"
    );

    Serial.println();

    // --------------------------------------------------------
    // SHT31
    // --------------------------------------------------------

    Serial.println(
        "SHT31:"
    );

    Serial.print(
        "Temperatura atual: "
    );

    if (isnan(currentTemperature))
    {
        Serial.println(
            "ERRO"
        );
    }
    else
    {
        Serial.print(
            currentTemperature,
            1
        );

        Serial.println(
            " C"
        );
    }

    Serial.print(
        "Umidade atual: "
    );

    if (isnan(currentHumidity))
    {
        Serial.println(
            "ERRO"
        );
    }
    else
    {
        Serial.print(
            currentHumidity,
            1
        );

        Serial.println(
            " %"
        );
    }

    Serial.println();

    // --------------------------------------------------------
    // PARAMETROS DA FASE
    // --------------------------------------------------------

    Serial.println(
        "FASE ATUAL:"
    );

    Serial.print(
        "Temperatura alvo: "
    );

    Serial.print(
        temperatureTarget,
        1
    );

    Serial.println(
        " C"
    );

    Serial.print(
        "RH alvo: "
    );

    Serial.print(
        rhTarget,
        1
    );

    Serial.println(
        " %"
    );

    Serial.print(
        "Histerese RH: "
    );

    Serial.print(
        rhHysteresis,
        1
    );

    Serial.println(
        " %"
    );

    Serial.print(
        "RH maxima: "
    );

    Serial.print(
        rhMax,
        1
    );

    Serial.println(
        " %"
    );

    Serial.print(
        "Pulso bomba: "
    );

    Serial.print(
        pumpPulseMs
    );

    Serial.println(
        " ms"
    );

    Serial.print(
        "Lockout: "
    );

    Serial.print(
        pumpLockoutMs
    );

    Serial.println(
        " ms"
    );

    Serial.print(
        "Viragem: "
    );

    Serial.println(
        turningEnabled
        ? "ON"
        : "OFF"
    );

    Serial.println();

    // --------------------------------------------------------
    // ESTADOS
    // --------------------------------------------------------

    Serial.print(
        "AUTO: "
    );

    Serial.println(
        automaticControl
        ? "ON"
        : "OFF"
    );

    Serial.print(
        "BOMBA: "
    );

    Serial.println(
        pumpState
        ? "ON"
        : "OFF"
    );

    Serial.print(
        "Estado: "
    );

    Serial.println(
        getStateName()
    );

    Serial.println();
}


// ============================================================
// LISTA DE PERFIS
// ============================================================

void printProfiles()
{
    Serial.println();

    Serial.println(
        "========== PERFIS =========="
    );

    Serial.println(
        "GALINHA  - 21 dias"
    );

    Serial.println(
        "CODORNA  - 18 dias"
    );

    Serial.println(
        "PAVAO    - 28 dias"
    );

    Serial.println(
        "PATO     - 28 dias"
    );

    Serial.println(
        "CUSTOM   - configuravel"
    );

    Serial.println(
        "============================"
    );

    Serial.println();
}


// ============================================================
// MOSTRA FASES DO PERFIL
// ============================================================

void printPhases()
{
    Serial.println();

    Serial.println(
        "========== FASES =========="
    );

    Serial.print(
        "Perfil: "
    );

    Serial.println(
        currentProfile->name
    );

    for (
        uint8_t i = 0;
        i < currentProfile->phaseCount;
        i++
    )
    {
        const IncubationPhase& p =
            currentProfile->phases[i];

        Serial.println();

        Serial.print(
            "Fase "
        );

        Serial.println(
            i + 1
        );

        Serial.print(
            "Dias: "
        );

        Serial.print(
            p.startDay
        );

        Serial.print(
            "-"
        );

        Serial.println(
            p.endDay
        );

        Serial.print(
            "Temperatura: "
        );

        Serial.print(
            p.temperatureTarget,
            1
        );

        Serial.println(
            " C"
        );

        Serial.print(
            "RH: "
        );

        Serial.print(
            p.rhTarget,
            1
        );

        Serial.println(
            " %"
        );

        Serial.print(
            "Histerese: "
        );

        Serial.print(
            p.rhHysteresis,
            1
        );

        Serial.println(
            " %"
        );

        Serial.print(
            "RH maxima: "
        );

        Serial.print(
            p.rhMax,
            1
        );

        Serial.println(
            " %"
        );

        Serial.print(
            "Pulso: "
        );

        Serial.print(
            p.pumpPulseMs
        );

        Serial.println(
            " ms"
        );

        Serial.print(
            "Lockout: "
        );

        Serial.print(
            p.pumpLockoutMs
        );

        Serial.println(
            " ms"
        );

        Serial.print(
            "Viragem: "
        );

        Serial.println(
            p.turningEnabled
            ? "ON"
            : "OFF"
        );
    }

    Serial.println();

    Serial.println(
        "============================"
    );

    Serial.println();
}


// ============================================================
// SELECIONA PERFIL
// ============================================================

bool selectProfile(
    const String& name
)
{
    if (incubationActive)
    {
        Serial.println(
            "[ERRO] Pare a incubacao antes de trocar o perfil."
        );

        return false;
    }

    if (name == "CODORNA")
    {
        currentProfile =
            &profileQuail;
    }
    else if (name == "GALINHA")
    {
        currentProfile =
            &profileChicken;
    }
    else if (name == "PAVAO")
    {
        currentProfile =
            &profilePeacock;
    }
    else if (name == "PATO")
    {
        currentProfile =
            &profileDuck;
    }
    else if (name == "CUSTOM")
    {
        currentProfile =
            &customProfile;
    }
    else
    {
        Serial.println(
            "[ERRO] Perfil desconhecido."
        );

        return false;
    }

    currentPhaseIndex =
        -1;

    Serial.print(
        "[PERFIL] Selecionado: "
    );

    Serial.println(
        currentProfile->name
    );

    Serial.print(
        "[PERFIL] Duracao: "
    );

    Serial.print(
        currentProfile->totalDays
    );

    Serial.println(
        " dias"
    );

    return true;
}


// ============================================================
// NVS - PERSISTENCIA DA INCUBACAO
// ============================================================
//
// Salva na memoria NVS do ESP32:
//
// - Incubacao ativa
// - Data/hora de inicio
// - Perfil utilizado
//
// Assim, uma queda de energia nao reinicia
// a contagem da incubacao.
//
// ============================================================

void saveIncubationState()
{
    preferences.begin("chocadeira", false);

    preferences.putBool(
        "active",
        incubationActive
    );

    if (incubationActive)
    {
        preferences.putULong64(
            "start",
            incubationStart.unixtime()
        );

        preferences.putString(
            "profile",
            currentProfile->name
        );
    }

    preferences.end();

    Serial.println(
        "[NVS] Estado da incubacao salvo."
    );
}


// ============================================================
// CARREGA INCUBACAO DA NVS
// ============================================================

bool loadIncubationState()
{
    preferences.begin(
        "chocadeira",
        true
    );

    bool active =
        preferences.getBool(
            "active",
            false
        );

    if (!active)
    {
        preferences.end();

        Serial.println(
            "[NVS] Nenhuma incubacao salva."
        );

        return false;
    }

    uint64_t startUnix =
        preferences.getULong64(
            "start",
            0
        );

    String profileName =
        preferences.getString(
            "profile",
            "GALINHA"
        );

    preferences.end();

    // --------------------------------------------------------
    // Verifica data salva
    // --------------------------------------------------------

    if (startUnix == 0)
    {
        Serial.println(
            "[NVS] Data de inicio invalida."
        );

        return false;
    }

    // --------------------------------------------------------
    // Recupera perfil
    // --------------------------------------------------------

    if (profileName == "CODORNA")
    {
        currentProfile =
            &profileQuail;
    }
    else if (profileName == "GALINHA")
    {
        currentProfile =
            &profileChicken;
    }
    else if (profileName == "PAVAO")
    {
        currentProfile =
            &profilePeacock;
    }
    else if (profileName == "PATO")
    {
        currentProfile =
            &profileDuck;
    }
    else if (profileName == "CUSTOM")
    {
        currentProfile =
            &customProfile;
    }
    else
    {
        Serial.println(
            "[NVS] Perfil salvo desconhecido."
        );

        return false;
    }

    // --------------------------------------------------------
    // Recupera data/hora
    // --------------------------------------------------------

    incubationStart =
        DateTime(
            (uint32_t)startUnix
        );

    incubationActive =
        true;

    postIncubationMode =
        false;

    currentPhaseIndex =
        -1;

    // --------------------------------------------------------
    // Verifica se o ciclo ja terminou
    // --------------------------------------------------------

    int day =
        getIncubationDay();

    if (day >
        currentProfile->totalDays)
    {
        Serial.println(
            "[NVS] Incubacao salva ja terminou."
        );

        incubationActive =
            false;

        postIncubationMode =
            true;

        currentPhaseIndex =
            -1;

        configurePostIncubation();

        saveIncubationState();

        return false;
    }

    // --------------------------------------------------------
    // Reaplica a fase correta
    // --------------------------------------------------------

    applyCurrentPhase();

    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "[NVS] INCUBACAO RECUPERADA"
    );

    Serial.println(
        "========================================"
    );

    Serial.print(
        "Perfil: "
    );

    Serial.println(
        currentProfile->name
    );

    Serial.print(
        "Inicio: "
    );

    printDateTimeSerial(
        incubationStart
    );

    Serial.print(
        "Dia: "
    );

    Serial.print(
        getIncubationDay()
    );

    Serial.print(
        "/"
    );

    Serial.println(
        currentProfile->totalDays
    );

    Serial.print(
        "Fase: "
    );

    Serial.print(
        currentPhaseIndex + 1
    );

    Serial.print(
        "/"
    );

    Serial.println(
        currentProfile->phaseCount
    );

    Serial.println(
        "========================================"
    );

    return true;
}

// ============================================================
// INICIA INCUBACAO
// ============================================================
//
// Formas:
//
// INCUBACAO INICIAR
//
//     Usa a data/hora atual do DS3231.
//
// INCUBACAO INICIAR 03/09/2026 08:00:00
//
//     Usa a data/hora informada.
//
// A data de inicio pode estar no passado,
// mas nunca pode estar no futuro.
//
// ============================================================

void startIncubation(
    const DateTime& start
)
{
    if (!rtcAvailable)
    {
        Serial.println(
            "[ERRO] RTC indisponivel."
        );

        return;
    }

    if (currentProfile->phaseCount == 0)
    {
        Serial.println(
            "[ERRO] Perfil sem fases configuradas."
        );

        return;
    }

    DateTime now =
        rtc.now();

    if (start.unixtime() > now.unixtime())
    {
        Serial.println(
            "[ERRO] Inicio da incubacao nao pode estar no futuro."
        );

        return;
    }

    incubationStart =
        start;

    incubationActive =
        true;

    postIncubationMode =
        false;

    currentPhaseIndex =
        -1;

    applyCurrentPhase();

    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        "       INCUBACAO INICIADA"
    );

    Serial.println(
        "========================================"
    );

    Serial.print(
        "Perfil       : "
    );

    Serial.println(
        currentProfile->name
    );

    Serial.print(
        "Inicio       : "
    );

    printDateTimeSerial(
        incubationStart
    );

    Serial.print(
        "Data atual   : "
    );

    printDateTimeSerial(
        now
    );

    Serial.print(
        "Dia da incubacao: "
    );

    Serial.print(
        getIncubationDay()
    );

    Serial.print(
        "/"
    );

    Serial.println(
        currentProfile->totalDays
    );

    Serial.println(
        "========================================"
    );

    Serial.println();
    saveIncubationState();
}


// ============================================================
// PARA INCUBACAO
// ============================================================

void stopIncubation()
{
    incubationActive = false;

    postIncubationMode = false;

    currentPhaseIndex = -1;

    pumpOff();

    controlState = STATE_NORMAL;

    // Volta os parametros para a Fase 1.
    restorePhase1Parameters();

    // Salva na NVS que nao existe mais
    // uma incubacao ativa.
    saveIncubationState();

    Serial.println(
        "[INCUBACAO] Parada."
    );
}

// ============================================================
// RESET INCUBACAO
// ============================================================

void resetIncubation()
{
    incubationActive = false;

    postIncubationMode = false;

    currentPhaseIndex = -1;

    pumpOff();

    controlState = STATE_NORMAL;

    // Retorna aos parametros da Fase 1.
    restorePhase1Parameters();

    // Remove o estado de incubacao da NVS.
    saveIncubationState();

    Serial.println(
        "[INCUBACAO] Resetada."
    );
}

// ============================================================
// CUSTOM CLEAR
// ============================================================

void clearCustomProfile()
{
    if (incubationActive)
    {
        Serial.println(
            "[ERRO] Pare a incubacao antes de configurar."
        );

        return;
    }

    customProfile.phaseCount =
        0;

    customProfile.totalDays =
        21;

    Serial.println(
        "[CUSTOM] Todas as fases foram removidas."
    );
}


// ============================================================
// CUSTOM TOTAL
// ============================================================

void configureCustomTotal(
    const String& command
)
{
    if (incubationActive)
    {
        Serial.println(
            "[ERRO] Pare a incubacao antes de configurar."
        );

        return;
    }

    int days =
        command.substring(13).toInt();

    if (days < 1 ||
        days > 365)
    {
        Serial.println(
            "[ERRO] Duracao invalida."
        );

        return;
    }

    customProfile.totalDays =
        days;

    Serial.print(
        "[CUSTOM] Duracao = "
    );

    Serial.print(
        days
    );

    Serial.println(
        " dias"
    );
}


// ============================================================
// CUSTOM PHASE
// ============================================================
//
// Formato:
//
// CUSTOM PHASE
// diaInicial
// diaFinal
// temperatura
// RH
// histerese
// pulso
// lockout
// RHmax
// viragem
//
// Exemplo:
//
// CUSTOM PHASE 1 10 37.5 55 2 2000 60000 70 1
//
// ============================================================

void configureCustomPhase(
    const String& command
)
{
    if (incubationActive)
    {
        Serial.println(
            "[ERRO] Pare a incubacao antes de configurar."
        );

        return;
    }

    if (customProfile.phaseCount >=
        MAX_CUSTOM_PHASES)
    {
        Serial.println(
            "[ERRO] Limite de 8 fases atingido."
        );

        return;
    }

    int startDay;
    int endDay;

    float temp;
    float rh;
    float hyst;

    unsigned long pulse;
    unsigned long lockout;

    float rhMaxValue;

    int turning;

    int result =
        sscanf(
            command.c_str(),

            "CUSTOM PHASE %d %d %f %f %f %lu %lu %f %d",

            &startDay,
            &endDay,

            &temp,
            &rh,
            &hyst,

            &pulse,
            &lockout,

            &rhMaxValue,

            &turning
        );

    if (result != 9)
    {
        Serial.println(
            "[ERRO] Formato invalido."
        );

        Serial.println(
            "Use:"
        );

        Serial.println(
            "CUSTOM PHASE diaIni diaFim temp RH hyst pulso lockout rhmax viragem"
        );

        return;
    }

    if (startDay < 1 ||
        endDay < startDay ||
        endDay > 365)
    {
        Serial.println(
            "[ERRO] Dias invalidos."
        );

        return;
    }

    if (temp < MIN_TEMP ||
        temp > MAX_TEMP)
    {
        Serial.println(
            "[ERRO] Temperatura invalida."
        );

        return;
    }

    if (rh < MIN_RH_TARGET ||
        rh > MAX_RH_TARGET)
    {
        Serial.println(
            "[ERRO] RH invalida."
        );

        return;
    }

    if (hyst < MIN_RH_HYST ||
        hyst > MAX_RH_HYST)
    {
        Serial.println(
            "[ERRO] Histerese invalida."
        );

        return;
    }

    if (rhMaxValue < MIN_RH_MAX ||
        rhMaxValue > MAX_RH_MAX)
    {
        Serial.println(
            "[ERRO] RH maxima invalida."
        );

        return;
    }

    if (pulse < MIN_PULSE ||
        pulse > MAX_PULSE)
    {
        Serial.println(
            "[ERRO] Pulso invalido."
        );

        return;
    }

    if (lockout < MIN_LOCKOUT ||
        lockout > MAX_LOCKOUT)
    {
        Serial.println(
            "[ERRO] Lockout invalido."
        );

        return;
    }

    if (rh + hyst >= rhMaxValue)
    {
        Serial.println(
            "[ERRO] RH alvo + histerese deve ficar abaixo da RH maxima."
        );

        return;
    }

    if (turning != 0 &&
        turning != 1)
    {
        Serial.println(
            "[ERRO] Viragem deve ser 0 ou 1."
        );

        return;
    }

    uint8_t index =
        customProfile.phaseCount;

    customPhases[index].startDay =
        startDay;

    customPhases[index].endDay =
        endDay;

    customPhases[index].temperatureTarget =
        temp;

    customPhases[index].rhTarget =
        rh;

    customPhases[index].rhHysteresis =
        hyst;

    customPhases[index].rhMax =
        rhMaxValue;

    customPhases[index].pumpPulseMs =
        pulse;

    customPhases[index].pumpLockoutMs =
        lockout;

    customPhases[index].turningEnabled =
        (turning == 1);

    customProfile.phaseCount++;

    if (endDay >
        customProfile.totalDays)
    {
        customProfile.totalDays =
            endDay;
    }

    Serial.print(
        "[CUSTOM] Fase "
    );

    Serial.print(
        index + 1
    );

    Serial.println(
        " adicionada."
    );
}


// ============================================================
// COMANDOS SERIAL
// ============================================================

void processSerialCommand()
{
    if (!Serial.available())
        return;

    String command =
        Serial.readStringUntil(
            '\n'
        );

    command.trim();

    command.toUpperCase();

    if (command.length() == 0)
        return;

    Serial.print(
        "[CMD] "
    );

    Serial.println(
        command
    );


    // ========================================================
    // STATUS
    // ========================================================

    if (command == "STATUS")
    {
        printStatus();
        return;
    }


    // ========================================================
    // HELP
    // ========================================================

    if (command == "HELP")
    {
        Serial.println();

        Serial.println(
            "========== COMANDOS V1.5 =========="
        );

        Serial.println();

        Serial.println("STATUS");

        Serial.println("PROFILES");

        Serial.println(
            "PROFILE CODORNA"
        );

        Serial.println(
            "PROFILE GALINHA"
        );

        Serial.println(
            "PROFILE PAVAO"
        );

        Serial.println(
            "PROFILE PATO"
        );

        Serial.println(
            "PROFILE CUSTOM"
        );

        Serial.println();

        Serial.println(
            "PHASES"
        );

        Serial.println();

        Serial.println(
            "INCUBACAO INICIAR"
        );

        Serial.println(
            "INCUBACAO INICIAR DD/MM/YYYY HH:MM:SS"
        );

        Serial.println(
            "INCUBACAO PARAR"
        );

        Serial.println(
            "INCUBACAO RESET"
        );

        Serial.println();

        Serial.println(
            "RTC"
        );

        Serial.println(
            "RTC SET DD/MM/YYYY HH:MM:SS"
        );

        Serial.println();

        Serial.println(
            "PUMP ON"
        );

        Serial.println(
            "PUMP OFF"
        );

        Serial.println(
            "PUMP 2000"
        );

        Serial.println();

        Serial.println(
            "AUTO ON"
        );

        Serial.println(
            "AUTO OFF"
        );

        Serial.println();

        Serial.println(
            "SET RH 55"
        );

        Serial.println(
            "SET HYST 2"
        );

        Serial.println(
            "SET PULSE 2000"
        );

        Serial.println(
            "SET LOCKOUT 60000"
        );

        Serial.println(
            "SET RHMAX 70"
        );

        Serial.println(
            "SET TEMP 37.5"
        );

        Serial.println();

        Serial.println(
            "CUSTOM TOTAL 30"
        );

        Serial.println(
            "CUSTOM CLEAR"
        );

        Serial.println(
            "CUSTOM PHASE 1 10 37.5 55 2 2000 60000 70 1"
        );

        Serial.println();

        Serial.println(
            "==================================="
        );

        Serial.println();

        return;
    }


    // ========================================================
    // PERFIS
    // ========================================================

    if (command == "PROFILES")
    {
        printProfiles();
        return;
    }


    if (command.startsWith(
            "PROFILE "
        ))
    {
        selectProfile(
            command.substring(8)
        );

        return;
    }


    // ========================================================
    // FASES
    // ========================================================

    if (command == "PHASES")
    {
        printPhases();
        return;
    }


    // ========================================================
    // INCUBACAO INICIAR COM DATA/HORA
    // ========================================================

    if (command.startsWith(
            "INCUBACAO INICIAR "
        ))
    {
        if (!rtcAvailable)
        {
            Serial.println(
                "[ERRO] RTC indisponivel."
            );

            return;
        }

        // "INCUBACAO INICIAR " = 18 caracteres

        String dateText =
            command.substring(18);

        DateTime start;

        if (!parseDateTime(
                dateText,
                start
            ))
        {
            Serial.println(
                "[ERRO] Data/hora invalida."
            );

            Serial.println(
                "Use:"
            );

            Serial.println(
                "INCUBACAO INICIAR DD/MM/YYYY HH:MM:SS"
            );

            return;
        }

        startIncubation(
            start
        );

        return;
    }


    // ========================================================
    // INCUBACAO INICIAR AGORA
    // ========================================================

    if (command == "INCUBACAO INICIAR")
    {
        if (!rtcAvailable)
        {
            Serial.println(
                "[ERRO] RTC indisponivel."
            );

            return;
        }

        DateTime start =
            rtc.now();

        startIncubation(
            start
        );

        return;
    }


    // ========================================================
    // INCUBACAO PARAR
    // ========================================================

    if (command == "INCUBACAO PARAR")
    {
        stopIncubation();
        return;
    }


    // ========================================================
    // INCUBACAO RESET
    // ========================================================

    if (command == "INCUBACAO RESET")
    {
        resetIncubation();
        return;
    }


    // ========================================================
    // RTC SET
    // ========================================================

    if (command.startsWith(
            "RTC SET "
        ))
    {
        if (!rtcAvailable)
        {
            Serial.println(
                "[ERRO] DS3231 nao disponivel."
            );

            return;
        }

        int day;
        int month;
        int year;

        int hour;
        int minute;
        int second;

        int result =
            sscanf(
                command.c_str(),

                "RTC SET %d/%d/%d %d:%d:%d",

                &day,
                &month,
                &year,

                &hour,
                &minute,
                &second
            );

        if (result != 6)
        {
            Serial.println(
                "[ERRO] Formato RTC invalido."
            );

            Serial.println(
                "Use: RTC SET DD/MM/YYYY HH:MM:SS"
            );

            return;
        }

        if (hour < 0 ||
            hour > 23 ||
            minute < 0 ||
            minute > 59 ||
            second < 0 ||
            second > 59)
        {
            Serial.println(
                "[ERRO] Hora invalida."
            );

            return;
        }

        if (month < 1 ||
            month > 12)
        {
            Serial.println(
                "[ERRO] Mes invalido."
            );

            return;
        }

        if (day < 1 ||
            day > 31)
        {
            Serial.println(
                "[ERRO] Dia invalido."
            );

            return;
        }

        if (year < 2000 ||
            year > 2099)
        {
            Serial.println(
                "[ERRO] Ano invalido."
            );

            return;
        }

        DateTime newDateTime(
            year,
            month,
            day,
            hour,
            minute,
            second
        );

        // Verifica se a data e realmente valida.
        if (newDateTime.year() != year ||
            newDateTime.month() != month ||
            newDateTime.day() != day ||
            newDateTime.hour() != hour ||
            newDateTime.minute() != minute ||
            newDateTime.second() != second)
        {
            Serial.println(
                "[ERRO] Data invalida."
            );

            return;
        }

        rtc.adjust(
            newDateTime
        );

        Serial.println(
            "[RTC] Data e hora configuradas."
        );

        Serial.print(
            "[RTC] "
        );

        printDateTimeSerial(
            newDateTime
        );

        return;
    }


    // ========================================================
    // RTC
    // ========================================================

    if (command == "RTC")
    {
        if (!rtcAvailable)
        {
            Serial.println(
                "[ERRO] DS3231 nao disponivel."
            );

            return;
        }

        Serial.print(
            "RTC: "
        );

        printDateTimeSerial(
            rtc.now()
        );

        return;
    }


    // ========================================================
    // PUMP ON
    // ========================================================

    if (command == "PUMP ON")
    {
        automaticControl =
            false;

        pumpOn();

        return;
    }


    // ========================================================
    // PUMP OFF
    // ========================================================

    if (command == "PUMP OFF")
    {
        pumpOff();

        automaticControl =
            false;

        controlState =
            STATE_NORMAL;

        return;
    }


    // ========================================================
    // PUMP TEMPO
    // ========================================================

    if (command.startsWith(
            "PUMP "
        ))
    {
        unsigned long duration =
            command.substring(5).toInt();

        if (duration == 0)
        {
            Serial.println(
                "[ERRO] Tempo invalido."
            );

            return;
        }

        automaticControl =
            false;

        pumpOn(
            duration
        );

        return;
    }


    // ========================================================
    // AUTO ON
    // ========================================================

    if (command == "AUTO ON")
    {
        automaticControl =
            true;

        controlState =
            STATE_NORMAL;

        Serial.println(
            "[CONTROLE] Automatico ON."
        );

        return;
    }


    // ========================================================
    // AUTO OFF
    // ========================================================

    if (command == "AUTO OFF")
    {
        automaticControl =
            false;

        pumpOff();

        controlState =
            STATE_NORMAL;

        Serial.println(
            "[CONTROLE] Automatico OFF."
        );

        return;
    }


    // ========================================================
    // SET RH
    // ========================================================

    if (command.startsWith(
            "SET RH "
        ))
    {
        float value =
            command.substring(7).toFloat();

        if (value < MIN_RH_TARGET ||
            value > MAX_RH_TARGET)
        {
            Serial.println(
                "[ERRO] RH fora do limite."
            );

            return;
        }

        rhTarget =
            value;

        Serial.print(
            "[CONFIG] RH alvo = "
        );

        Serial.print(
            rhTarget,
            1
        );

        Serial.println(
            " %"
        );

        return;
    }


    // ========================================================
    // SET HYST
    // ========================================================

    if (command.startsWith(
            "SET HYST "
        ))
    {
        float value =
            command.substring(9).toFloat();

        if (value < MIN_RH_HYST ||
            value > MAX_RH_HYST)
        {
            Serial.println(
                "[ERRO] Histerese invalida."
            );

            return;
        }

        rhHysteresis =
            value;

        Serial.print(
            "[CONFIG] Histerese = "
        );

        Serial.print(
            rhHysteresis,
            1
        );

        Serial.println(
            " %"
        );

        return;
    }


    // ========================================================
    // SET PULSE
    // ========================================================

    if (command.startsWith(
            "SET PULSE "
        ))
    {
        unsigned long value =
            command.substring(10).toInt();

        if (value < MIN_PULSE ||
            value > MAX_PULSE)
        {
            Serial.println(
                "[ERRO] Pulso invalido."
            );

            return;
        }

        pumpPulseMs =
            value;

        Serial.print(
            "[CONFIG] Pulso = "
        );

        Serial.print(
            pumpPulseMs
        );

        Serial.println(
            " ms"
        );

        return;
    }


    // ========================================================
    // SET LOCKOUT
    // ========================================================

    if (command.startsWith(
            "SET LOCKOUT "
        ))
    {
        unsigned long value =
            command.substring(12).toInt();

        if (value < MIN_LOCKOUT ||
            value > MAX_LOCKOUT)
        {
            Serial.println(
                "[ERRO] Lockout invalido."
            );

            return;
        }

        pumpLockoutMs =
            value;

        Serial.print(
            "[CONFIG] Lockout = "
        );

        Serial.print(
            pumpLockoutMs
        );

        Serial.println(
            " ms"
        );

        return;
    }


    // ========================================================
    // SET RHMAX
    // ========================================================

    if (command.startsWith(
            "SET RHMAX "
        ))
    {
        float value =
            command.substring(10).toFloat();

        if (value < MIN_RH_MAX ||
            value > MAX_RH_MAX)
        {
            Serial.println(
                "[ERRO] RHMAX invalido."
            );

            return;
        }

        rhMax =
            value;

        Serial.print(
            "[CONFIG] RH maxima = "
        );

        Serial.print(
            rhMax,
            1
        );

        Serial.println(
            " %"
        );

        return;
    }


    // ========================================================
    // SET TEMP
    // ========================================================

    if (command.startsWith(
            "SET TEMP "
        ))
    {
        float value =
            command.substring(9).toFloat();

        if (value < MIN_TEMP ||
            value > MAX_TEMP)
        {
            Serial.println(
                "[ERRO] Temperatura invalida."
            );

            return;
        }

        temperatureTarget =
            value;

        Serial.print(
            "[CONFIG] Temperatura alvo = "
        );

        Serial.print(
            temperatureTarget,
            1
        );

        Serial.println(
            " C"
        );

        return;
    }


    // ========================================================
    // CUSTOM TOTAL
    // ========================================================

    if (command.startsWith(
            "CUSTOM TOTAL "
        ))
    {
        configureCustomTotal(
            command
        );

        return;
    }


    // ========================================================
    // CUSTOM CLEAR
    // ========================================================

    if (command == "CUSTOM CLEAR")
    {
        clearCustomProfile();

        return;
    }


    // ========================================================
    // CUSTOM PHASE
    // ========================================================

    if (command.startsWith(
            "CUSTOM PHASE "
        ))
    {
        configureCustomPhase(
            command
        );

        return;
    }


    // ========================================================
    // COMANDO DESCONHECIDO
    // ========================================================

    Serial.println(
        "[ERRO] Comando desconhecido."
    );
}


// ============================================================
// MONITORAMENTO
// ============================================================

void printSensorData()
{
    Serial.println();

    Serial.println(
        "----------------------------------------"
    );

    Serial.print(
        "Perfil      : "
    );

    Serial.println(
        currentProfile->name
    );

    if (incubationActive)
    {
        int day =
            getIncubationDay();

        Serial.print(
            "Dia         : "
        );

        Serial.print(
            day
        );

        Serial.print(
            "/"
        );

        Serial.println(
            currentProfile->totalDays
        );
    }

    Serial.print(
        "Temperatura : "
    );

    if (isnan(currentTemperature))
    {
        Serial.println(
            "ERRO"
        );
    }
    else
    {
        Serial.print(
            currentTemperature,
            2
        );

        Serial.println(
            " C"
        );
    }

    Serial.print(
        "Umidade     : "
    );

    if (isnan(currentHumidity))
    {
        Serial.println(
            "ERRO"
        );
    }
    else
    {
        Serial.print(
            currentHumidity,
            2
        );

        Serial.println(
            " %"
        );
    }

    Serial.print(
        "Alvo RH     : "
    );

    Serial.print(
        rhTarget,
        2
    );

    Serial.println(
        " %"
    );

    Serial.print(
        "Alvo Temp   : "
    );

    Serial.print(
        temperatureTarget,
        2
    );

    Serial.println(
        " C"
    );

    Serial.print(
        "Bomba       : "
    );

    Serial.println(
        pumpState
        ? "ON"
        : "OFF"
    );

    Serial.print(
        "Viragem     : "
    );

    Serial.println(
        turningEnabled
        ? "ON"
        : "OFF"
    );

    Serial.print(
        "Estado      : "
    );

    Serial.println(
        getStateName()
    );

    Serial.println(
        "----------------------------------------"
    );

    Serial.println();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    // --------------------------------------------------------
    // SERIAL
    // --------------------------------------------------------

    Serial.begin(
        115200
    );

    delay(1000);

    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        "     CHOCADEIRA - CONTROLE V1.5"
    );

    Serial.println(
        "========================================"
    );


    // --------------------------------------------------------
    // BOMBA
    // --------------------------------------------------------

    pinMode(
        PUMP_PIN,
        OUTPUT
    );

    digitalWrite(
        PUMP_PIN,
        LOW
    );

    pumpState =
        false;

    Serial.print(
        "Pump GPIO = "
    );

    Serial.println(
        PUMP_PIN
    );


    // --------------------------------------------------------
    // I2C
    // --------------------------------------------------------

    Wire.begin(
        I2C_SDA,
        I2C_SCL
    );

    Wire.setClock(
        100000
    );

    Serial.println();

    Serial.println(
        "I2C inicializado."
    );

    Serial.print(
        "SDA = GPIO "
    );

    Serial.println(
        I2C_SDA
    );

    Serial.print(
        "SCL = GPIO "
    );

    Serial.println(
        I2C_SCL
    );


    // --------------------------------------------------------
    // SHT31
    // --------------------------------------------------------

    Serial.println();

    Serial.println(
        "Inicializando SHT31..."
    );

    if (sht31.begin(
            SHT31_ADDRESS
        ))
    {
        sht31Available =
            true;

        Serial.println(
            "[OK] SHT31 encontrado."
        );

        sht31.heater(
            false
        );

        Serial.println(
            "[OK] Aquecedor interno SHT31: OFF"
        );
    }
    else
    {
        sht31Available =
            false;

        Serial.println(
            "[ERRO] SHT31 nao encontrado!"
        );
    }


    // --------------------------------------------------------
    // DS3231
    // --------------------------------------------------------

    Serial.println();

    Serial.println(
        "Inicializando DS3231..."
    );

    if (rtc.begin())
    {
        rtcAvailable =
            true;

        Serial.println(
            "[OK] DS3231 encontrado."
        );

        if (rtc.lostPower())
        {
            Serial.println(
                "[AVISO] DS3231 perdeu alimentacao."
            );

            Serial.println(
                "[AVISO] Hora precisa ser configurada."
            );
        }
        else
        {
            Serial.println(
                "[OK] DS3231 com horario valido."
            );
        }
    }
    else
    {
        rtcAvailable =
            false;

        Serial.println(
            "[ERRO] DS3231 nao encontrado!"
        );
    }


    // --------------------------------------------------------
    // OLED
    // --------------------------------------------------------

    Serial.println();

    Serial.println(
        "Inicializando OLED..."
    );

    if (display.begin(
            SSD1306_SWITCHCAPVCC,
            OLED_ADDRESS
        ))
    {
        oledAvailable =
            true;

        Serial.println(
            "[OK] OLED SSD1306 encontrado."
        );

        display.clearDisplay();

        display.setTextColor(
            SSD1306_WHITE
        );

        display.setTextSize(
            1
        );

        display.setCursor(
            0,
            0
        );

        display.println(
            "CHOCADEIRA"
        );

        display.println();

        display.println(
            "V1.5"
        );

        display.println();

        display.println(
            "Inicializando..."
        );

        display.display();

        delay(1000);
    }
    else
    {
        oledAvailable =
            false;

        Serial.println(
            "[ERRO] OLED nao encontrado!"
        );
    }



    // --------------------------------------------------------
    // CONFIGURACAO
    // --------------------------------------------------------

    Serial.println();

    Serial.println(
        "========== CONFIGURACAO =========="
    );

    Serial.print(
        "Perfil        : "
    );

    Serial.println(
        currentProfile->name
    );

    Serial.print(
        "Duracao       : "
    );

    Serial.print(
        currentProfile->totalDays
    );

    Serial.println(
        " dias"
    );

    Serial.print(
        "RH alvo       : "
    );

    Serial.print(
        rhTarget,
        1
    );

    Serial.println(
        " %"
    );

    Serial.print(
        "Histerese     : "
    );

    Serial.print(
        rhHysteresis,
        1
    );

    Serial.println(
        " %"
    );

    Serial.print(
        "RH maxima     : "
    );

    Serial.print(
        rhMax,
        1
    );

    Serial.println(
        " %"
    );

    Serial.print(
        "Temp alvo     : "
    );

    Serial.print(
        temperatureTarget,
        1
    );

    Serial.println(
        " C"
    );

    Serial.print(
        "Pulso bomba   : "
    );

    Serial.print(
        pumpPulseMs
    );

    Serial.println(
        " ms"
    );

    Serial.print(
        "Lockout       : "
    );

    Serial.print(
        pumpLockoutMs / 1000
    );

    Serial.println(
        " s"
    );

    Serial.println(
        "=================================="
    );

    Serial.println();

    Serial.println(
        "[OK] Sistema pronto."
    );

    Serial.println(
        "[OK] Controle automatico disponivel."
    );

    Serial.println();

    Serial.println(
        "Digite HELP para comandos."
    );

Serial.println();

// --------------------------------------------------------
// RECUPERACAO DA INCUBACAO
// --------------------------------------------------------
//
// Se existir uma incubacao salva na NVS,
// recupera automaticamente apos uma queda
// de energia ou reset do ESP32.
//
// --------------------------------------------------------

if (rtcAvailable)
{
    loadIncubationState();
}

updateDisplay();
}



// ============================================================
// LOOP PRINCIPAL
// ============================================================

void loop()
{
    unsigned long now =
        millis();


    // --------------------------------------------------------
    // SERIAL
    // --------------------------------------------------------

    processSerialCommand();


    // --------------------------------------------------------
    // BOMBA
    // --------------------------------------------------------

    updatePump();


    // --------------------------------------------------------
    // LOCKOUT
    // --------------------------------------------------------

    updateLockout();


    // --------------------------------------------------------
    // INCUBACAO
    // --------------------------------------------------------

    if (incubationActive || postIncubationMode)
    {
        // Verifica se mudou de fase.
        applyCurrentPhase();

        int day =
            getIncubationDay();

        // Se terminou o ciclo,
        // encerra automaticamente.

        if (day >
    currentProfile->totalDays)
{
    Serial.println();

    Serial.println(
        "[INCUBACAO] FIM DO CICLO."
    );

    // --------------------------------------------------------
    // Encerra incubacao
    // --------------------------------------------------------

    incubationActive =
        false;

    currentPhaseIndex =
        -1;

    postIncubationMode =
        true;

    if (!pumpState)
        controlState = STATE_NORMAL;

    configurePostIncubation();

    // --------------------------------------------------------
    // Salva na NVS que o ciclo terminou.
    // --------------------------------------------------------

    saveIncubationState();

    Serial.println(
        "[INCUBACAO] Controle pos-incubacao ativo."
    );
}


    // --------------------------------------------------------
    // SENSOR
    // --------------------------------------------------------

    if (now -
        lastSensorRead >=
        SENSOR_INTERVAL_MS)
    {
        lastSensorRead =
            now;

        if (readSensor())
        {
            printSensorData();

            humidityControl();
        }
        else
        {
            Serial.println(
                "[ERRO] Falha SHT31."
            );

            pumpOff();

            controlState =
                STATE_SENSOR_ERROR;
        }
    }

// --------------------------------------------------------
// OLED - TROCA AUTOMATICA DE PAGINA
// --------------------------------------------------------

updateOLEDPage();


// --------------------------------------------------------
// OLED - ATUALIZACAO DA TELA
// --------------------------------------------------------

if (now -
    lastDisplayUpdate >=
    DISPLAY_INTERVAL_MS)
{
    lastDisplayUpdate =
        now;

    updateDisplay();
}
    
}
}