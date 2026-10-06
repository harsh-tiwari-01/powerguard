#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

#define DHT_PIN 15
#define MAINS_PIN 4 
#define VOLT_PIN 34
#define LED_CUT 2
#define LED_ALARM 5

#define M_TX 17
#define M_RX 18
#define S_RX 16
#define S_TX 19
#define SLAVE_ID 1
#define DEMO_TIME_SCALE 60.0f /
#define TEMP_LIMIT 8.0f
#define VOLT_LOW 180.0f
#define VOLT_HIGH 250.0f
#define BREACH_HOLD_MS 8000

#define TOPIC_BASE "powerguard/harsh-lko-7421/shop1"
#define TOPIC_TELE TOPIC_BASE "/telemetry"
#define TOPIC_ALERT TOPIC_BASE "/alert"
#define TOPIC_STATUS TOPIC_BASE "/status"
#define TOPIC_CMD TOPIC_BASE "/cmd"

#define USE_THERMAL_SIM 1 
#define AMBIENT_C 32.0f
#define TAU_WARM 390.0f 
#define TAU_COOL 40.0f  

DHT dht(DHT_PIN, DHT22);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

enum SysState
{
    ST_NORMAL,
    ST_POWER_CUT,
    ST_ALARM
};
const char *stateName[] = {"NORMAL", "POWER_CUT", "ALARM"};

struct Reading
{
    float temp, hum, volt, amps, power, kwh;
    bool mains;
    bool meterOk;
};
struct Outage
{
    uint32_t startMs;
    uint32_t durMs;
    float maxTemp;
};
struct Event
{
    char text[128];
};

QueueHandle_t readingQ, eventQ;
SemaphoreHandle_t dataMutex;
Reading latest = {25, 50, 220, 0, 0, 0, true, true};
volatile SysState sysState = ST_NORMAL;
Outage outageLog[10];
volatile int outageCount = 0;

volatile float gTemp = 25.0f;
volatile float meterV = 0, meterA = 0, meterP = 0, meterKwh = 0;
volatile bool meterOk = true;
volatile bool gCompOn = false, cmdFault = false;
volatile uint32_t doorUntil = 0;
volatile float aiSlope = 0; 
volatile int aiEta = -1, aiRisk = 0, predErr = 9999;
volatile bool aiFault = false;
uint16_t holdingRegs[4] = {0, 0, 0, 0}; 

void pushEvent(const char *type, const char *msg)
{
    Event e;
    snprintf(e.text, sizeof(e.text),
             "{\"type\":\"%s\",\"msg\":\"%s\",\"up_s\":%lu}", type, msg, millis() / 1000);
    xQueueSend(eventQ, &e, 0);
}

uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++)
    {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}
size_t addCrc(uint8_t *f, size_t len)
{
    uint16_t c = crc16(f, len);
    f[len++] = c & 0xFF;
    f[len++] = c >> 8;
    return len;
}

void handleRequest(uint8_t *r, size_t n)
{
    if (n < 8)
        return;
    if (crc16(r, n - 2) != (uint16_t)(r[n - 2] | (r[n - 1] << 8)))
        return;
    if (r[0] != SLAVE_ID)
        return;
    uint8_t out[32];
    size_t len = 0;
    uint16_t a = (r[2] << 8) | r[3];
    uint16_t b = (r[4] << 8) | r[5];
    out[len++] = SLAVE_ID;
    if (r[1] == 0x03)
    {
        if (b == 0 || a + b > 4)
        {
            out[len++] = 0x83;
            out[len++] = 0x02;
        }
        else
        {
            out[len++] = 0x03;
            out[len++] = b * 2;
            for (uint16_t i = 0; i < b; i++)
            {
                out[len++] = holdingRegs[a + i] >> 8;
                out[len++] = holdingRegs[a + i] & 0xFF;
            }
        }
    }
    else
    {
        out[len++] = r[1] | 0x80;
        out[len++] = 0x01;
    }
    len = addCrc(out, len);
    Serial2.write(out, len);
}

void slaveTask(void *)
{
    uint8_t buf[64];
    uint32_t lastCalc = millis();
    float whAcc = 0;
    for (;;)
    {
        uint32_t now = millis();
        float dtHours = (now - lastCalc) / 3600000.0f * DEMO_TIME_SCALE;
        lastCalc = now;

        bool mains = (digitalRead(MAINS_PIN) == LOW);
        float v = mains ? 100.0f + analogRead(VOLT_PIN) * 200.0f / 4095.0f : 0.0f;
        float a = mains ? ((gCompOn ? 2.2f : 0.2f) + random(-5, 6) / 100.0f) : 0.0f;
        float p = v * a * 0.9f; 
        whAcc += p * dtHours;

        holdingRegs[0] = (uint16_t)(v * 10);
        holdingRegs[1] = (uint16_t)(a * 100);
        holdingRegs[2] = (uint16_t)p;
        holdingRegs[3] = (uint16_t)(whAcc / 10.0f); 

        if (Serial2.available())
        {
            vTaskDelay(pdMS_TO_TICKS(15));
            size_t n = 0;
            while (Serial2.available() && n < sizeof(buf))
                buf[n++] = Serial2.read();
            handleRequest(buf, n);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

size_t transact(uint8_t *req, size_t reqLen, uint8_t *resp, size_t maxLen)
{
    while (Serial1.available())
        Serial1.read();
    Serial1.write(req, reqLen);
    size_t n = 0;
    uint32_t t0 = millis(), last = t0;
    while (millis() - t0 < 300)
    {
        if (Serial1.available())
        {
            resp[n++] = Serial1.read();
            last = millis();
            if (n >= maxLen)
                break;
        }
        else
        {
            if (n > 0 && millis() - last > 10)
                break;
            vTaskDelay(1);
        }
    }
    return n;
}

bool readRegs(uint8_t id, uint16_t start, uint16_t qty, uint16_t *out)
{
    uint8_t req[8] = {id, 0x03, (uint8_t)(start >> 8), (uint8_t)start,
                      (uint8_t)(qty >> 8), (uint8_t)qty};
    addCrc(req, 6);
    uint8_t resp[64];
    size_t n = transact(req, 8, resp, sizeof(resp));
    if (n == 0)
    {
        Serial.println("[MODBUS] Timeout");
        return false;
    }
    if (n < 5 || crc16(resp, n - 2) != (uint16_t)(resp[n - 2] | (resp[n - 1] << 8)))
    {
        Serial.println("[MODBUS] Bad CRC");
        return false;
    }
    if (resp[1] & 0x80)
    {
        Serial.printf("[MODBUS] Exception 0x%02X\n", resp[2]);
        return false;
    }
    for (uint16_t i = 0; i < qty; i++)
        out[i] = (resp[3 + i * 2] << 8) | resp[4 + i * 2];
    return true;
}

void modbusTask(void *)
{
    uint16_t v[4];
    int fails = 0;
    for (;;)
    {
        if (readRegs(SLAVE_ID, 0, 4, v))
        {
            meterV = v[0] / 10.0f;
            meterA = v[1] / 100.0f;
            meterP = v[2];
            meterKwh = v[3] / 100.0f;
            if (!meterOk)
            {
                meterOk = true;
                pushEvent("MODBUS", "Energy meter link restored");
            }
            fails = 0;
        }
        else if (++fails >= 3 && meterOk)
        {
            meterOk = false;
            pushEvent("MODBUS", "Energy meter not responding");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void sensorTask(void *)
{
    float temp = 4.5f, hum = 50.0f, simT = 4.5f;
    int tick = 0;
    for (;;)
    {
        if (tick % 4 == 0)
        {
            float t = dht.readTemperature(), h = dht.readHumidity();
            (void)t;
            if (!isnan(h))
                hum = h;
#if !USE_THERMAL_SIM
            if (!isnan(t))
                temp = t;
#endif
        }
        tick++;
        bool mains = (digitalRead(MAINS_PIN) == LOW);
#if USE_THERMAL_SIM
        if (simT > 6.0f)
            gCompOn = true;
        else if (simT < 4.0f)
            gCompOn = false;
        bool cooling = gCompOn && mains && !cmdFault;
        float heat = (AMBIENT_C - simT) / TAU_WARM;
        if (millis() < doorUntil)
            heat *= 3.0f;
        float cool = cooling ? simT / TAU_COOL : 0.0f;
        simT += (heat - cool) * 0.5f;
        temp = simT + random(-5, 6) / 100.0f;
#endif
        gTemp = temp;
        Reading r;
        r.temp = temp;
        r.hum = hum;
        r.volt = meterV;
        r.amps = meterA;
        r.power = meterP;
        r.kwh = meterKwh;
        r.meterOk = meterOk;
        r.mains = mains;
        xQueueSend(readingQ, &r, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void logicTask(void *)
{
    Reading r;
    bool prevMains = true;
    uint32_t cutStart = 0, breachStart = 0, lastPrint = 0;
    float maxT = -100.0f;
    bool voltWarned = false;
    char m[80];

    for (;;)
    {
        if (xQueueReceive(readingQ, &r, portMAX_DELAY) != pdTRUE)
            continue;
        uint32_t now = millis();

        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            latest = r;
            xSemaphoreGive(dataMutex);
        }

        if (prevMains && !r.mains)
        {
            cutStart = now;
            maxT = r.temp;
            Serial.println("[POWER] *** CUT detected ***");
            pushEvent("POWER_CUT", "Mains power lost");
        }
        if (!prevMains && r.mains)
        {
            Outage o = {cutStart, now - cutStart, maxT};
            outageLog[outageCount % 10] = o;
            outageCount++;
            Serial.printf("[POWER] Restored. Outage #%d lasted %.1f s, max temp %.1f C\n",
                          outageCount, o.durMs / 1000.0, o.maxTemp);
            snprintf(m, sizeof(m), "Restored after %.1f s, max temp %.1f C", o.durMs / 1000.0, o.maxTemp);
            pushEvent("POWER_RESTORED", m);
        }
        if (!r.mains && r.temp > maxT)
            maxT = r.temp;
        prevMains = r.mains;

        {
            static float ts[20], tv[20];
            static int cnt = 0, idx = 0, prevRisk = 0;
            static uint32_t predAt = 0, compOnAt = 0;
            static bool predSet = false, prevComp = false, pm = true;

            ts[idx] = now / 1000.0f;
            tv[idx] = r.temp;
            idx = (idx + 1) % 20;
            if (cnt < 20)
                cnt++;

            bool comp = gCompOn;
            if ((comp && !prevComp) || (r.mains && !pm))
                compOnAt = now;
            prevComp = comp;
            pm = r.mains;

            if (cnt >= 12)
            {
                int first = (idx + 20 - cnt) % 20;
                float t0 = ts[first], sx = 0, sy = 0, sxx = 0, sxy = 0;
                for (int i = 0; i < cnt; i++)
                {
                    int j = (first + i) % 20;
                    float x = ts[j] - t0, y = tv[j];
                    sx += x;
                    sy += y;
                    sxx += x * x;
                    sxy += x * y;
                }
                float den = cnt * sxx - sx * sx;
                float slope = den > 0 ? (cnt * sxy - sx * sy) / den : 0; // C per second
                aiSlope = slope * 60.0f;

                bool settled = r.mains ? true : (now - cutStart >= 10000);
                aiFault = r.mains && gCompOn && (now - compOnAt) > 20000 && slope > 0.01f && cnt >= 20;
                bool unprotected = !r.mains || aiFault;

                int eta = -1;
                if (unprotected && settled && slope > 0.003f && r.temp < TEMP_LIMIT)
                {
                    float tau = (AMBIENT_C - r.temp) / slope; // learned time constant
                    float ratio = (AMBIENT_C - r.temp) / (AMBIENT_C - TEMP_LIMIT);
                    if (ratio > 1.0f)
                        eta = (int)(tau * logf(ratio));
                }
                aiEta = eta;
                aiRisk = aiFault ? 2 : (eta < 0 ? 0 : (eta < 120 ? 2 : 1));

                if (eta >= 0 && !predSet)
                {
                    predAt = now + (uint32_t)eta * 1000UL;
                    predSet = true;
                    Serial.printf("[AI] Predicted: limit in %d s\n", eta);
                }
                if (predSet && r.temp >= TEMP_LIMIT)
                {
                    predErr = (int)((int32_t)(now - predAt) / 1000);
                    Serial.printf("[AI] Actual vs predicted crossing: %+d s\n", predErr);
                    predSet = false;
                }
                if (r.mains && !unprotected)
                    predSet = false;

                if (aiRisk == 2 && prevRisk < 2)
                {
                    if (aiFault)
                        snprintf(m, sizeof(m), "Compressor on but room warming: possible cooling fault");
                    else
                        snprintf(m, sizeof(m), "Predicted to cross limit in %d s", eta);
                    pushEvent("AI_WARNING", m);
                }
                prevRisk = aiRisk;
            }
        }

        if (r.temp > TEMP_LIMIT)
        {
            if (!breachStart)
                breachStart = now;
        }
        else
            breachStart = 0;
        bool breach = breachStart && (now - breachStart >= BREACH_HOLD_MS);

        SysState ns = breach ? ST_ALARM : (!r.mains ? ST_POWER_CUT : ST_NORMAL);
        if (ns != sysState)
        {
            Serial.printf("[STATE] %s -> %s\n", stateName[sysState], stateName[ns]);
            if (ns == ST_ALARM)
            {
                const char *msg = r.mains ? "Cooling fault: temp high with mains ON"
                                          : "Temp rising during power cut - save stock";
                Serial.printf("[ALERT] %s\n", msg);
                pushEvent("TEMP_ALARM", msg);
            }
            sysState = ns;
        }

        bool badV = r.mains && r.meterOk && r.volt > 20.0f && (r.volt < VOLT_LOW || r.volt > VOLT_HIGH);
        if (badV && !voltWarned)
        {
            Serial.printf("[VOLT] Warning: %.1f V out of safe range\n", r.volt);
            snprintf(m, sizeof(m), "Voltage %.1f V out of safe range", r.volt);
            pushEvent("VOLTAGE", m);
            voltWarned = true;
        }
        if (!badV)
            voltWarned = false;

        if (now - lastPrint >= 2000)
        {
            lastPrint = now;
            Serial.printf("[STATUS] %-9s mains=%s V=%.1f A=%.2f P=%.0fW kWh=%.2f T=%.1fC H=%.0f%% meter=%s slope=%.2fC/min eta=%d\n",
                          stateName[sysState], r.mains ? "ON " : "OFF", r.volt, r.amps, r.power,
                          r.kwh, r.temp, r.hum, r.meterOk ? "ok" : "FAIL", aiSlope, aiEta);
        }
    }
}

void alarmTask(void *)
{
    bool blink = false;
    for (;;)
    {
        switch (sysState)
        {
        case ST_NORMAL:
            digitalWrite(LED_CUT, LOW);
            digitalWrite(LED_ALARM, LOW);
            vTaskDelay(pdMS_TO_TICKS(200));
            break;
        case ST_POWER_CUT:
            blink = !blink;
            digitalWrite(LED_CUT, blink);
            digitalWrite(LED_ALARM, LOW);
            vTaskDelay(pdMS_TO_TICKS(500));
            break;
        case ST_ALARM:
            blink = !blink;
            digitalWrite(LED_ALARM, blink);
            digitalWrite(LED_CUT, LOW);
            vTaskDelay(pdMS_TO_TICKS(150));
            break;
        }
    }
}

void onCmd(char *topic, byte *payload, unsigned int len)
{
    String s;
    for (unsigned i = 0; i < len; i++)
        s += (char)payload[i];
    if (s == "door")
    {
        doorUntil = millis() + 15000;
        pushEvent("DOOR", "Door opened (simulated)");
    }
    else if (s == "fault")
    {
        cmdFault = true;
        pushEvent("TEST", "Compressor fault injected (simulated)");
    }
    else if (s == "clear")
    {
        cmdFault = false;
        doorUntil = 0;
        pushEvent("TEST", "Faults cleared");
    }
}

void mqttTask(void *)
{
    Serial.println("[WIFI] connecting...");
    WiFi.begin("Wokwi-GUEST", "", 6);
    while (WiFi.status() != WL_CONNECTED)
        vTaskDelay(pdMS_TO_TICKS(300));
    Serial.println("[WIFI] connected");

    mqtt.setServer("broker.hivemq.com", 1883);
    mqtt.setBufferSize(512);
    mqtt.setCallback(onCmd);
    uint32_t lastTele = 0;
    char buf[360];

    for (;;)
    {
        if (!mqtt.connected())
        {
            String cid = "pg-" + String((uint32_t)ESP.getEfuseMac(), HEX);
            if (mqtt.connect(cid.c_str(), TOPIC_STATUS, 1, true, "{\"online\":false}"))
            {
                mqtt.publish(TOPIC_STATUS, "{\"online\":true}", true);
                mqtt.subscribe(TOPIC_CMD);
                Serial.println("[MQTT] connected");
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
        }
        mqtt.loop();

        Event e;
        while (xQueueReceive(eventQ, &e, 0) == pdTRUE)
        {
            mqtt.publish(TOPIC_ALERT, e.text);
            Serial.printf("[MQTT] alert sent: %s\n", e.text);
        }

        if (millis() - lastTele >= 5000)
        {
            lastTele = millis();
            Reading r;
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                r = latest;
                xSemaphoreGive(dataMutex);
                snprintf(buf, sizeof(buf),
                         "{\"temp\":%.1f,\"hum\":%.0f,\"volt\":%.1f,\"amps\":%.2f,\"power_w\":%.0f,\"kwh\":%.2f,"
                         "\"meter\":%s,\"mains\":%s,\"state\":\"%s\",\"outages\":%d,"
                         "\"slope\":%.2f,\"eta_s\":%d,\"risk\":%d,\"ai_fault\":%s,\"pred_err\":%d,\"up_s\":%lu}",
                         r.temp, r.hum, r.volt, r.amps, r.power, r.kwh,
                         r.meterOk ? "true" : "false", r.mains ? "true" : "false",
                         stateName[sysState], outageCount,
                         aiSlope, aiEta, aiRisk, aiFault ? "true" : "false", predErr, millis() / 1000);
                mqtt.publish(TOPIC_TELE, buf);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(MAINS_PIN, INPUT_PULLUP);
    pinMode(LED_CUT, OUTPUT);
    pinMode(LED_ALARM, OUTPUT);
    dht.begin();
    Serial1.begin(9600, SERIAL_8N1, M_RX, M_TX); 
    Serial2.begin(9600, SERIAL_8N1, S_RX, S_TX); 
    readingQ = xQueueCreate(5, sizeof(Reading));
    eventQ = xQueueCreate(10, sizeof(Event));
    dataMutex = xSemaphoreCreateMutex();
    xTaskCreate(slaveTask, "meter", 4096, NULL, 2, NULL);
    xTaskCreate(modbusTask, "modbus", 4096, NULL, 2, NULL);
    xTaskCreate(sensorTask, "sensor", 4096, NULL, 2, NULL);
    xTaskCreate(logicTask, "logic", 4096, NULL, 2, NULL);
    xTaskCreate(alarmTask, "alarm", 2048, NULL, 1, NULL);
    xTaskCreate(mqttTask, "mqtt", 8192, NULL, 1, NULL);
    Serial.println("PowerGuard started");
}
void loop() { vTaskDelay(portMAX_DELAY); }