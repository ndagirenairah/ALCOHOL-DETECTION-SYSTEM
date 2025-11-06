#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// TINYGSM CONFIGURATION
// ============================================================================
#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024
#define SerialMon Serial
#define SerialAT Serial1

#include <TinyGsmClient.h>

// ============================================================================
// HARDWARE PINS
// ============================================================================
// TTGO T-Call Modem
#define MODEM_RST       5
#define MODEM_PWKEY     4
#define MODEM_POWER_ON  23
#define MODEM_TX        27
#define MODEM_RX        26
#define I2C_SDA         21
#define I2C_SCL         22

// Peripherals
#define MQ3_PIN         34
#define BUZZER_PIN      13
#define LED_PIN         12

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
#define SAMPLES_FOR_BASELINE    20      // Take 20 samples for baseline
#define THRESHOLD_OFFSET        100     // Threshold = Baseline + 100
#define DEBOUNCE_COUNT          2       // Need 2 consistent readings

// Timing
#define READING_INTERVAL        3000    // 3 seconds
#define THINGSPEAK_INTERVAL     15000   // 15 seconds
#define BUZZ_COUNT              3
#define BUZZ_DURATION           150
#define BUZZ_PAUSE              150

// Credentials
const char PHONE_NUMBER[]       = "+256771858922";
const char THINGSPEAK_API_KEY[] = "DM05KQGCVCO64QDZ";
const char THINGSPEAK_APN[]     = "internet";

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
enum State {
    SAFE,
    DRUNK
};

State current_state = SAFE;
uint8_t state_counter = 0;
unsigned long last_reading = 0;
unsigned long last_upload = 0;

uint16_t baseline = 0;          // Calibrated baseline
uint16_t threshold = 0;         // Dynamic threshold

TinyGsm modem(SerialAT);

// ============================================================================
// BUZZER
// ============================================================================
void buzz(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(BUZZ_DURATION);
        digitalWrite(BUZZER_PIN, LOW);
        if (i < times - 1) delay(BUZZ_PAUSE);
    }
}

// ============================================================================
// SENSOR
// ============================================================================
uint16_t read_sensor() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 5; i++) {
        sum += analogRead(MQ3_PIN);
        delay(10);
    }
    return sum / 5;
}

// ============================================================================
// CALIBRATION
// ============================================================================
void calibrate_sensor() {
    SerialMon.println("\n=================================");
    SerialMon.println("CALIBRATING SENSOR");
    SerialMon.println("Keep sensor in CLEAN AIR");
    SerialMon.println("=================================\n");
    
    uint32_t sum = 0;
    uint16_t min_val = 4095;
    uint16_t max_val = 0;
    
    for (uint8_t i = 0; i < SAMPLES_FOR_BASELINE; i++) {
        uint16_t reading = read_sensor();
        sum += reading;
        
        if (reading < min_val) min_val = reading;
        if (reading > max_val) max_val = reading;
        
        SerialMon.print("Sample ");
        SerialMon.print(i + 1);
        SerialMon.print("/");
        SerialMon.print(SAMPLES_FOR_BASELINE);
        SerialMon.print(": ");
        SerialMon.println(reading);
        
        delay(500);
    }
    
    baseline = sum / SAMPLES_FOR_BASELINE;
    threshold = baseline + THRESHOLD_OFFSET;
    
    SerialMon.println("\n=================================");
    SerialMon.println("CALIBRATION COMPLETE");
    SerialMon.println("=================================");
    SerialMon.print("Baseline (avg):  ");
    SerialMon.println(baseline);
    SerialMon.print("Min reading:     ");
    SerialMon.println(min_val);
    SerialMon.print("Max reading:     ");
    SerialMon.println(max_val);
    SerialMon.print("Threshold:       ");
    SerialMon.println(threshold);
    SerialMon.print("Sensitivity:     +");
    SerialMon.println(THRESHOLD_OFFSET);
    SerialMon.println("=================================\n");
    
    if (baseline > 800) {
        SerialMon.println("WARNING: Baseline too high!");
        SerialMon.println("Check sensor or ventilation.");
    }
}

// ============================================================================
// SMS
// ============================================================================
void send_sms(const char* msg) {
    SerialMon.print("SMS: ");
    SerialMon.println(msg);
    modem.sendSMS(PHONE_NUMBER, msg);
}

// ============================================================================
// THINGSPEAK
// ============================================================================
void upload_data(uint16_t value, uint8_t status) {
    if (!modem.isGprsConnected()) return;
    
    TinyGsmClient client(modem);
    
    char url[200];
    snprintf(url, sizeof(url), "/update?api_key=%s&field1=%u&field2=%u",
             THINGSPEAK_API_KEY, value, status);
    
    if (client.connect("api.thingspeak.com", 80)) {
        client.print(String("GET ") + url + " HTTP/1.1\r\n");
        client.print("Host: api.thingspeak.com\r\n");
        client.print("Connection: close\r\n\r\n");
        delay(500);
        client.stop();
        SerialMon.println("ThingSpeak: OK");
    }
}

// ============================================================================
// MODEM INIT
// ============================================================================
void init_modem() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.beginTransmission(0x75);
    Wire.write(0x00); Wire.write(0x37);
    Wire.endTransmission();
    
    pinMode(MODEM_PWKEY, OUTPUT);
    pinMode(MODEM_RST, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);
    
    digitalWrite(MODEM_POWER_ON, HIGH);
    digitalWrite(MODEM_RST, HIGH);
    delay(100);
    digitalWrite(MODEM_RST, LOW);
    delay(100);
    digitalWrite(MODEM_RST, HIGH);
    delay(1000);
    
    digitalWrite(MODEM_PWKEY, LOW);
    delay(1200);
    digitalWrite(MODEM_PWKEY, HIGH);
    
    SerialMon.println("Modem: Booting...");
    delay(5000);
    
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    
    if (!modem.init()) {
        SerialMon.println("Modem: FAILED");
        return;
    }
    
    if (modem.getSimStatus() != 1) {
        SerialMon.println("SIM: FAILED");
        return;
    }
    
    if (!modem.waitForNetwork(60000L)) {
        SerialMon.println("Network: FAILED");
        return;
    }
    
    if (!modem.gprsConnect(THINGSPEAK_APN, "", "")) {
        SerialMon.println("GPRS: FAILED");
        return;
    }
    
    SerialMon.println("Modem: READY");
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    SerialMon.begin(115200);
    delay(1000);
    
    SerialMon.println("\n=================================");
    SerialMon.println("ALCOHOL DETECTION SYSTEM");
    SerialMon.println("=================================\n");
    
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(MQ3_PIN, INPUT);
    
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    
    analogReadResolution(10);
    analogSetAttenuation(ADC_11db);
    
    SerialMon.println("Hardware: OK\n");
    
    calibrate_sensor();
    
    init_modem();
    
    char startup_msg[160];
    snprintf(startup_msg, sizeof(startup_msg), 
             "System Started. Baseline: %u, Threshold: %u", 
             baseline, threshold);
    send_sms(startup_msg);
    upload_data(baseline, 0);
    
    SerialMon.println("System: READY\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    unsigned long now = millis();
    
    // Read sensor every READING_INTERVAL
    if (now - last_reading < READING_INTERVAL) return;
    last_reading = now;
    
    uint16_t level = read_sensor();
    int16_t delta = level - baseline;
    bool is_drunk = (level > threshold);
    
    SerialMon.print("Level: ");
    SerialMon.print(level);
    SerialMon.print(" | Baseline: ");
    SerialMon.print(baseline);
    SerialMon.print(" | Delta: ");
    if (delta >= 0) SerialMon.print("+");
    SerialMon.print(delta);
    SerialMon.print(" | Threshold: ");
    SerialMon.print(threshold);
    SerialMon.print(" | Status: ");
    SerialMon.print(current_state == DRUNK ? "DRUNK" : "SAFE");
    SerialMon.print(" | Counter: ");
    SerialMon.print(state_counter);
    SerialMon.print(" | LED: ");
    SerialMon.println(current_state == DRUNK ? "ON" : "OFF");
    
    // ========================================================================
    // STATE MACHINE
    // ========================================================================
    if (current_state == SAFE && is_drunk) {
        state_counter++;
        if (state_counter >= DEBOUNCE_COUNT) {
            // TRANSITION: SAFE → DRUNK
            current_state = DRUNK;
            state_counter = 0;
            
            SerialMon.println("\n>>> ALCOHOL DETECTED <<<");
            SerialMon.print("Trigger level: ");
            SerialMon.print(level);
            SerialMon.print(" (");
            SerialMon.print(delta);
            SerialMon.println(" above baseline)");
            SerialMon.println("LED: TURNED ON (will stay on until sober)\n");
            
            // Turn LED ON and keep it on
            digitalWrite(LED_PIN, HIGH);
            
            // Buzz as alert
            buzz(BUZZ_COUNT);
            
            char msg[160];
            snprintf(msg, sizeof(msg), 
                     "ALERT! Alcohol content high!", 
                     level, baseline, delta);
            send_sms(msg);
        }
    }
    else if (current_state == DRUNK && !is_drunk) {
        state_counter++;
        if (state_counter >= DEBOUNCE_COUNT) {
            // TRANSITION: DRUNK → SAFE
            current_state = SAFE;
            state_counter = 0;
            
            SerialMon.println("\n>>> DRIVER SOBER <<<");
            SerialMon.println("LED: TURNED OFF\n");
            
            // Turn LED OFF now that driver is sober
            digitalWrite(LED_PIN, LOW);
            
            char msg[160];
            snprintf(msg, sizeof(msg), 
                     "Driver sober! Alcohol levels Low!", 
                     level, baseline);
            send_sms(msg);
        }
    }
    else {
        state_counter = 0;
    }
    
    // ========================================================================
    // LED STATE MAINTENANCE (Keep LED on while DRUNK)
    // ========================================================================
    // This ensures LED stays on continuously during DRUNK state
    if (current_state == DRUNK) {
        digitalWrite(LED_PIN, HIGH);  // Keep LED on
    } else {
        digitalWrite(LED_PIN, LOW);   // Keep LED off
    }
    
    // ========================================================================
    // THINGSPEAK UPLOAD
    // ========================================================================
    if (now - last_upload >= THINGSPEAK_INTERVAL) {
        uint8_t status = (current_state == DRUNK) ? 1 : 0;
        upload_data(level, status);
        last_upload = now;
    }
    
    SerialMon.println("---------------------------------------------------------------------------------------------------------");
}