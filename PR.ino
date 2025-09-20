#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>

const char* ssid = "";
const char* password = "";

// Mathematical constants for CSI calculations
#define PI 3.14159265359
#define LIGHT_SPEED 299792458.0           
#define WIFI_CENTER_FREQ 2412000000.0     
#define SUBCARRIER_SPACING 312500.0       
#define MAX_SUBCARRIERS 64
#define BASELINE_SAMPLES 50

// Volumetric radar data structure
typedef struct {
    float amplitude_baseline[MAX_SUBCARRIERS];
    float phase_baseline[MAX_SUBCARRIERS];
    float amplitude_variance[MAX_SUBCARRIERS];
    float phase_variance[MAX_SUBCARRIERS];
    float distance_estimates[MAX_SUBCARRIERS];
    float doppler_shift[MAX_SUBCARRIERS];
    bool baseline_calibrated;
    int sample_count;
    unsigned long last_timestamp;
} volumetric_radar_t;

volumetric_radar_t radar = {0};

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    Serial.println("ESP32-S3 WiFi Connection Status 6 Fix");
    
    // Critical fixes for WL_CONNECT_FAILED (Status 6)
    fixWiFiConnectionStatus6();
}

void fixWiFiConnectionStatus6() {
    Serial.println("Applying Status 6 fixes...");
    
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(2000);
    
    esp_wifi_restore();
    delay(1000);
    
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    Serial.println("Scanning for network...");
    int n = WiFi.scanNetworks();
    bool networkFound = false;
    int networkChannel = 0;
    
    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) == String(ssid)) {
            networkFound = true;
            networkChannel = WiFi.channel(i);
            Serial.printf("Found network: %s\n", ssid);
            Serial.printf("Channel: %d, RSSI: %d dBm\n", networkChannel, WiFi.RSSI(i));
            Serial.printf("Encryption: %d\n", WiFi.encryptionType(i));
            break;
        }
    }
    
    if (!networkFound) {
        Serial.println("ERROR: Network not found!");
        Serial.println("Available networks:");
        for (int i = 0; i < min(n, 10); i++) {
            Serial.printf("  %s (RSSI: %d)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
        }
        return;
    }
    
    if (networkChannel > 0) {
        esp_wifi_set_channel(networkChannel, WIFI_SECOND_CHAN_NONE);
        Serial.printf("Set to channel %d\n", networkChannel);
    }
    
    Serial.printf("Password length: %d characters\n", strlen(password));
    Serial.println("Attempting connection...");
    WiFi.begin(ssid, password);
    
    unsigned long startTime = millis();
    int attempts = 0;
    
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < 30000) {
        attempts++;
        delay(2000);
        
        wl_status_t status = WiFi.status();
        Serial.printf("Attempt %d: Status = %d ", attempts, status);
        
        switch(status) {
            case WL_IDLE_STATUS:     Serial.println("(Idle)"); break;
            case WL_NO_SSID_AVAIL:   Serial.println("(No SSID)"); break;
            case WL_SCAN_COMPLETED:  Serial.println("(Scan complete)"); break;
            case WL_CONNECTED:       Serial.println("(Connected)"); break;
            case WL_CONNECT_FAILED:  Serial.println("(Connect failed)"); break;
            case WL_CONNECTION_LOST: Serial.println("(Connection lost)"); break;
            case WL_DISCONNECTED:    Serial.println("(Disconnected)"); break;
            default:                 Serial.printf("(Unknown: %d)\n", status); break;
        }
        
        if (status == WL_CONNECT_FAILED) {
            Serial.println("Status 6 detected - trying password fixes...");
            tryPasswordVariations();
            return;
        }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n=== CONNECTION SUCCESSFUL ===");
        Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
        Serial.println("============================");
        
        // Initialize complete CSI radar system
        initializeCompleteCSI();
    } else {
        Serial.println("Connection failed - trying alternative methods...");
        tryAlternativeMethods();
    }
}

void initializeCompleteCSI() {
    Serial.println("Initializing complete CSI radar system...");
    
    // Configure CSI with full mathematical precision
    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = 0,
        .shift = 0,
    };
    
    esp_err_t ret = esp_wifi_set_csi_config(&csi_config);
    if (ret == ESP_OK) {
        // Set the callback function
        esp_wifi_set_csi_rx_cb(&csi_math_callback, NULL);
        esp_wifi_set_csi(true);
        
        // Initialize radar data structure
        radar.baseline_calibrated = false;
        radar.sample_count = 0;
        radar.last_timestamp = micros();
        
        // Clear all arrays
        for (int i = 0; i < MAX_SUBCARRIERS; i++) {
            radar.amplitude_baseline[i] = 0.0;
            radar.phase_baseline[i] = 0.0;
            radar.amplitude_variance[i] = 0.0;
            radar.phase_variance[i] = 0.0;
            radar.distance_estimates[i] = 0.0;
            radar.doppler_shift[i] = 0.0;
        }
        
        Serial.println("CSI Mathematical Engine Initialized Successfully!");
        Serial.println("Calibrating baseline - please remain stationary for 10 seconds...");
    } else {
        Serial.printf("CSI initialization failed with error: %d\n", ret);
        Serial.println("Continuing with basic WiFi monitoring...");
    }
}

void ARDUINO_ISR_ATTR csi_math_callback(void *ctx, wifi_csi_info_t *data) {
    if (!data || !data->buf || data->len < 4) return;
    
    // Process the CSI data
    processVolumetricMath(data);
}

void processVolumetricMath(wifi_csi_info_t *data) {
    int8_t *csi_raw = data->buf;
    int num_subcarriers = min(data->len / 2, MAX_SUBCARRIERS);
    unsigned long current_time = micros();
    float time_delta = (current_time - radar.last_timestamp) / 1000000.0;
    
    // Process each subcarrier
    for (int i = 0; i < num_subcarriers; i++) {
        // Extract I/Q components
        int8_t I = csi_raw[i * 2 + 1];     // Real component
        int8_t Q = csi_raw[i * 2];         // Imaginary component
        
        // Calculate amplitude and phase
        float amplitude = sqrt(I * I + Q * Q);
        float phase = atan2(Q, I);
        
        // Baseline calibration phase
        if (radar.sample_count < BASELINE_SAMPLES) {
            radar.amplitude_baseline[i] += amplitude / BASELINE_SAMPLES;
            radar.phase_baseline[i] += phase / BASELINE_SAMPLES;
        }
        // Motion detection phase
        else if (radar.baseline_calibrated) {
            float amp_deviation = amplitude - radar.amplitude_baseline[i];
            float phase_deviation = phase - radar.phase_baseline[i];
            
            // Handle phase wrapping
            if (phase_deviation > PI) phase_deviation -= 2 * PI;
            if (phase_deviation < -PI) phase_deviation += 2 * PI;
            
            // Update variance
            radar.amplitude_variance[i] = updateVariance(radar.amplitude_variance[i], 
                                                        amp_deviation, radar.sample_count);
            radar.phase_variance[i] = updateVariance(radar.phase_variance[i], 
                                                   phase_deviation, radar.sample_count);
            
            // Calculate distance and velocity
            float subcarrier_freq = WIFI_CENTER_FREQ + (i * SUBCARRIER_SPACING);
            float wavelength = LIGHT_SPEED / subcarrier_freq;
            radar.distance_estimates[i] = calculateDistance(phase_deviation, wavelength);
            
            if (time_delta > 0) {
                float phase_rate = phase_deviation / time_delta;
                radar.doppler_shift[i] = (phase_rate * wavelength) / (4 * PI);
            }
            
            // Detect motion
            if (detectVolumetricMotion(i, amp_deviation, phase_deviation)) {
                performSpatialAnalysis(i, data->rx_ctrl.rssi, subcarrier_freq, time_delta);
            }
        }
    }
    
    radar.sample_count++;
    radar.last_timestamp = current_time;
    
    // Complete baseline calibration
    if (radar.sample_count == BASELINE_SAMPLES && !radar.baseline_calibrated) {
        radar.baseline_calibrated = true;
        Serial.println("\n=== BASELINE CALIBRATION COMPLETE ===");
        Serial.println("3D Volumetric WiFi Radar is now ACTIVE!");
        Serial.println("Move around to see motion detection...");
        Serial.println("======================================\n");
        //printCalibrationSummary();
    }
}

float updateVariance(float current_variance, float new_value, int sample_count) {
    if (sample_count <= BASELINE_SAMPLES) return 0.0;
    
    float delta = new_value * new_value;
    return current_variance + (delta - current_variance) / (sample_count - BASELINE_SAMPLES);
}

float calculateDistance(float phase_deviation, float wavelength) {
    float distance = (fabs(phase_deviation) * wavelength) / (4.0 * PI);
    return distance;
}

bool detectVolumetricMotion(int subcarrier_idx, float amp_dev, float phase_dev) {
    float amp_threshold = 2.0 + (0.1 * subcarrier_idx);
    float phase_threshold = 0.3 + (0.01 * subcarrier_idx);
    float variance_threshold = 1.5;
    
    bool amplitude_motion = fabs(amp_dev) > amp_threshold;
    bool phase_motion = fabs(phase_dev) > phase_threshold;
    bool variance_motion = radar.amplitude_variance[subcarrier_idx] > variance_threshold;
    
    return (amplitude_motion || phase_motion || variance_motion);
}

void performSpatialAnalysis(int subcarrier_idx, int rssi, float frequency, float time_delta) {
    float distance = radar.distance_estimates[subcarrier_idx];
    float doppler = radar.doppler_shift[subcarrier_idx];
    float velocity = fabs(doppler);
    
    float angle = calculateAngleOfArrival(subcarrier_idx, radar.amplitude_variance[subcarrier_idx]);
    
    float x = distance * cos(angle * PI / 180.0);
    float y = distance * sin(angle * PI / 180.0);
    float z = calculateVerticalPosition(frequency, rssi, distance);
    
    String motion_type = classifyMotion(velocity, radar.amplitude_variance[subcarrier_idx]);
    
    Serial.printf("Subcarrier: %d (%.1f MHz)\n", subcarrier_idx, frequency / 1000000.0);
    Serial.printf("Distance: %.2f m\n", distance);
    Serial.printf("Velocity: %.2f m/s\n", velocity);
    Serial.printf("Angle: %.1f°\n", angle);
    Serial.printf("3D Position: (%.2f, %.2f, %.2f) m\n", x, y, z);
    Serial.printf("Motion Type: %s\n", motion_type.c_str());
    Serial.printf("RSSI: %d dBm\n", rssi);
}

float calculateAngleOfArrival(int subcarrier_idx, float amplitude_variance) {
    float base_angle = (subcarrier_idx * 360.0) / MAX_SUBCARRIERS;
    float variance_offset = amplitude_variance * 10.0;
    return fmod(base_angle + variance_offset, 360.0);
}

float calculateVerticalPosition(float frequency, int rssi, float horizontal_distance) {
    float theoretical_rssi = -32.45 - 20*log10(horizontal_distance) - 20*log10(frequency/1000000.0);
    float height_factor = (rssi - theoretical_rssi) / 6.0;
    return fmax(0.0, height_factor * 0.5);
}

String classifyMotion(float velocity, float amplitude_variance) {
    if (velocity < 0.1 && amplitude_variance < 1.0) return "Micro-motion";
    else if (velocity < 0.5) return "Slow movement";
    else if (velocity < 1.5) return "Walking";
    else if (velocity < 3.0) return "Fast movement";
    else return "Rapid motion";
}

void tryPasswordVariations() {
    Serial.println("Trying common password issues...");
    
    // Common password problems that cause Status 6:
    String passwords[] = {
        String(password),           // Original
        String(password) + " ",     // Extra space
        String(password).substring(0, strlen(password)-1), // Missing last char
        " " + String(password),     // Leading space
    };
    
    for (int i = 0; i < 4; i++) {
        Serial.printf("Trying password variation %d (length: %d)\n", i+1, passwords[i].length());
        
        WiFi.disconnect();
        delay(2000);
        WiFi.begin(ssid, passwords[i].c_str());
        
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
            delay(1000);
            Serial.print(".");
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\nPassword variation %d worked!\n", i+1);
            Serial.printf("Correct password: '%s'\n", passwords[i].c_str());
            return;
        } else {
            Serial.printf(" Failed (Status: %d)\n", WiFi.status());
        }
    }
}

void tryAlternativeMethods() {
    Serial.println("Trying router compatibility fixes...");
    
    // Method 1: Try with specific WiFi config
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    
    // Force WPA2 authentication
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    
    WiFi.begin();
    
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 20000) {
        delay(1000);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nRouter compatibility fix worked!");
        Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nAll methods failed. Check these:");
        Serial.println("1. WiFi password is EXACTLY correct (case sensitive)");
        Serial.println("2. Password is 8+ characters long");
        Serial.println("3. Router is set to WPA2 (not WPA3)");
        Serial.println("4. 2.4GHz band is enabled (not 5GHz only)");
        Serial.println("5. Try moving closer to router");
        Serial.println("6. Restart your router");
        Serial.println("7. Check for hidden characters in SSID/password");
    }
}

void printCalibrationSummary() {
    Serial.println("=== CALIBRATION SUMMARY ===");
    for (int i = 0; i < min(8, MAX_SUBCARRIERS); i++) {
        Serial.printf("SC%d: Amp=%.2f, Phase=%.3f rad\n", 
                     i, radar.amplitude_baseline[i], radar.phase_baseline[i]);
    }
    //Serial.println("============================\n");
}

void generateCSIPing() {
    static unsigned long lastPing = 0;
    if (millis() - lastPing > 500) {  // Ping every 500ms
        WiFiClient client;
        if (client.connect("8.8.8.8", 53)) {
            client.print("GET / HTTP/1.1\r\n\r\n");
            client.stop();
        }
        lastPing = millis();
    }
}

void printRadarStatus() {
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 5000 && radar.baseline_calibrated) {  // Every 5 seconds
        Serial.println("\n=== RADAR STATUS ===");
        Serial.printf("Samples: %d, RSSI: %d dBm\n", radar.sample_count, WiFi.RSSI());
        
        for (int i = 0; i < 4; i++) {
            Serial.printf("SC%d: A=%.2f, P=%.3f, D=%.2fm\n", 
                         i, radar.amplitude_variance[i], radar.phase_variance[i], 
                         radar.distance_estimates[i]);
        }
        Serial.println("===================\n");
        lastStatus = millis();
    }
}

void loop() {
    // Check WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected - restarting...");
        ESP.restart();
    }
    
    // Generate CSI data
    generateCSIPing();
    
    // Print status periodically
    //printRadarStatus();
    
    delay(1000);
}
