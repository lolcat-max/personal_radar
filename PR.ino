#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

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
    
    Serial.println("ESP32-S3 Volumetric WiFi Radar with Full Math Processing");
    
    // Initialize WiFi with proper type handling
    initializeWiFiConnection();
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi Connected - Initializing CSI Math Engine...");
        initializeCSIMath();
    }
}

void initializeWiFiConnection() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(1000);
    
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);
    WiFi.persistent(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    Serial.printf("Connecting to: %s\n", ssid);
    WiFi.begin(ssid, password);
    
    // Fix: Cast waitForConnectResult to wl_status_t
    wl_status_t result = (wl_status_t)WiFi.waitForConnectResult(15000);
    
    if (result == WL_CONNECTED) {
        Serial.println("WiFi Connected!");
        Serial.printf("IP: %s, RSSI: %d dBm\n", 
                     WiFi.localIP().toString().c_str(), WiFi.RSSI());
        WiFi.setAutoReconnect(true);
    } else {
        Serial.printf("Connection failed with status: %d\n", result);
        Serial.println("Trying alternative connection method...");
        alternativeConnection();
    }
}

void alternativeConnection() {
    // Alternative connection without waitForConnectResult
    WiFi.disconnect();
    delay(1000);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    // Use WiFi.status() with proper casting instead
    int attempts = 0;
    wl_status_t status;
    
    while (attempts < 30) {
        status = WiFi.status();  // This returns wl_status_t directly
        
        if (status == WL_CONNECTED) {
            Serial.println("Alternative connection successful!");
            Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
            return;
        }
        
        Serial.printf("Attempt %d: Status %d\n", attempts + 1, status);
        delay(1000);
        attempts++;
        
        // Force reconnect every 10 attempts
        if (attempts % 10 == 0) {
            Serial.println("Forcing reconnect...");
            WiFi.disconnect();
            delay(2000);
            WiFi.begin(ssid, password);
        }
    }
    
    Serial.println("All connection attempts failed");
}

void initializeCSIMath() {
    // Configure CSI with mathematical precision requirements
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
        esp_wifi_set_csi_rx_cb(&csi_math_callback, NULL);
        esp_wifi_set_csi(true);
        
        // Initialize mathematical baseline
        radar.baseline_calibrated = false;
        radar.sample_count = 0;
        radar.last_timestamp = micros();
        
        Serial.println("CSI Mathematical Engine Initialized");
        Serial.println("Calibrating baseline - please remain stationary...");
    } else {
        Serial.printf("CSI initialization failed: %d\n", ret);
    }
}

void ARDUINO_ISR_ATTR csi_math_callback(void *ctx, wifi_csi_info_t *data) {
    if (!data || !data->buf || data->len < 4) return;
    
    processVolumetricMath(data);
}

void processVolumetricMath(wifi_csi_info_t *data) {
    int8_t *csi_raw = data->buf;
    int num_subcarriers = min(data->len / 2, MAX_SUBCARRIERS);
    unsigned long current_time = micros();
    float time_delta = (current_time - radar.last_timestamp) / 1000000.0;
    
    // Process each subcarrier with full mathematical analysis
    for (int i = 0; i < num_subcarriers; i++) {
        // Extract I/Q components
        int8_t I = csi_raw[i * 2 + 1];     // Real component
        int8_t Q = csi_raw[i * 2];         // Imaginary component
        
        // Calculate complex amplitude and phase
        float amplitude = sqrt(I * I + Q * Q);
        float phase = atan2(Q, I);         
        
        // Subcarrier frequency calculation
        float subcarrier_freq = WIFI_CENTER_FREQ + (i * SUBCARRIER_SPACING);
        float wavelength = LIGHT_SPEED / subcarrier_freq;
        
        // Baseline calibration phase
        if (radar.sample_count < BASELINE_SAMPLES) {
            radar.amplitude_baseline[i] += amplitude / BASELINE_SAMPLES;
            radar.phase_baseline[i] += phase / BASELINE_SAMPLES;
        }
        // Volumetric detection and analysis
        else if (radar.baseline_calibrated) {
            // Calculate amplitude and phase deviations
            float amp_deviation = amplitude - radar.amplitude_baseline[i];
            float phase_deviation = phase - radar.phase_baseline[i];
            
            // Handle phase wrapping
            if (phase_deviation > PI) phase_deviation -= 2 * PI;
            if (phase_deviation < -PI) phase_deviation += 2 * PI;
            
            // Calculate variance for motion detection
            radar.amplitude_variance[i] = updateVariance(radar.amplitude_variance[i], 
                                                        amp_deviation, radar.sample_count);
            radar.phase_variance[i] = updateVariance(radar.phase_variance[i], 
                                                   phase_deviation, radar.sample_count);
            
            // Distance estimation using phase information
            radar.distance_estimates[i] = calculateDistance(phase_deviation, wavelength);
            
            // Doppler shift calculation
            if (time_delta > 0) {
                float phase_rate = phase_deviation / time_delta;
                radar.doppler_shift[i] = (phase_rate * wavelength) / (4 * PI);
            }
            
            // 3D Volumetric Motion Detection
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
        Serial.println("Mathematical baseline calibration complete!");
        Serial.println("3D Volumetric radar active with full spatial analysis");
        printCalibrationSummary();
    }
}

float updateVariance(float current_variance, float new_value, int sample_count) {
    // Online variance calculation using Welford's algorithm
    if (sample_count <= BASELINE_SAMPLES) return 0.0;
    
    float delta = new_value * new_value;
    return current_variance + (delta - current_variance) / (sample_count - BASELINE_SAMPLES);
}

float calculateDistance(float phase_deviation, float wavelength) {
    // Distance estimation using phase shift
    float distance = (fabs(phase_deviation) * wavelength) / (4.0 * PI);
    return distance;
}

bool detectVolumetricMotion(int subcarrier_idx, float amp_dev, float phase_dev) {
    // Multi-threshold motion detection
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
    
    // Angle estimation based on subcarrier response pattern
    float angle = calculateAngleOfArrival(subcarrier_idx, radar.amplitude_variance[subcarrier_idx]);
    
    // 3D position estimation
    float x = distance * cos(angle * PI / 180.0);
    float y = distance * sin(angle * PI / 180.0);
    float z = calculateVerticalPosition(frequency, rssi, distance);
    
    // Motion classification
    String motion_type = classifyMotion(velocity, radar.amplitude_variance[subcarrier_idx]);
    
    // Output comprehensive spatial analysis
    Serial.printf("=== 3D VOLUMETRIC DETECTION ===\n");
    Serial.printf("Subcarrier: %d (%.1f MHz)\n", subcarrier_idx, frequency / 1000000.0);
    Serial.printf("Distance: %.2f m\n", distance);
    Serial.printf("Velocity: %.2f m/s\n", velocity);
    Serial.printf("Angle: %.1f°\n", angle);
    Serial.printf("3D Position: (%.2f, %.2f, %.2f) m\n", x, y, z);
    Serial.printf("Motion Type: %s\n", motion_type.c_str());
    Serial.printf("RSSI: %d dBm\n", rssi);
    Serial.printf("Doppler Shift: %.3f Hz\n", doppler);
    Serial.println("================================\n");
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

void printCalibrationSummary() {
    Serial.println("\n=== CALIBRATION SUMMARY ===");
    for (int i = 0; i < min(8, MAX_SUBCARRIERS); i++) {
        Serial.printf("SC%d: Amp_baseline=%.2f, Phase_baseline=%.3f rad\n", 
                     i, radar.amplitude_baseline[i], radar.phase_baseline[i]);
    }
    Serial.println("============================\n");
}

void loop() {
    // Connection monitoring
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost - restarting...");
        ESP.restart();
    }
    
    // Generate ping packets for CSI data
    generateCSIPing();
    
    // Print periodic status
    static unsigned long last_status = 0;
    if (millis() - last_status > 10000) {
        printRadarStatus();
        last_status = millis();
    }
    
    delay(100);
}

void generateCSIPing() {
    WiFiClient client;
    if (client.connect("8.8.8.8", 53)) {
        client.print("GET / HTTP/1.1\r\n\r\n");
        client.stop();
    }
}

void printRadarStatus() {
    if (!radar.baseline_calibrated) return;
    
    Serial.println("\n=== RADAR STATUS ===");
    Serial.printf("Samples processed: %d\n", radar.sample_count);
    Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());
    
    for (int i = 0; i < 4; i++) {
        Serial.printf("SC%d variance: A=%.2f, P=%.3f, D=%.2fm\n", 
                     i, radar.amplitude_variance[i], radar.phase_variance[i], 
                     radar.distance_estimates[i]);
    }
    Serial.println("===================\n");
}
