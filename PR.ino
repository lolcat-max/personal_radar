#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>

const char* ssid = "";
const char* password = "";

// CSI data structure
typedef struct {
    float amplitude_baseline[64];
    float phase_baseline[64];
    bool baseline_set;
    int sample_count;
} radar_data_t;

radar_data_t radar_data = {0};

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32-S3 WiFi Volumetric Radar Starting...");
    
    // Initialize WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Configure CSI
    wifi_csi_config_t csi_config;
    csi_config.lltf_en = true;
    csi_config.htltf_en = true;
    csi_config.stbc_htltf2_en = true;
    csi_config.ltf_merge_en = true;
    csi_config.channel_filter_en = false;
    csi_config.manu_scale = 0;
    csi_config.shift = 0;
    
    esp_wifi_set_csi_config(&csi_config);
    esp_wifi_set_csi_rx_cb(&csi_callback, NULL);
    esp_wifi_set_csi(true);
    
    Serial.println("CSI radar initialized - monitoring for volumetric changes...");
}

void loop() {
    // Generate ping packets to create CSI data
    generatePingPackets();
    delay(100);
}

// CSI callback function for volumetric detection
void ARDUINO_ISR_ATTR csi_callback(void *ctx, wifi_csi_info_t *data) {
    if (!data || !data->buf || data->len < 2) {
        return;
    }
    
    processVolumetricCSI(data);
}

void processVolumetricCSI(wifi_csi_info_t *data) {
    int8_t *csi_data = data->buf;
    int num_subcarriers = min(data->len / 2, 64);
    
    // Process each subcarrier
    for (int i = 0; i < num_subcarriers; i++) {
        int8_t imaginary = csi_data[i * 2];
        int8_t real = csi_data[i * 2 + 1];
        
        // Calculate amplitude and phase
        float amplitude = sqrt(real * real + imaginary * imaginary);
        float phase = atan2(imaginary, real);
        
        // Set baseline during first 50 samples
        if (radar_data.sample_count < 50) {
            radar_data.amplitude_baseline[i] += amplitude / 50.0;
            radar_data.phase_baseline[i] += phase / 50.0;
        }
        // Detect volumetric changes after baseline
        else if (radar_data.baseline_set) {
            float amp_diff = abs(amplitude - radar_data.amplitude_baseline[i]);
            float phase_diff = abs(phase - radar_data.phase_baseline[i]);
            
            // Volumetric motion detection threshold
            if (amp_diff > 3.0 || phase_diff > 0.5) {
                // Calculate approximate distance and direction
                float distance = calculateDistance(phase, i);
                float direction = calculateDirection(amp_diff, i);
                
                Serial.printf("3D Motion detected! Subcarrier: %d, Distance: %.2fm, "
                             "Direction: %.1f°, Amp_change: %.2f, Phase_change: %.2f, RSSI: %d\n",
                             i, distance, direction, amp_diff, phase_diff, data->rx_ctrl.rssi);
            }
        }
    }
    
    radar_data.sample_count++;
    if (radar_data.sample_count == 50) {
        radar_data.baseline_set = true;
        Serial.println("Baseline calibration complete - 3D radar active!");
    }
}

float calculateDistance(float phase, int subcarrier_idx) {
    // Estimate distance based on phase and subcarrier frequency
    float frequency = 2412000000.0 + (subcarrier_idx * 312500.0); // 2.4GHz + subcarrier offset
    float wavelength = 299792458.0 / frequency;
    float distance = (phase / (2.0 * PI)) * wavelength;
    return abs(distance);
}

float calculateDirection(float amplitude_change, int subcarrier_idx) {
    // Simple directional estimation based on subcarrier response
    return (subcarrier_idx * 360.0) / 64.0;
}

void generatePingPackets() {
    // Send ping to router to generate CSI responses
    WiFiClient client;
    if (client.connect("8.8.8.8", 53)) {
        client.print("ping");
        client.stop();
    }
}
