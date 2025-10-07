/*
  FPM383C Basic Usage Example

  This example demonstrates basic fingerprint enrollment and matching
  using the FPM383C fingerprint sensor.

  Hardware Connections:
  - V_TOUCH: 3.3V
  - TOUCHOUT: Digital Pin 3 (optional)
  - VCC: 3.3V
  - TX: Digital Pin 2
  - RX: Digital Pin 4 (with 10kΩ pull-up)
  - GND: GND
*/
#include <WiFi.h>
#include <WiFiUdp.h>
#include <FPM383C.h>

// WiFi credentials
const char* ssid = "HOME";
const char* password = "amin% 1993";

// UDP settings
WiFiUDP udp;
const char* pcIP = "192.168.1.109";  // Change to your PC's IP address
const int udpPort = 8889;            // Different port for image data
#define IMAGE_WIDTH 64
#define IMAGE_HEIGHT 80
uint16_t imageSize = 5120 ;
uint32_t imageBytes = 5120;
uint8_t* imageBuffer = NULL;  // Will be allocated dynamically

// Initialize sensor (RX pin 2, TX pin 4, Touch interrupt pin 3)
FPM383C fingerprint(19, 18, 17);
uint8_t imageData[64 * 80];
#define LED_PIN 2

void setup() {
  Serial.begin(115200);
  Serial.println("FPM383C Basic Usage Example");

  // Allocate image buffer in PSRAM if available, otherwise in heap
  imageBuffer = (uint8_t*)ps_malloc(imageSize);
  if (imageBuffer == NULL) {
    imageBuffer = (uint8_t*)malloc(imageSize);
  }
  
  if (imageBuffer == NULL) {
    Serial.println("Failed to allocate memory for image buffer!");
    while(1);
  }
  // Connect to WiFi
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  digitalWrite(LED_PIN, LOW);
  
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Sending images to: ");
  Serial.print(pcIP);
  Serial.print(":");
  Serial.println(udpPort);
  
  delay(1000);

  // Enable debug output
  fingerprint.enableDebug(false);
  
  // Initialize sensor
  if (fingerprint.begin(115200)) {
    Serial.println("Sensor initialized successfully!");

    // Get module information
    String moduleId = fingerprint.getModuleId();
    Serial.println("Module ID: " + moduleId);

    uint16_t templateCount = fingerprint.getTemplateCount();
    Serial.println("Stored templates: " + String(templateCount));

    // Set LED to green to indicate ready
    fingerprint.setLED(FP_LED_MODE_ON, FP_LED_GREEN);
  } else {
    Serial.println("Failed to initialize sensor!");
    Serial.println("Error: " + fingerprint.getErrorString(fingerprint.getLastError()));
    while (1); // Stop execution
  }

  //  if(fingerprint.setBaudrate(115200)){
  //    Serial.println("set 115200 sensor!");
  //  }else{
  //
  //    Serial.println("Failed to set buad rate sensor!");
  //  }
}

void loop() {

  fingerprint.setLED(FP_LED_MODE_ON, FP_LED_RED);

  if (fingerprint.featureInformation()) {
    //    fingerprint.setLED(FP_LED_MODE_BLINK, FP_LED_GREEN, 10, 10, 3);

    if (fingerprint.uploadFingerprintImage(imageBuffer)) {
      sendImageViaUDP();
      // Serial.println("============== finger print ============== ");
      // for ( int i = 0 ; i < imageSize ; i++) {
      //   Serial.print(imageBuffer[i], HEX);
      //   if ( i % 64 == 0 )
      //     Serial.println(" ");
      // }
    }
    delay(1500);
  }

}


void sendImageViaUDP() {
  Serial.println("\n=== Sending Image via UDP ===");
  
  // Create image header with metadata
  struct ImageHeader {
    char marker[8];      // "FPIMG"
    uint16_t width;
    uint16_t height;
    uint32_t size;
    uint32_t timestamp;
  } header;
  
  memcpy(header.marker, "FPIMG\0\0\0", 8);
  header.width = IMAGE_WIDTH;
  header.height = IMAGE_HEIGHT;
  header.size = imageBytes;
  header.timestamp = millis();
  
  // Send header
  udp.beginPacket(pcIP, udpPort);
  udp.write((uint8_t*)&header, sizeof(header));
  udp.endPacket();
  delay(50);
  
  Serial.print("Sending ");
  Serial.print(imageBytes);
  Serial.print(" bytes (");
  Serial.print(IMAGE_WIDTH);
  Serial.print("x");
  Serial.print(IMAGE_HEIGHT);
  Serial.println(" pixels)");
  
  // Send image data in chunks
  int chunkSize = 1400;  // Safe UDP payload size
  int bytesSent = 0;
  int chunkNum = 0;
  
  while (bytesSent < imageBytes) {
    int bytesToSend = min(chunkSize, int(imageBytes - bytesSent));
    
    // Send chunk
    udp.beginPacket(pcIP, udpPort);
    udp.write(imageBuffer + bytesSent, bytesToSend);
    udp.endPacket();
    
    bytesSent += bytesToSend;
    chunkNum++;
    
    // Progress
    float progress = (float)bytesSent / imageBytes * 100.0;
    Serial.print("Chunk ");
    Serial.print(chunkNum);
    Serial.print(": ");
    Serial.print(bytesSent);
    Serial.print("/");
    Serial.print(imageBytes);
    Serial.print(" (");
    Serial.print(progress, 1);
    Serial.println("%)");
    
    delay(10);  // Small delay between chunks
  }
  
  // Send end marker
  delay(50);
  udp.beginPacket(pcIP, udpPort);
  udp.write((uint8_t*)"FPIMGEND", 8);
  udp.endPacket();
  
  Serial.println("\n=== Image sent successfully! ===");
  
  // Display image preview (show first few pixels as ASCII art)
  displayImagePreview();
}

void displayImagePreview() {
  Serial.println("\nImage preview (top-left corner):");
  
  // Show a small preview using ASCII art
  int previewWidth = min(32, IMAGE_WIDTH);
  int previewHeight = min(16, IMAGE_HEIGHT);
  
  for (int y = 0; y < previewHeight; y++) {
    for (int x = 0; x < previewWidth; x++) {
      int idx = y * IMAGE_WIDTH + x;
      uint8_t pixel = imageBuffer[idx];
      
      // Convert to ASCII based on intensity
      char c;
      if (pixel < 32) c = ' ';
      else if (pixel < 64) c = '.';
      else if (pixel < 96) c = ':';
      else if (pixel < 128) c = '-';
      else if (pixel < 160) c = '=';
      else if (pixel < 192) c = '+';
      else if (pixel < 224) c = '*';
      else c = '#';
      
      Serial.print(c);
    }
    Serial.println();
  }
}
