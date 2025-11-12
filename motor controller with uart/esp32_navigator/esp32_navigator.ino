/**
 * ESP32 Navigator Module
 * ======================
 * Sends navigation commands to Raspberry Pi via Serial/WiFi
 * 
 * Hardware: ESP32 DevKit
 * Communication: WiFi TCP or Serial UART
 * Protocol: JSON messages over newline-delimited text
 * 
 * Author: Robot Project Team
 * Date: November 2025
 * Version: 1.0
 */

#include <Arduino.h>
#include <ArduinoJson.h>  // Install via Library Manager

// ============================================================================
// CONFIGURATION - Choose Communication Method
// ============================================================================

#define USE_WIFI 1        // Set to 1 for WiFi, 0 for Serial
#define USE_SERIAL 0      // Set to 1 for Serial, 0 for WiFi

// WiFi Configuration
#if USE_WIFI
  #include <WiFi.h>
  
  const char* WIFI_SSID = "YourWiFiSSID";        // Change to your WiFi name
  const char* WIFI_PASSWORD = "YourWiFiPassword"; // Change to your WiFi password
  const int SERVER_PORT = 8888;                   // TCP server port
  
  WiFiServer server(SERVER_PORT);
  WiFiClient client;
  bool client_connected = false;
#endif

// Serial Configuration
#if USE_SERIAL
  #define SERIAL_BAUDRATE 115200
  #define RPI_SERIAL Serial2  // Use Serial2 for ESP32 (TX=17, RX=16)
#endif

// ============================================================================
// NAVIGATION COMMANDS
// ============================================================================

enum NavCommand {
  CMD_MOVE_FORWARD,
  CMD_TURN_LEFT,
  CMD_TURN_RIGHT,
  CMD_STOP,
  CMD_REACHED_WAYPOINT,
  CMD_REACHED_DESTINATION,
  CMD_RECALCULATING
};

// Command strings (must match Python NavCommand enum)
const char* cmdStrings[] = {
  "MOVE_FORWARD",
  "TURN_LEFT",
  "TURN_RIGHT",
  "STOP",
  "REACHED_WAYPOINT",
  "REACHED_DESTINATION",
  "RECALCULATING"
};

// ============================================================================
// PATHFINDING & NAVIGATION STATE
// ============================================================================

struct Waypoint {
  float x;
  float y;
};

// Example path (replace with your pathfinding algorithm)
Waypoint path[] = {
  {0.0, 0.0},    // Start
  {2.0, 0.0},    // Waypoint 1
  {2.0, 2.0},    // Waypoint 2
  {0.0, 2.0}     // Destination
};
const int pathLength = 4;
int currentWaypointIndex = 0;

// Current robot state (update from RPi feedback)
float currentX = 0.0;
float currentY = 0.0;
float currentHeading = 0.0;  // Degrees

// ============================================================================
// COMMUNICATION FUNCTIONS
// ============================================================================

/**
 * Send navigation command to Raspberry Pi
 * 
 * @param command Navigation command type
 * @param angle Turn angle in degrees (for TURN_LEFT/TURN_RIGHT)
 * @param distance Distance in meters (for MOVE_FORWARD)
 * @param speed Motor speed percentage 0-100
 */
void sendNavigationCommand(NavCommand command, float angle = 0, float distance = 0, int speed = 70) {
  // Create JSON document
  StaticJsonDocument<256> doc;
  
  doc["type"] = "navigation";
  doc["command"] = cmdStrings[command];
  doc["angle"] = angle;
  doc["distance"] = distance;
  doc["speed"] = speed;
  doc["timestamp"] = millis();
  doc["priority"] = "normal";
  
  // Serialize to string
  String output;
  serializeJson(doc, output);
  output += "\n";  // Newline delimiter for message framing
  
  // Send via appropriate channel
#if USE_WIFI
  if (client_connected && client.connected()) {
    client.print(output);
    Serial.print("WiFi → RPi: ");
    Serial.println(output);
  } else {
    Serial.println("⚠️  No client connected");
  }
#endif

#if USE_SERIAL
  RPI_SERIAL.print(output);
  Serial.print("Serial → RPi: ");
  Serial.println(output);
#endif
}

/**
 * Receive status message from Raspberry Pi
 * Parses JSON and updates robot state
 */
void receiveStatusMessage() {
  String message = "";
  
#if USE_WIFI
  if (client_connected && client.available()) {
    message = client.readStringUntil('\n');
  }
#endif

#if USE_SERIAL
  if (RPI_SERIAL.available()) {
    message = RPI_SERIAL.readStringUntil('\n');
  }
#endif

  if (message.length() > 0) {
    Serial.print("RPi → ESP32: ");
    Serial.println(message);
    
    // Parse JSON
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (!error) {
      // Extract status information
      const char* type = doc["type"];
      
      if (strcmp(type, "status") == 0) {
        const char* state = doc["state"];
        bool obstacle = doc["obstacle_detected"];
        float pos_x = doc["position"]["x"];
        float pos_y = doc["position"]["y"];
        float heading = doc["heading"];
        float us_left = doc["ultrasonic_left"];
        float us_right = doc["ultrasonic_right"];
        
        // Update robot state
        currentX = pos_x;
        currentY = pos_y;
        currentHeading = heading;
        
        Serial.printf("  State: %s, Pos: (%.2f, %.2f), Heading: %.1f°\n", 
                     state, pos_x, pos_y, heading);
        Serial.printf("  Ultrasonic: L=%.1fcm R=%.1fcm\n", us_left, us_right);
        
        if (obstacle) {
          Serial.println("  ⚠️  OBSTACLE DETECTED!");
        }
      }
    } else {
      Serial.print("  JSON parse error: ");
      Serial.println(error.c_str());
    }
  }
}

// ============================================================================
// NAVIGATION LOGIC
// ============================================================================

/**
 * Calculate angle to turn from current heading to target
 * 
 * @param targetX Target X coordinate
 * @param targetY Target Y coordinate
 * @return Angle to turn in degrees (-180 to +180)
 */
float calculateTurnAngle(float targetX, float targetY) {
  // Calculate angle to target
  float dx = targetX - currentX;
  float dy = targetY - currentY;
  float targetAngle = atan2(dy, dx) * 180.0 / PI;
  
  // Calculate turn angle (normalize to -180 to +180)
  float turnAngle = targetAngle - currentHeading;
  
  while (turnAngle > 180) turnAngle -= 360;
  while (turnAngle < -180) turnAngle += 360;
  
  return turnAngle;
}

/**
 * Calculate distance to target waypoint
 */
float calculateDistance(float targetX, float targetY) {
  float dx = targetX - currentX;
  float dy = targetY - currentY;
  return sqrt(dx*dx + dy*dy);
}

/**
 * Navigate to next waypoint in path
 */
void navigateToNextWaypoint() {
  if (currentWaypointIndex >= pathLength) {
    // Reached destination
    sendNavigationCommand(CMD_REACHED_DESTINATION);
    Serial.println("🏁 Destination reached!");
    return;
  }
  
  // Get target waypoint
  Waypoint target = path[currentWaypointIndex];
  
  Serial.printf("\n📍 Navigating to waypoint %d: (%.2f, %.2f)\n", 
               currentWaypointIndex, target.x, target.y);
  
  // Calculate turn angle
  float turnAngle = calculateTurnAngle(target.x, target.y);
  
  // Send turn command if needed (threshold: 5 degrees)
  if (abs(turnAngle) > 5.0) {
    if (turnAngle > 0) {
      sendNavigationCommand(CMD_TURN_LEFT, abs(turnAngle), 0, 60);
      Serial.printf("  🔄 Turn LEFT %.1f°\n", abs(turnAngle));
    } else {
      sendNavigationCommand(CMD_TURN_RIGHT, abs(turnAngle), 0, 60);
      Serial.printf("  🔄 Turn RIGHT %.1f°\n", abs(turnAngle));
    }
    
    delay(2000);  // Wait for turn to complete (adjust based on your robot)
  }
  
  // Calculate distance
  float distance = calculateDistance(target.x, target.y);
  
  // Send forward command
  sendNavigationCommand(CMD_MOVE_FORWARD, 0, distance, 70);
  Serial.printf("  ⬆️  Move FORWARD %.2fm\n", distance);
  
  delay(3000);  // Wait for movement (adjust based on your robot)
  
  // Move to next waypoint
  currentWaypointIndex++;
  sendNavigationCommand(CMD_REACHED_WAYPOINT);
}

// ============================================================================
// SETUP & MAIN LOOP
// ============================================================================

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║   ESP32 Navigation Module v1.0        ║");
  Serial.println("╚════════════════════════════════════════╝");
  
#if USE_WIFI
  // Connect to WiFi
  Serial.printf("\n📶 Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\n✓ WiFi connected!");
  Serial.print("  IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("  Server Port: %d\n", SERVER_PORT);
  
  // Start TCP server
  server.begin();
  Serial.println("✓ TCP server started");
  Serial.println("\n⏳ Waiting for Raspberry Pi to connect...");
#endif

#if USE_SERIAL
  // Initialize Serial2 for RPi communication
  RPI_SERIAL.begin(SERIAL_BAUDRATE, SERIAL_8N1, 16, 17);  // RX=16, TX=17
  Serial.printf("\n✓ Serial initialized: %d baud\n", SERIAL_BAUDRATE);
  Serial.println("  TX Pin: GPIO 17");
  Serial.println("  RX Pin: GPIO 16");
#endif

  Serial.println("\n✓ Setup complete. Ready to navigate!\n");
  delay(2000);
}

void loop() {
#if USE_WIFI
  // Check for new client connections
  if (!client_connected) {
    client = server.available();
    if (client) {
      Serial.println("\n✓ Raspberry Pi connected!");
      client_connected = true;
      
      // Send initial greeting
      sendNavigationCommand(CMD_STOP);
      delay(1000);
    }
  }
  
  // Check if client disconnected
  if (client_connected && !client.connected()) {
    Serial.println("\n⚠️  Raspberry Pi disconnected");
    client_connected = false;
    client.stop();
  }
#endif

  // Main navigation logic
  static unsigned long lastCommandTime = 0;
  const unsigned long COMMAND_INTERVAL = 5000;  // 5 seconds between waypoints
  
  if (millis() - lastCommandTime > COMMAND_INTERVAL) {
    lastCommandTime = millis();
    
#if USE_WIFI
    if (client_connected) {
      navigateToNextWaypoint();
    }
#endif

#if USE_SERIAL
    navigateToNextWaypoint();
#endif
  }
  
  // Receive status updates from RPi
  receiveStatusMessage();
  
  delay(100);
}

// ============================================================================
// EXAMPLE: Manual Command Testing
// ============================================================================

/**
 * Use this function to manually test commands via Serial Monitor
 * Type commands in Serial Monitor (115200 baud):
 *   F - Forward 1m
 *   L - Turn left 90°
 *   R - Turn right 90°
 *   S - Stop
 */
void manualControlMode() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case 'F':
      case 'f':
        sendNavigationCommand(CMD_MOVE_FORWARD, 0, 1.0, 70);
        Serial.println("Manual: Forward 1m");
        break;
      
      case 'L':
      case 'l':
        sendNavigationCommand(CMD_TURN_LEFT, 90, 0, 60);
        Serial.println("Manual: Turn left 90°");
        break;
      
      case 'R':
      case 'r':
        sendNavigationCommand(CMD_TURN_RIGHT, 90, 0, 60);
        Serial.println("Manual: Turn right 90°");
        break;
      
      case 'S':
      case 's':
        sendNavigationCommand(CMD_STOP);
        Serial.println("Manual: Stop");
        break;
    }
  }
}

// To enable manual mode, replace main loop with:
// void loop() { manualControlMode(); delay(100); }
