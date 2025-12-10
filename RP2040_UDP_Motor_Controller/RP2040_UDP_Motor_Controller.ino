#include <SPI.h>
#include <Ethernet.h>

// --- NETWORK CONFIGURATION ---
byte mac[] = {0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE}; 
IPAddress ip(128, 138, 224, 6);                     
unsigned int localPort = 8888;                    

EthernetUDP Udp; 

// --- MOTOR/PWM CONFIGURATION ---
const int W5500_CS = D7;    
const int MOTOR_PWM_PIN = D5;
const long PWM_FREQUENCY = 2000; 

// --- Motor Control Variables ---
bool motorEnabled = false;
int currentSpeed = 0;

// --- Function Prototypes ---
void processMotorCommand(char* buffer, int len);
void outputOff();

// -------------------------------------------------------------------
// Function to force the PWM pin to 0V using dual logic
// This ensures both the PWM peripheral and the pin are set to LOW.
// -------------------------------------------------------------------
void outputOff() {
  // 1. Tell the PWM peripheral to stop (duty cycle 0)
  analogWrite(MOTOR_PWM_PIN, 255); 
  
  Serial.println(" -> OUTPUT FORCED to (0V)");
}

// -------------------------------------------------------------------
// Function to handle the custom duty cycle logic
// -------------------------------------------------------------------
void applyPwm(int speed) {
  int dutyCycle;
  int dutyPercent;

  // Constants for 0-255 range (8-bit)
  const int MIN_DUTY = 212; 
  const int MAX_DUTY = 0;
  const int SPEED_MIN = 1;
  const int SPEED_MAX = 255;
  
  if (speed == 0) {
    // If speed is 0, rely on digitalOff() to stop
    outputOff(); 
    return; 
  } 
  
  // Before applying PWM, ensure the pin is configured for PWM again
  // NOTE: This relies on the Arduino core handling the transition correctly
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  analogWriteFreq(PWM_FREQUENCY);
  analogWriteRange(255);

  speed = constrain(speed, SPEED_MIN, SPEED_MAX);

  // CORE MAPPING: Map speed (1 to 255) linearly to the 8-bit range (0  to 212)
  dutyCycle = map(speed, 1, 255, 212, 0);

  // Write the calculated duty cycle (0-255 value)
  analogWrite(MOTOR_PWM_PIN, dutyCycle);
}



// -------------------------------------------------------------------
// Function to process the incoming two-byte UDP command
// -------------------------------------------------------------------
void processMotorCommand(char* buffer, int len) {
  if (len < 2) {
    Serial.println("Error: Packet too short. Expecting 2 bytes.");
    return;
  }
  
  byte enableCmd = (byte)buffer[0];
  byte speedCmd = (byte)buffer[1];

  // 1. Process Enable Command
  if (enableCmd == 1) {
    motorEnabled = true;
    Serial.print(" [ENABLE: ON] ");
  } else if (enableCmd == 0) {
    motorEnabled = false;
    currentSpeed = 0; // Force speed to 0 when disabled
    // Use digitalOff to reliably stop the motor when disabled
    outputOff(); 
    Serial.print(" [ENABLE: OFF] ");
    return;
  } else {
    Serial.print(" [ENABLE: Invalid Command (not 0 or 1)] ");
    return;
  }

  // 2. Process Speed Command (Only if enabled)
  if (motorEnabled) {
    currentSpeed = speedCmd; 
    
    Serial.print(" [SPEED: ");
    Serial.print(currentSpeed);
    Serial.print("] \n");

    applyPwm(currentSpeed);
  }
}

// -------------------------------------------------------------------
// setup() Function
// -------------------------------------------------------------------
void setup() {
  digitalWrite(MOTOR_PWM_PIN,HIGH);

  Serial.begin(115200);
  delay(100);

   // Initialize PWM parameters for when the motor is commanded ON
 pinMode(MOTOR_PWM_PIN, OUTPUT);
  analogWriteFreq(PWM_FREQUENCY);
  analogWriteRange(255);
  analogWrite(MOTOR_PWM_PIN, 255);

  Serial.print("PWM Set: Pin D6 @ ");
  Serial.print(PWM_FREQUENCY);
  Serial.println("Hz with resolution 0-255");


  // --- ETHERNET SETUP ---
  Serial.println("Initializing Ethernet...");
  Ethernet.init(W5500_CS); 
  Ethernet.begin(mac, ip); 
  
  Serial.print("Attempting to connect...");
  delay(1000); 
  
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("\nFailed: Ethernet hardware not found. Check wiring.");
  } else if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("\nFailed: Ethernet cable is not connected.");
  } else {
    Serial.println("\nSuccess!");
    Serial.print("Local IP address: ");
    Serial.println(Ethernet.localIP());
  }

  // Start listening for UDP packets
  Udp.begin(localPort);
  Serial.print("Listening for UDP packets on port: ");
  Serial.println(localPort);
  Serial.println("------------------------------------");
} 

// -------------------------------------------------------------------
// loop() Function
// -------------------------------------------------------------------
void loop() {
  int packetSize = Udp.parsePacket();

  if (packetSize) {
    char packetBuffer[2]; 
    int len = Udp.read(packetBuffer, 2); 

    Serial.print("--- UDP Command Received (2 bytes expected) ---");
    Serial.print(" | Size: ");
    Serial.print(len);
    Serial.println(" bytes");

    processMotorCommand(packetBuffer, len);
    
    Serial.println("------------------------------------");
  }

  delay(1);
}