// settings.h
// Configuration constants for the RP2040 UDP Motor Controller.

#pragma once

#include <Arduino.h> // Include Arduino types
#include <IPAddress.h> // Include for IPAddress type

// --- Network Settings ---
// MAC Address (must be 6 bytes/uint8_t array)
static byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// Static IP configuration (change these to match your network)
const IPAddress staticIP(192, 168, 0, 60);  // IP Address of the XIAO RP2040
const IPAddress gateway(192, 168, 0, 1);     // Your router/gateway IP (Adjusted for consistency with new subnet)
const IPAddress subnet(255, 255, 255, 0);    // Your subnet mask

// UDP Port
const unsigned int UDP_PORT = 5000; 

// --- Hardware Pins ---
// Chip Select (CS) pin for W5500 Ethernet. (You specified P7/GP1/D7)
const int CS_PIN = 1; 

// Motor PWM Pin. (You specified D6/GP0)
const int MOTOR_PIN = 0; 

// --- Motor Control Settings ---
// Voltage settings used for PWM calculation (matching your original logic)
const float V_MIN = 1.5;
const float V_MAX = 12.0;
const float BATTERY_V = 12.0;

// Watchdog Timeout (in milliseconds). 5 seconds = 5000 ms.
const long WATCHDOG_TIMEOUT_MS = 5000;