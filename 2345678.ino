/*
======================================================================
                    FUELGUARD - COMPLETE SYSTEM DEMO
======================================================================

CONTROLLER:
    ESP32-WROOM

SYSTEM:
    Vehicle ON/OFF detection
    Fuel level monitoring
    Vibration/tamper detection
    MPU6050 motion/tilt monitorihttps://github.com/rahulsd042007-wq/Underground-Cable-Fault-Detectionng
    Refueling detection
    Fuel theft detection
    GPS location logging
    Trip start/stop fuel + GPS snapshot
    GPS-based trip distance and estimated mileage

======================================================================
HARDWARE PIN MAP
======================================================================

MPU6050
    VCC  -> 3.3V
    GND  -> GND
    SDA  -> GPIO 21
    SCL  -> GPIO 22

SW-420
    VCC  -> 3.3V
    GND  -> GND
    DO   -> GPIO 27

NEO-6M GPS
    VCC  -> 3.3V
    GND  -> GND
    TX   -> GPIO 16
    RX   -> GPIO 17

DC VOLTAGE SENSOR
    OUT  -> GPIO 33
    GND  -> GND
    +    -> Switched vehicle supply

HC-SR04
    TRIG -> GPIO 5
    ECHO -> GPIO 18

IMPORTANT:
    HC-SR04 ECHO is a 5V signal.
    Use a voltage divider / level shifter before GPIO18.

======================================================================
DEMO VOLTAGE CONDITIONS
======================================================================

Vehicle OFF:
    Input supply approximately 12 V
    Voltage sensor output approximately 2.4 V

Vehicle ON:
    Boosted supply approximately 14 V
    Voltage sensor output approximately 2.8 V

Decision threshold:
    2.60 V

    < 2.60 V -> VEHICLE OFF
    >= 2.60 V -> VEHICLE ON

======================================================================
CASE 1 - THEFT
======================================================================

Vehicle OFF
      +
Vibration detected
      |
      v
SUSPICIOUS ACTIVITY
      |
      v
Monitor fuel level
      |
      v
Fuel decreases significantly
      |
      v
THEFT CONFIRMED
      |
      v
GPS LOCATION

======================================================================
CASE 2 - VEHICLE ON
======================================================================

Vehicle ON
      |
      +--> Vibration -> IGNORE
      |
      +--> MPU movement -> IGNORE for theft
      |
      v
CONTINUOUS FUEL MONITORING
      |
      v
TRIP DISTANCE + MILEAGE TRACKING (GPS)

======================================================================
CASE 3 - REFUELING
======================================================================

Vehicle OFF
      |
      v
Fuel level increases
      |
      v
Save INITIAL fuel
      |
      v
Wait for level to stabilize
      |
      v
Save FINAL fuel
      |
      v
FINAL - INITIAL
      |
      v
FUEL ADDED + GPS LOCATION

======================================================================
CASE 4 - TRIP START / STOP
======================================================================

OFF -> ON : save tripStartFuel + start GPS + start time
ON -> OFF : save tripEndFuel + end GPS + end time
            -> tripFuelConsumed = start - end (clamped >= 0)
            -> tripMileage = tripDistanceKm / tripFuelConsumed

======================================================================
*/

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <WiFi.h>
#include <WebServer.h>


// ====================================================================
// 1. PIN DEFINITIONS
// ====================================================================

// Vehicle voltage sensor
#define VOLTAGE_PIN       33

// HC-SR04
#define TRIG_PIN          5
#define ECHO_PIN          18

// SW-420
#define VIBRATION_PIN     27

// GPS
#define GPS_RX_PIN        16
#define GPS_TX_PIN        17

// MPU6050
#define MPU_SDA_PIN       21
#define MPU_SCL_PIN       22

// Theft alarm output
#define BUZZER_PIN        25
#define ALARM_LED_PIN     26

// Non-blocking blink/beep rate for the theft alarm
#define ALARM_BLINK_MS    300


// ====================================================================
// 2. VEHICLE VOLTAGE SETTINGS
// ====================================================================

// Voltage sensor output:
// Approximately 2.4 V -> vehicle OFF
// Approximately 2.8 V -> vehicle ON

#define VEHICLE_ON_THRESHOLD_V      2.60

// Optional hysteresis to prevent ON/OFF flickering
#define VEHICLE_ON_LEVEL_V          2.60
#define VEHICLE_OFF_LEVEL_V         2.50


// ====================================================================
// 3. TANK SETTINGS
// ====================================================================

#define TANK_CAPACITY_L             20.0

// Minimum change required for event detection
#define FUEL_CHANGE_THRESHOLD_L     0.5


// ====================================================================
// 4. ULTRASONIC SETTINGS
// ====================================================================

#define ULTRASONIC_SAMPLES          7

// Time between ultrasonic measurements
#define ULTRASONIC_SAMPLE_DELAY_MS  50


// ====================================================================
// 5. STABILITY SETTINGS
// ====================================================================

// Fuel must remain stable before confirming event
#define FUEL_STABLE_TIME_MS         3000

// Maximum acceptable variation during stabilization
#define FUEL_STABLE_BAND_L          0.5


// ====================================================================
// 6. DISPLAY / SENSOR TIMING
// ====================================================================

// Serial dashboard every 3 seconds
#define DASHBOARD_INTERVAL_MS       3000

// Main sensor processing every 1 second
#define SENSOR_INTERVAL_MS          1000


// ====================================================================
// 7. STARTUP / SHUTDOWN SETTLING
// ====================================================================

#define STARTUP_SETTLE_MS           2000

#define PARKING_SETTLE_MS           2000


// ====================================================================
// 8. SUSPICIOUS ACTIVITY
// ====================================================================

// Time allowed for theft confirmation
#define THEFT_MONITOR_TIME_MS       10000


// ====================================================================
// 9. TRIP / GPS DISTANCE SETTINGS
// ====================================================================

// Any implied speed above this between two GPS fixes is treated as a
// GPS jump and the segment is discarded instead of added to distance.
#define MAX_VALID_SPEED_KMH         150.0

// Roll/pitch beyond this angle means the vehicle is standing on a
// slope/incline rather than level ground.
#define INCLINE_THRESHOLD_DEG       10.0


// ====================================================================
// 9B. INCLINE / TILT SETTINGS
// ====================================================================

// If |roll| or |pitch| exceeds this angle, the vehicle is considered
// parked/standing on an incline (not flat ground). Used to flag
// fuel-level readings that may be skewed by slope, and shown live
// on the dashboard's 3D orientation indicator.
#define INCLINE_THRESHOLD_DEG       10.0


// ====================================================================
// 10. OBJECTS
// ====================================================================

TinyGPSPlus gps;

HardwareSerial GPS_Serial(1);

Adafruit_MPU6050 mpu;


// ====================================================================
// 11. MPU STATUS
// ====================================================================

bool mpuAvailable = false;


// ====================================================================
// 12. SYSTEM STATES
// ====================================================================

enum SystemState
{
    STATE_VEHICLE_OFF,
    STATE_VEHICLE_ON,
    STATE_SUSPICIOUS,
    STATE_REFUELING,
    STATE_THEFT
};


SystemState currentState =
    STATE_VEHICLE_OFF;


// ====================================================================
// 13. VEHICLE VARIABLES
// ====================================================================

bool vehicleOn = false;

bool previousVehicleOn = false;


// ====================================================================
// 14. FUEL VARIABLES
// ====================================================================

float currentFuelLevel = 0.0;

float previousFuelLevel = 0.0;

float storedFuelReference = 0.0;


// Refueling

float refuelInitialLevel = 0.0;

float refuelFinalLevel = 0.0;

float fuelAdded = 0.0;


// Theft

float theftInitialLevel = 0.0;

float theftFinalLevel = 0.0;

float fuelLost = 0.0;

// Fuel-drop theft monitor (independent of vibration)

bool fuelDropMonitoring = false;

float fuelDropInitialLevel = 0.0;

unsigned long fuelDropStartTime = 0;

// Persisted event flags - these stay TRUE (driving the alarm + the
// dashboard Event Status block) until the vehicle is next started,
// unlike currentState which gets re-evaluated every sensor cycle.

bool theftAlarmActive = false;

bool refuelEventActive = false;

// Buzzer/LED blink state

bool alarmOutputState = false;

unsigned long lastAlarmToggleTime = 0;


// ====================================================================
// 15. ULTRASONIC VARIABLES
// ====================================================================

float currentDistance = -1.0;

float previousDistance = -1.0;


// ====================================================================
// 16. VOLTAGE VARIABLES
// ====================================================================

int voltageADC = 0;

float sensorVoltage = 0.0;


// ====================================================================
// 17. VIBRATION VARIABLES
// ====================================================================

bool vibrationDetected = false;

bool previousVibration = false;


// ====================================================================
// 18. MPU VARIABLES
// ====================================================================

float roll = 0.0;

float pitch = 0.0;

float accelerationMagnitude = 0.0;

float gyroZ = 0.0;

bool mpuMovementDetected = false;

bool mpuTilted = false;

// Static incline (roll/pitch) status - separate from the gyro-based
// mpuTilted flag, which is about motion/tamper, not standing slope.
bool vehicleInclined = false;


// ====================================================================
// 19. GPS VARIABLES
// ====================================================================

double gpsLatitude = 0.0;

double gpsLongitude = 0.0;

bool gpsFixAvailable = false;


// ====================================================================
// 20. TRIP VARIABLES (NEW)
// ====================================================================

float tripStartFuel      = 0.0f;
float tripEndFuel        = 0.0f;
float tripFuelConsumed   = 0.0f;
float tripDistanceKm     = 0.0f;
float tripMileage        = -1.0f;   // -1 means "not available yet"

double startLatitude  = 0.0;
double startLongitude = 0.0;

double endLatitude  = 0.0;
double endLongitude = 0.0;

bool tripActive        = false;
bool tripStartGPSValid = false;
bool tripEndGPSValid   = false;

unsigned long tripStartTime = 0;
unsigned long tripEndTime   = 0;

// Rolling "last accepted GPS point" used for live distance accumulation
double lastTripLat = 0.0;
double lastTripLon = 0.0;
bool   lastTripPointValid = false;
unsigned long lastGpsPointTime = 0;


// ====================================================================
// 21. TIMERS
// ====================================================================

unsigned long lastSensorTime = 0;

unsigned long lastDashboardTime = 0;

unsigned long refuelingStartTime = 0;


// ====================================================================
// 22. FLAGS
// ====================================================================

bool suspiciousActivity = false;

bool refuelingMonitoring = false;

bool theftMonitoring = false;


// ====================================================================
// 23. FUNCTION PROTOTYPES
// ====================================================================

float readDistance();

float getFilteredDistance();

float calculateLitres(float distance);

float readFuelLevel();

bool readVehicleState();

void updateMPU();

void updateVibration();

void updateGPS();

double haversineKm(double lat1, double lon1, double lat2, double lon2);

void updateTripDistance();

void startTrip();

void stopTrip();

void updateTheftAlarm();

void processVehicleState();

void processVehicleON();

void processVehicleOFF();

void processRefuelingDetection();

void printDashboard();

void printGPSLocation();

void printTheftAlert();

void printRefuelingAlert();

void printSystemState();

void printTripStartedBanner();

void printTripStoppedBanner();



// ====================================================================
// 24. WIFI WEB DASHBOARD (STA MODE)
// ====================================================================

WebServer server(80);

// Wi-Fi router credentials
const char* WIFI_SSID = "Siva";
const char* WIFI_PASSWORD = "1234ABCD";

unsigned long lastWiFiPrint = 0;

String stateToString(SystemState s)
{
    switch (s)
    {
        case STATE_VEHICLE_ON:  return "VEHICLE ON";
        case STATE_SUSPICIOUS:  return "SUSPICIOUS ACTIVITY";
        case STATE_REFUELING:   return "REFUELING";
        case STATE_THEFT:       return "THEFT CONFIRMED";
        default:                return "VEHICLE OFF";
    }
}

String jsonBool(bool v) { return v ? "true" : "false"; }

void printWiFiStatus()
{
    Serial.println();
    Serial.println("========================================");
    Serial.println("          FuelGuard Wi-Fi Status");
    Serial.println("========================================");

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("SSID        : ");
        Serial.println(WiFi.SSID());

        Serial.print("IP Address  : ");
        Serial.println(WiFi.localIP());

        Serial.print("RSSI        : ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");

        Serial.println("Status      : CONNECTED");
    }
    else
    {
        Serial.println("Status      : DISCONNECTED");
    }

    Serial.println("========================================");
}

String makeJson()
{
    String j = "{";
    j += "\"vehicleOn\":" + jsonBool(vehicleOn) + ",";
    j += "\"state\":\"" + stateToString(currentState) + "\",";
    j += "\"fuel\":" + String(currentFuelLevel, 2) + ",";
    j += "\"distanceCm\":" + String(currentDistance, 2) + ",";
    j += "\"sensorVoltage\":" + String(sensorVoltage, 3) + ",";
    j += "\"voltageADC\":" + String(voltageADC) + ",";
    j += "\"vibration\":" + jsonBool(vibrationDetected) + ",";
    j += "\"mpuAvailable\":" + jsonBool(mpuAvailable) + ",";
    j += "\"roll\":" + String(roll, 2) + ",";
    j += "\"pitch\":" + String(pitch, 2) + ",";
    j += "\"acceleration\":" + String(accelerationMagnitude, 2) + ",";
    j += "\"gyroZ\":" + String(gyroZ, 2) + ",";
    j += "\"mpuMovement\":" + jsonBool(mpuMovementDetected) + ",";
    j += "\"mpuTilted\":" + jsonBool(mpuTilted) + ",";
    j += "\"inclined\":" + jsonBool(vehicleInclined) + ",";
    j += "\"inclineThreshold\":" + String(INCLINE_THRESHOLD_DEG, 1) + ",";
    j += "\"gpsFix\":" + jsonBool(gpsFixAvailable) + ",";
    j += "\"latitude\":" + String(gpsLatitude, 6) + ",";
    j += "\"longitude\":" + String(gpsLongitude, 6) + ",";
    j += "\"fuelAdded\":" + String(fuelAdded, 2) + ",";
    j += "\"fuelLost\":" + String(fuelLost, 2) + ",";
    j += "\"suspicious\":" + jsonBool(suspiciousActivity) + ",";
    j += "\"refueling\":" + jsonBool(refuelingMonitoring) + ",";
    // "theft" and "refuelConfirmed" are PERSISTED event flags - they
    // stay true (driving the buzzer/LED and the dashboard Event
    // Status block) until the vehicle is next started, unlike
    // currentState which gets re-evaluated every sensor cycle.
    j += "\"theft\":" + jsonBool(theftAlarmActive) + ",";
    j += "\"refuelConfirmed\":" + jsonBool(refuelEventActive) + ",";

    // ---- NEW: trip fields ----
    j += "\"tripActive\":" + jsonBool(tripActive) + ",";
    j += "\"tripStartFuel\":" + String(tripStartFuel, 2) + ",";
    j += "\"tripEndFuel\":" + String(tripEndFuel, 2) + ",";
    j += "\"tripFuelConsumed\":" + String(tripFuelConsumed, 2) + ",";
    j += "\"tripDistanceKm\":" + String(tripDistanceKm, 2) + ",";
    j += "\"tripMileage\":" + String(tripMileage, 2) + ",";
    j += "\"startGpsValid\":" + jsonBool(tripStartGPSValid) + ",";
    j += "\"endGpsValid\":" + jsonBool(tripEndGPSValid) + ",";
    j += "\"startLatitude\":" + String(startLatitude, 6) + ",";
    j += "\"startLongitude\":" + String(startLongitude, 6) + ",";
    j += "\"endLatitude\":" + String(endLatitude, 6) + ",";
    j += "\"endLongitude\":" + String(endLongitude, 6) + ",";

    j += "\"uptime\":" + String(millis());
    j += "}";
    return j;
}

const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>FuelGuard+</title>
<style>
:root{
  --yellow:#FFC107; --amber:#FF8F00; --orange:#FF6F00; --deep:#E65100;
  --cream:#FFF8E1; --white:#FFFFFF; --ink:#2b1d00; --ink2:#5a4300;
  --ok:#2e7d32; --warn:#e65100; --bad:#c62828; --steel:#3a3a3a;
}
*{box-sizing:border-box}
body{margin:0;font-family:'Segoe UI',Arial,sans-serif;color:var(--ink);
  background:
    radial-gradient(1200px 500px at 10% -10%, #fff8dd 0%, transparent 60%),
    radial-gradient(1000px 600px at 110% 0%, #ffe0b2 0%, transparent 55%),
    linear-gradient(180deg,#FFFDF5 0%, #FFECB3 35%, #FFCC80 70%, #FFA726 100%);
  min-height:100vh;
}
header{padding:16px 20px;background:linear-gradient(120deg,var(--deep),var(--orange) 55%,var(--amber));
  border-bottom:4px solid #4e2600;box-shadow:0 6px 18px #0004;position:relative;overflow:hidden}
header:after{content:'';position:absolute;inset:0;background:repeating-linear-gradient(-45deg,#ffffff22 0 14px,transparent 14px 28px);pointer-events:none}
h1{margin:0;font-size:28px;color:#fff;text-shadow:0 2px 4px #0006;letter-spacing:.5px}
small{color:#fff2d0}
.wrap{max-width:1200px;margin:auto;padding:16px;position:relative;z-index:1}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:14px}
.card{background:linear-gradient(160deg,#ffffff,#fff6df);border:2px solid #ffd77a;border-radius:18px;
  padding:18px;box-shadow:0 10px 24px #7a4a0026, inset 0 1px 0 #fff;position:relative;overflow:hidden}
.card:before{content:'';position:absolute;top:0;left:0;right:0;height:5px;
  background:linear-gradient(90deg,var(--yellow),var(--orange),var(--deep))}
.label{color:var(--ink2);font-size:12px;text-transform:uppercase;font-weight:700;letter-spacing:.8px}
.value{font-size:30px;font-weight:800;margin-top:8px;color:var(--ink)}
.ok{color:var(--ok)}.warn{color:var(--warn)}.bad{color:var(--bad)}
.bar{height:14px;background:#f1dcae;border-radius:20px;overflow:hidden;margin-top:12px;border:1px solid #e0b45c}
.fill{height:100%;width:0;background:linear-gradient(90deg,#ffca28,#ff8f00);transition:.5s}
.row{display:flex;justify-content:space-between;gap:12px;margin:9px 0}
.status{font-weight:800;padding:8px 12px;border-radius:10px;background:#fff2cc;color:var(--ink);
  display:inline-block;border:1px solid #ffcf6b}
.top{display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap}
.btn{display:inline-block;padding:10px 14px;border-radius:10px;
  background:linear-gradient(160deg,#ffd54f,#ff8f00);color:#3a1e00;font-weight:700;text-decoration:none;
  margin:4px 6px 0 0;font-size:13px;cursor:pointer;border:1px solid #b45f00;box-shadow:0 3px 0 #b45f00}
.btn:active{transform:translateY(2px);box-shadow:0 1px 0 #b45f00}
.danger{background:linear-gradient(160deg,#ff7043,#c62828);color:#fff;border-color:#7f0000}
.muted{color:#8a6d2a}
.gps{word-break:break-all;color:var(--ink2);font-weight:600}
.langbtn{background:#fff;border:1px solid #ffcf6b;color:var(--ink)}
.trip{display:flex;justify-content:space-between;flex-wrap:wrap;gap:10px}
.trip .item{min-width:110px}
.hist{max-height:260px;overflow-y:auto}
.histrow{border-bottom:1px dashed #e0b45c;padding:8px 0;font-size:13px;color:var(--ink2)}

/* ---------------- 3D LORRY ANIMATION ---------------- */
.stage{position:relative;height:190px;perspective:900px;overflow:hidden;border-radius:16px;
  background:linear-gradient(180deg,#fff3d6 0%,#ffe0a3 55%,#ffb74d 100%);border:2px solid #ffcf6b}
.sky-glow{position:absolute;top:10px;right:16px;width:60px;height:60px;border-radius:50%;
  background:radial-gradient(circle,#fff6d0,#ffd54f 60%,transparent 70%);filter:blur(1px)}
.road{position:absolute;left:0;right:0;bottom:0;height:46px;background:
  linear-gradient(180deg,#5a5a5a,#2b2b2b);border-top:3px solid #ffca28}
.road:before{content:'';position:absolute;top:20px;left:0;right:0;height:5px;
  background:repeating-linear-gradient(90deg,#fff 0 30px,transparent 30px 60px);
  background-size:60px 5px;animation:roadmove 1.1s linear infinite;animation-play-state:paused}
.stage.moving .road:before{animation-play-state:running}
@keyframes roadmove{from{background-position:0 0}to{background-position:-60px 0}}
.truck{position:absolute;bottom:44px;left:40%;transform-style:preserve-3d;
  transform:translateX(-50%) rotateX(6deg) rotateY(-8deg);transition:transform .4s}
.stage.moving .truck{animation:bounce .5s ease-in-out infinite alternate}
@keyframes bounce{from{transform:translateX(-50%) rotateX(6deg) rotateY(-8deg) translateY(0)}
  to{transform:translateX(-50%) rotateX(6deg) rotateY(-8deg) translateY(-3px)}}
.cab{width:54px;height:46px;background:linear-gradient(160deg,#ffca28,#ff6f00);
  border-radius:8px 4px 4px 2px;position:relative;box-shadow:0 6px 10px #0004,inset 0 2px 0 #fff8;
  border:2px solid #b45f00}
.cab:before{content:'';position:absolute;top:6px;left:8px;width:20px;height:16px;
  background:linear-gradient(160deg,#bbeeff,#68c9e8);border-radius:3px;border:1px solid #0006}
.trailer{position:absolute;left:-92px;top:6px;width:92px;height:40px;
  background:linear-gradient(160deg,#fff,#ffe4b3);border:2px solid #b45f00;border-radius:4px 2px 2px 4px;
  box-shadow:0 6px 10px #0004,inset 0 2px 0 #fff}
.trailer:before{content:'⚡ FUELGUARD+';position:absolute;inset:0;display:flex;align-items:center;
  justify-content:center;font-size:9px;font-weight:800;color:#c62828;letter-spacing:.5px}
.wheel{position:absolute;bottom:-14px;width:22px;height:22px;border-radius:50%;
  background:radial-gradient(circle at 35% 35%,#666,#111 70%);border:3px solid #000;
  box-shadow:0 0 0 2px #ccc inset}
.wheel:before{content:'';position:absolute;inset:4px;border-radius:50%;
  background:repeating-conic-gradient(#999 0deg 30deg,#333 30deg 60deg)}
.w1{left:-72px}.w2{left:-28px}.w3{left:16px}
.stage.moving .wheel{animation:spin .5s linear infinite}
@keyframes spin{to{transform:rotate(360deg)}}
.headlamp{position:absolute;right:-4px;top:16px;width:8px;height:8px;border-radius:50%;
  background:#eee;box-shadow:0 0 4px 2px #eee}
.stage.on .headlamp{background:#fff59d;box-shadow:0 0 14px 6px #ffe57f}
.exhaust{position:absolute;left:-96px;top:-2px;width:6px;height:6px;border-radius:50%;
  background:#cfcfcf;opacity:0}
.stage.moving .exhaust{animation:puff 1s ease-out infinite}
@keyframes puff{0%{opacity:.7;transform:translate(0,0) scale(.6)}100%{opacity:0;transform:translate(-14px,-16px) scale(1.6)}}
.beacon{position:absolute;top:2px;left:16px;width:10px;height:10px;border-radius:50%;background:#c62828}
.stage.theft .beacon{animation:blink .4s steps(2) infinite;box-shadow:0 0 12px 6px #ff5252}
@keyframes blink{50%{opacity:.15}}

/* ---------------- FUEL GAUGE ---------------- */
.gauge{width:150px;height:150px;border-radius:50%;margin:10px auto 0;position:relative;
  background:conic-gradient(#c62828 0deg, var(--orange) 60deg, var(--yellow) 160deg, #ddd 0deg);
  box-shadow:inset 0 0 0 10px #fff, 0 6px 14px #0003;display:flex;align-items:center;justify-content:center}
.gauge .hole{width:104px;height:104px;border-radius:50%;background:#fff8e1;
  display:flex;flex-direction:column;align-items:center;justify-content:center;
  border:2px solid #ffcf6b;box-shadow:inset 0 0 10px #0001}
.gauge .lval{font-size:24px;font-weight:800;color:var(--ink)}
.gauge .lsub{font-size:10px;color:var(--ink2);text-transform:uppercase;font-weight:700}

/* ---------------- INCLINE / TILT GAUGE ---------------- */
.tiltbox{width:130px;height:130px;border-radius:50%;margin:10px auto 0;position:relative;
  background:radial-gradient(circle,#fff,#ffe9b8 70%,#ffca28);border:3px solid #ffcf6b;
  box-shadow:inset 0 0 12px #0002}
.tiltbox .cross{position:absolute;inset:0}
.tiltbox .cross:before,.tiltbox .cross:after{content:'';position:absolute;background:#e0b45c}
.tiltbox .cross:before{left:50%;top:8px;bottom:8px;width:1px}
.tiltbox .cross:after{top:50%;left:8px;right:8px;height:1px}
.bubble{position:absolute;width:26px;height:26px;border-radius:50%;left:50%;top:50%;
  background:radial-gradient(circle at 35% 30%,#8bc34a,#2e7d32);
  box-shadow:0 3px 6px #0004;transform:translate(-50%,-50%);transition:transform .3s}
.tiltbox.bad .bubble{background:radial-gradient(circle at 35% 30%,#ff7043,#c62828)}

@media(max-width:600px){h1{font-size:22px}.value{font-size:25px}.stage{height:160px}}

/* ---------------- LOGIN SCREEN ---------------- */
#loginScreen{position:fixed;inset:0;z-index:50;display:flex;align-items:center;justify-content:center;
  background:linear-gradient(160deg,#FF8F00,#FFC107 55%,#FFECB3);padding:16px}
.loginCard{background:#fffdf6;border:3px solid #ffcf6b;border-radius:20px;padding:32px 28px;
  width:100%;max-width:340px;text-align:center;box-shadow:0 16px 40px #0004}
.loginCard h2{margin:6px 0 2px;color:var(--ink)}
.loginCard p{color:var(--ink2);font-size:13px;margin-top:2px}
.loginCard input{width:100%;padding:11px 12px;margin-top:12px;border-radius:10px;
  border:2px solid #ffcf6b;background:#fff;font-size:15px}
.loginCard .btn{width:100%;margin-top:16px;padding:12px;font-size:15px}
.loginErr{color:var(--bad);font-size:13px;margin-top:10px;min-height:16px;font-weight:700}
.loginTruck{font-size:44px}

/* ---------------- THEFT BLINK ---------------- */
@keyframes pageBlink{0%,100%{background-color:transparent}50%{background-color:#c6282855}}
body.theft-alert{animation:pageBlink .6s steps(1) infinite}
@keyframes secBlink{0%,100%{background:#ffe0e0;color:#c62828}50%{background:#c62828;color:#fff}}
#security.blink{animation:secBlink .5s steps(1) infinite;border-radius:8px;padding:4px 8px}
</style></head><body>

<div id='loginScreen'>
  <div class='loginCard'>
    <div class='loginTruck'>🚛</div>
    <h2>FuelGuard+</h2>
    <p>Driver Login • ECE Embedded System</p>
    <input id='loginUser' placeholder='Username' autocomplete='username'>
    <input id='loginPass' type='password' placeholder='Password' autocomplete='current-password'>
    <button class='btn' onclick='doLogin()'>LOGIN</button>
    <div class='loginErr' id='loginErr'></div>
  </div>
</div>

<header><div class='wrap top'><div><h1>🚛 FuelGuard+</h1><small id='subtitle'>ECE Embedded System • Fuel Security &amp; Mileage Monitor</small></div>
<div style='display:flex;align-items:center;gap:8px'><button class='btn langbtn' onclick="setLang('en')">EN</button><button class='btn langbtn' onclick="setLang('ta')">தமிழ்</button><button class='btn langbtn' onclick='doLogout()'>Logout</button><div id='net' class='status'>Connecting...</div></div>
</div></header>
<main class='wrap' id='appMain' style='display:none'>

<div class='card' style='margin-bottom:14px'>
  <div class='label'>Live Lorry View</div>
  <div class='stage' id='stage'>
    <div class='sky-glow'></div>
    <div class='road'></div>
    <div class='truck'>
      <div class='beacon'></div>
      <div class='trailer'></div>
      <div class='cab'><div class='headlamp'></div></div>
      <div class='exhaust'></div>
      <div class='wheel w1'></div>
      <div class='wheel w2'></div>
      <div class='wheel w3'></div>
    </div>
  </div>
</div>

<div class='grid'>
<div class='card'><div class='label' data-i='vehicle'>Vehicle</div><div id='vehicle' class='value'>--</div><div id='state' class='status'>--</div></div>
<div class='card' style='text-align:center'><div class='label' data-i='fuelLevel'>Fuel Level</div>
  <div class='gauge' id='fuelGauge'><div class='hole'><div id='fuel' class='lval'>-- L</div><div class='lsub'>/ 20 L</div></div></div>
</div>
<div class='card'><div class='label'>Voltage Sensor</div><div id='volt' class='value'>-- V</div><small>ADC: <span id='adc'>--</span></small></div>
<div class='card'><div class='label' data-i='security'>Security</div><div id='security' class='value'>--</div><small>Vibration: <span id='vib'>--</span></small></div>
</div><br><div class='grid'>
<div class='card' style='text-align:center'>
  <div class='label' data-i='liveMileage'>Live Mileage (GPS)</div>
  <div id='liveMileage' class='value' style='margin-top:10px'>--</div>
  <small class='muted'><span data-i='basedOnGps'>Based on GPS distance ÷ fuel used</span></small>
  <div style='margin-top:8px'><small class='muted'>Distance: <span id='liveDist'>--</span> km &nbsp; Fuel Used: <span id='liveFuelUsed'>--</span> L</small></div>
</div>
<div class='card' style='text-align:center'>
  <div class='label' data-i='inclineTitle'>Vehicle Incline</div>
  <div class='tiltbox' id='tiltBox'><div class='cross'></div><div class='bubble' id='bubble'></div></div>
  <div id='inclineStatus' class='status' style='margin-top:10px'>--</div>
  <small class='muted'>Roll <span id='roll'>--</span>° &nbsp; Pitch <span id='pitch'>--</span>°</small>
</div>
<div class='card'><div class='label'>Gyro Z (Tamper)</div><div id='gyro' class='value'>-- rad/s</div></div>
</div><br>

<div class='card' id='eventCard'>
<div class='label' data-i='eventTitle'>⚠ Event Status — Refueling or Theft</div>
<div id='eventBadge' class='status' style='margin-top:8px;font-size:16px'>--</div>
<div id='eventDetail' class='muted' style='margin-top:10px'>--</div>
</div><br>

<div class='card' id='tripCard'>
<div class='label' data-i='tripTitle'>🚛 Trip Performance</div>
<div id='tripHeadline' class='status' style='margin-top:6px'>--</div>
<div class='trip' style='margin-top:14px'>
<div class='item'><div class='label' data-i='initFuel'>Initial Fuel</div><div id='tInitFuel' class='value' style='font-size:20px'>-- L</div></div>
<div class='item'><div class='label' data-i='curFuel'>Current / Final Fuel</div><div id='tCurFuel' class='value' style='font-size:20px'>-- L</div></div>
<div class='item'><div class='label' data-i='fuelUsed'>Fuel Consumed</div><div id='tFuelUsed' class='value' style='font-size:20px'>-- L</div></div>
<div class='item'><div class='label' data-i='dist2'>Distance Travelled</div><div id='tDist' class='value' style='font-size:20px'>-- km</div></div>
<div class='item'><div class='label' data-i='mileage'>Estimated Mileage</div><div id='tMileage' class='value' style='font-size:20px'>--</div></div>
</div>
<div style='margin-top:14px'>
<div class='label' data-i='startLoc'>Start Location</div><div id='startLoc' class='gps'>--</div>
<div class='label' style='margin-top:8px' data-i='endLoc'>End Location</div><div id='endLoc' class='gps'>--</div>
<div style='margin-top:10px'>
<a id='mapStart' class='btn' target='_blank' data-i='openStart'>OPEN START LOCATION</a>
<a id='mapEnd' class='btn' target='_blank' data-i='openEnd'>OPEN END LOCATION</a>
<a id='mapCurrent' class='btn' target='_blank' data-i='openCurrent'>OPEN CURRENT LOCATION</a>
</div>
</div>
</div>
<br>

<div class='grid'>
<div class='card'><div class='label'>GPS</div><div id='gps' class='gps'>--</div></div>
<div class='card'><div class='label'>Fuel Event</div><div>Added: <b id='added'>0</b> L</div><div>Lost: <b id='lost'>0</b> L</div></div>
<div class='card'><div class='label'>System</div><div>Uptime: <b id='up'>0</b> s</div><div>MPU: <b id='mpu'>--</b></div></div>
<div class='card'><div class='label'>Web Access</div><div class='muted'>Connect your phone/laptop to the same Wi-Fi network and open the ESP32 IP address.</div></div>
</div><br>

<div class='card'>
<div class='label' data-i='tripHistory'>Trip History</div>
<div id='histList' class='hist' style='margin-top:10px'></div>
</div>

</main>
<script>
const $=id=>document.getElementById(id);function set(id,v){$(id).textContent=v}

const I18N = {
en:{vehicle:'Vehicle',fuelLevel:'Fuel Level',security:'Security',inclineTitle:'Vehicle Incline',tripTitle:'🚛 Trip Performance',initFuel:'Initial Fuel',curFuel:'Current / Final Fuel',fuelUsed:'Fuel Consumed',dist2:'Distance Travelled',mileage:'Estimated Mileage',startLoc:'Start Location',endLoc:'End Location',openStart:'OPEN START LOCATION',openEnd:'OPEN END LOCATION',openCurrent:'OPEN CURRENT LOCATION',tripHistory:'Trip History',running:'🚛 VEHICLE RUNNING',parked:'🅿 VEHICLE PARKED',noFix:'No GPS fix',calculating:'CALCULATING',na:'N/A',level:'LEVEL',inclined:'INCLINED (on slope)',liveMileage:'Live Mileage (GPS)',basedOnGps:'Based on GPS distance ÷ fuel used',normal:'NORMAL',suspicious:'SUSPICIOUS - MONITORING',theft:'THEFT CONFIRMED',vehOn:'VEHICLE ON',vehOff:'VEHICLE OFF',eventTitle:'⚠ Event Status — Refueling or Theft',eventTheft:'🚨 THEFT CONFIRMED',eventRefuel:'⛽ REFUELING CONFIRMED',eventNone:'No event - monitoring normally'},
ta:{vehicle:'வாகனம்',fuelLevel:'எரிபொருள் அளவு',security:'பாதுகாப்பு',inclineTitle:'வாகன சாய்வு',tripTitle:'🚛 பயண செயல்திறன்',initFuel:'ஆரம்ப எரிபொருள்',curFuel:'தற்போதைய / இறுதி எரிபொருள்',fuelUsed:'பயன்படுத்திய எரிபொருள்',dist2:'பயணித்த தூரம்',mileage:'மைலேஜ்',startLoc:'தொடக்க இருப்பிடம்',endLoc:'முடிவு இருப்பிடம்',openStart:'தொடக்க இருப்பிடத்தை திற',openEnd:'முடிவு இருப்பிடத்தை திற',openCurrent:'தற்போதைய இருப்பிடத்தை திற',tripHistory:'பயண வரலாறு',running:'🚛 வாகனம் இயங்குகிறது',parked:'🅿 வாகனம் நிறுத்தப்பட்டது',noFix:'GPS சிக்னல் இல்லை',calculating:'கணக்கிடுகிறது',na:'இல்லை',level:'சமமானது',inclined:'சாய்வில் உள்ளது',liveMileage:'நேரடி மைலேஜ் (GPS)',basedOnGps:'GPS தூரம் ÷ பயன்படுத்திய எரிபொருள்',normal:'சாதாரணம்',suspicious:'சந்தேகம் - கண்காணிப்பு',theft:'திருட்டு உறுதி செய்யப்பட்டது',vehOn:'வாகனம் இயங்குகிறது',vehOff:'வாகனம் நிறுத்தப்பட்டது',eventTitle:'⚠ நிகழ்வு நிலை — எரிபொருள் நிரப்புதல் அல்லது திருட்டு',eventTheft:'🚨 திருட்டு உறுதி செய்யப்பட்டது',eventRefuel:'⛽ எரிபொருள் நிரப்புதல் உறுதி செய்யப்பட்டது',eventNone:'நிகழ்வு இல்லை - சாதாரண கண்காணிப்பு'}
};
let currentLang = localStorage.getItem('fg_lang') || 'en';

function applyLang(){
  const t = I18N[currentLang];
  document.querySelectorAll('[data-i]').forEach(el=>{
    const key = el.getAttribute('data-i');
    if(t[key]) el.textContent = t[key];
  });
}
function setLang(l){ currentLang = l; localStorage.setItem('fg_lang', l); applyLang(); }

// ---------------- LOGIN ----------------
const DRIVER_USER = 'driver';
const DRIVER_PASS = '1234';

function showApp(){
  $('loginScreen').style.display = 'none';
  $('appMain').style.display = 'block';
}
function doLogin(){
  const u = $('loginUser').value.trim();
  const p = $('loginPass').value;
  if(u === DRIVER_USER && p === DRIVER_PASS){
    sessionStorage.setItem('fg_auth','ok');
    $('loginErr').textContent = '';
    showApp();
  } else {
    $('loginErr').textContent = 'Invalid username or password';
  }
}
function doLogout(){
  sessionStorage.removeItem('fg_auth');
  $('appMain').style.display = 'none';
  $('loginScreen').style.display = 'flex';
  $('loginUser').value=''; $('loginPass').value='';
}
document.addEventListener('keydown', e=>{
  if(e.key==='Enter' && $('loginScreen').style.display !== 'none') doLogin();
});
if(sessionStorage.getItem('fg_auth') === 'ok'){ showApp(); }

function mapsUrl(lat,lon){ return 'https://maps.google.com/?q='+lat+','+lon; }

function loadHistory(){
  try{ return JSON.parse(localStorage.getItem('fg_trip_history')||'[]'); }catch(e){ return []; }
}
function saveHistory(list){
  if(list.length>50) list = list.slice(list.length-50);
  localStorage.setItem('fg_trip_history', JSON.stringify(list));
}
function renderHistory(){
  const list = loadHistory();
  const box = $('histList');
  if(list.length===0){ box.innerHTML = '<div class="muted">No trips recorded yet.</div>'; return; }
  let html='';
  for(let i=list.length-1;i>=0;i--){
    const t = list[i];
    html += '<div class="histrow"><b>'+t.label+'</b><br>'
      + t.startTime+' &rarr; '+t.endTime+'<br>'
      + 'Fuel: '+t.initFuel.toFixed(1)+'L &rarr; '+t.finalFuel.toFixed(1)+'L (used '+t.consumed.toFixed(1)+'L)<br>'
      + 'Distance: '+t.distance.toFixed(1)+' km &nbsp; Mileage: '+(t.mileage>0?t.mileage.toFixed(1)+' km/L':'N/A')
      + '</div>';
  }
  box.innerHTML = html;
}

let lastTripActive = null;
let tripCounter = parseInt(localStorage.getItem('fg_trip_counter')||'0');

function recordTripIfJustEnded(d){
  if(lastTripActive === true && d.tripActive === false){
    tripCounter++;
    localStorage.setItem('fg_trip_counter', tripCounter);
    const list = loadHistory();
    list.push({
      label: 'TRIP #' + String(tripCounter).padStart(2,'0'),
      startTime: new Date(Date.now() - (d.uptime)).toLocaleTimeString(),
      endTime: new Date().toLocaleTimeString(),
      initFuel: d.tripStartFuel,
      finalFuel: d.tripEndFuel,
      consumed: d.tripFuelConsumed,
      distance: d.tripDistanceKm,
      mileage: d.tripMileage
    });
    saveHistory(list);
    renderHistory();
  }
  lastTripActive = d.tripActive;
}

async function update(){try{let r=await fetch('/api',{cache:'no-store'});let d=await r.json();

// Vehicle card: ONLY ever shows ON/OFF here - never "suspicious"
set('vehicle',d.vehicleOn?'ON':'OFF');
set('state', d.vehicleOn ? I18N[currentLang].vehOn : I18N[currentLang].vehOff);

set('fuel',d.fuel.toFixed(1)+' L');
const pct=Math.max(0,Math.min(100,d.fuel/20*100));
$('fuelGauge').style.background='conic-gradient(#c62828 0deg,'+(pct<20?'#c62828':'#ff8f00')+' '+(pct*1.2)+'deg,#ddd '+(pct*3.6)+'deg,#ddd 360deg)';
set('volt',d.sensorVoltage.toFixed(2)+' V');set('adc',d.voltageADC);

// Security card: this is the ONLY place suspicious/theft is shown,
// and it blinks (card + full page) once theft is confirmed.
set('security', d.theft ? I18N[currentLang].theft : (d.suspicious ? I18N[currentLang].suspicious : I18N[currentLang].normal));
$('security').className='value '+(d.theft?'bad blink':(d.suspicious?'warn':'ok'));
document.body.classList.toggle('theft-alert', !!d.theft);

set('vib',d.vibration?'DETECTED':'CLEAR');
set('roll',d.roll.toFixed(1));set('pitch',d.pitch.toFixed(1));
set('gyro',d.gyroZ.toFixed(3));
set('added',d.fuelAdded.toFixed(2));set('lost',d.fuelLost.toFixed(2));
set('up',(d.uptime/1000).toFixed(0));set('mpu',d.mpuAvailable?'CONNECTED':'NOT FOUND');
if(d.gpsFix){set('gps',d.latitude.toFixed(6)+', '+d.longitude.toFixed(6));}else{set('gps',I18N[currentLang].noFix);}
$('vehicle').className='value '+(d.vehicleOn?'ok':'warn');
$('net').textContent='Live • ESP32 Wi-Fi';

// Live GPS-based mileage: while the vehicle is ON and fuel is
// dropping, mileage = GPS distance travelled so far / fuel used so far.
{
  const liveUsed = d.vehicleOn ? Math.max(0,(d.tripStartFuel - d.fuel)) : d.tripFuelConsumed;
  const liveDist = d.tripDistanceKm;
  set('liveDist', liveDist.toFixed(2));
  set('liveFuelUsed', liveUsed.toFixed(2));
  if(liveDist>0 && liveUsed>0){ set('liveMileage', (liveDist/liveUsed).toFixed(2)+' km/L'); }
  else{ set('liveMileage', I18N[currentLang].calculating); }
}

// Event Status block - explicitly says whether the last confirmed
// event was a REFUELING or a THEFT (independent of the Security card).
{
  const badge = $('eventBadge');
  const detail = $('eventDetail');
  if(d.theft){
    badge.textContent = I18N[currentLang].eventTheft;
    badge.className = 'status bad blink';
    detail.textContent = 'Fuel Lost: ' + d.fuelLost.toFixed(2) + ' L';
  } else if(d.refuelConfirmed){
    badge.textContent = I18N[currentLang].eventRefuel;
    badge.className = 'status ok';
    detail.textContent = 'Fuel Added: ' + d.fuelAdded.toFixed(2) + ' L';
  } else {
    badge.textContent = I18N[currentLang].eventNone;
    badge.className = 'status';
    detail.textContent = '';
  }
}

// Incline bubble - shift by roll/pitch, clamped visually
const bx = Math.max(-40,Math.min(40, d.roll*2));
const by = Math.max(-40,Math.min(40, d.pitch*2));
$('bubble').style.transform = 'translate(calc(-50% + '+bx+'px), calc(-50% + '+by+'px))';
$('tiltBox').className = 'tiltbox' + (d.inclined?' bad':'');
set('inclineStatus', d.inclined ? I18N[currentLang].inclined : I18N[currentLang].level);
$('inclineStatus').className = 'status ' + (d.inclined?'bad':'ok');

// 3D lorry stage state
const stage = $('stage');
stage.classList.toggle('on', d.vehicleOn);
stage.classList.toggle('moving', d.vehicleOn && d.tripActive);
stage.classList.toggle('theft', d.theft);

// Trip card
$('tripHeadline').textContent = d.tripActive ? I18N[currentLang].running : I18N[currentLang].parked;
set('tInitFuel', (d.tripStartFuel||0).toFixed(2)+' L');
set('tCurFuel', (d.tripActive ? d.fuel : d.tripEndFuel).toFixed(2)+' L');
set('tFuelUsed', d.tripActive ? Math.max(0,(d.tripStartFuel-d.fuel)).toFixed(2)+' L' : d.tripFuelConsumed.toFixed(2)+' L');
set('tDist', d.tripDistanceKm.toFixed(2)+' km');
if(d.tripDistanceKm<=0 || (d.tripActive? Math.max(0,(d.tripStartFuel-d.fuel)) : d.tripFuelConsumed)<=0){ set('tMileage', I18N[currentLang].calculating); }
else if(!d.tripActive && d.tripMileage<=0){ set('tMileage', I18N[currentLang].na); }
else if(d.tripActive){
  const used = Math.max(0,(d.tripStartFuel-d.fuel));
  set('tMileage', used>0 ? (d.tripDistanceKm/used).toFixed(2)+' km/L' : I18N[currentLang].calculating);
}else{ set('tMileage', d.tripMileage.toFixed(2)+' km/L'); }

if(d.startGpsValid){ set('startLoc', d.startLatitude.toFixed(6)+', '+d.startLongitude.toFixed(6)); $('mapStart').href=mapsUrl(d.startLatitude,d.startLongitude); }
else { set('startLoc', I18N[currentLang].noFix); $('mapStart').removeAttribute('href'); }

if(d.endGpsValid){ set('endLoc', d.endLatitude.toFixed(6)+', '+d.endLongitude.toFixed(6)); $('mapEnd').href=mapsUrl(d.endLatitude,d.endLongitude); }
else { set('endLoc', I18N[currentLang].noFix); $('mapEnd').removeAttribute('href'); }

if(d.gpsFix){ $('mapCurrent').href=mapsUrl(d.latitude,d.longitude); } else { $('mapCurrent').removeAttribute('href'); }

recordTripIfJustEnded(d);

if(d.theft && 'Notification' in window && Notification.permission==='granted'){if(!window.__fgTheft){new Notification('FuelGuard Theft Alert',{body:'Fuel theft confirmed. Check the lorry immediately.'});window.__fgTheft=true}}else if(!d.theft){window.__fgTheft=false}
}catch(e){$('net').textContent='Disconnected';}}

if('Notification'in window && Notification.permission==='default') Notification.requestPermission().catch(()=>{});
applyLang();
renderHistory();
update();setInterval(update,1000);
</script></body></html>
)rawliteral";

void handleRoot()
{
    server.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleApi()
{
    server.send(200, "application/json", makeJson());
}

void setupWebServer()
{
    // Connect ESP32 to an existing Wi-Fi router/hotspot
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);

    Serial.println();
    Serial.println("========================================");
    Serial.println("       FuelGuard Wi-Fi Connection");
    Serial.println("========================================");
    Serial.print("Connecting to: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;

    while (WiFi.status() != WL_CONNECTED && attempts < 40)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    Serial.println();

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("Wi-Fi CONNECTED!");
        Serial.print("SSID    : ");
        Serial.println(WiFi.SSID());
        Serial.print("IP      : ");
        Serial.println(WiFi.localIP());
        Serial.print("URL     : http://");
        Serial.println(WiFi.localIP());
        Serial.print("RSSI    : ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    }
    else
    {
        Serial.println("Wi-Fi CONNECTION FAILED");
        Serial.print("Status  : ");
        Serial.println(WiFi.status());
        Serial.println("Check Wi-Fi name/password and use a 2.4 GHz network.");
    }

    // Start web server even if Wi-Fi is not connected yet.
    // If Wi-Fi reconnects later, the server can be accessed normally.
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api", HTTP_GET, handleApi);
    server.begin();

    Serial.println("Web server started.");
    Serial.println("========================================");
}

// ====================================================================
// SETUP
// ====================================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);


    // ---------------------------------------------------------------
    // ULTRASONIC
    // ---------------------------------------------------------------

    pinMode(TRIG_PIN, OUTPUT);

    pinMode(ECHO_PIN, INPUT);


    // ---------------------------------------------------------------
    // VIBRATION
    // ---------------------------------------------------------------

    pinMode(
        VIBRATION_PIN,
        INPUT
    );


    // ---------------------------------------------------------------
    // VOLTAGE SENSOR
    // ---------------------------------------------------------------

    pinMode(
        VOLTAGE_PIN,
        INPUT
    );


    // ---------------------------------------------------------------
    // THEFT ALARM OUTPUTS (BUZZER + LED)
    // ---------------------------------------------------------------

    pinMode(
        BUZZER_PIN,
        OUTPUT
    );

    pinMode(
        ALARM_LED_PIN,
        OUTPUT
    );

    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(ALARM_LED_PIN, LOW);


    // ---------------------------------------------------------------
    // GPS
    // ---------------------------------------------------------------

    GPS_Serial.begin(
        9600,
        SERIAL_8N1,
        GPS_RX_PIN,
        GPS_TX_PIN
    );


    // ---------------------------------------------------------------
    // MPU6050
    // ---------------------------------------------------------------

    Wire.begin(
        MPU_SDA_PIN,
        MPU_SCL_PIN
    );


    if (mpu.begin())
    {
        mpuAvailable = true;

        mpu.setAccelerometerRange(
            MPU6050_RANGE_8_G
        );

        mpu.setGyroRange(
            MPU6050_RANGE_500_DEG
        );

        mpu.setFilterBandwidth(
            MPU6050_BAND_21_HZ
        );

        Serial.println(
            "MPU6050: CONNECTED"
        );
    }
    else
    {
        mpuAvailable = false;

        Serial.println(
            "MPU6050: NOT FOUND"
        );
    }


    // ---------------------------------------------------------------
    // BOOT SCREEN
    // ---------------------------------------------------------------

    Serial.println();
    Serial.println(
        "================================================"
    );

    Serial.println(
        "             FUELGUARD SYSTEM"
    );

    Serial.println(
        "             COMPLETE DEMO"
    );

    Serial.println(
        "================================================"
    );

    Serial.println();

    Serial.println(
        "Vehicle OFF : ~12 V -> sensor ~2.4 V"
    );

    Serial.println(
        "Vehicle ON  : ~14 V -> sensor ~2.8 V"
    );

    Serial.println(
        "Threshold   : 2.60 V"
    );

    Serial.println();

    Serial.println(
        "System Initializing..."
    );


    // ---------------------------------------------------------------
    // INITIAL VEHICLE STATE
    // ---------------------------------------------------------------

    vehicleOn =
        readVehicleState();

    previousVehicleOn =
        vehicleOn;


    // ---------------------------------------------------------------
    // INITIAL FUEL
    // ---------------------------------------------------------------

    delay(1000);

    currentFuelLevel =
        readFuelLevel();

    previousFuelLevel =
        currentFuelLevel;

    storedFuelReference =
        currentFuelLevel;


    Serial.println();

    Serial.print(
        "Initial Fuel Level : "
    );

    Serial.print(
        currentFuelLevel,
        1
    );

    Serial.println(
        " L"
    );


    // ---------------------------------------------------------------
    // BOOT STATE
    // ---------------------------------------------------------------

    if (vehicleOn)
    {
        currentState =
            STATE_VEHICLE_ON;

        Serial.println(
            "BOOT STATE : VEHICLE ON"
        );

        // If we boot with the vehicle already running, start a trip
        // using the best information available right now.
        startTrip();
    }
    else
    {
        currentState =
            STATE_VEHICLE_OFF;

        Serial.println(
            "BOOT STATE : VEHICLE OFF"
        );
    }


    Serial.println();

    Serial.println(
        "SYSTEM READY"
    );

    Serial.println(
        "================================================"
    );

    Serial.println();

    setupWebServer();
    printWiFiStatus();
}


// ====================================================================
// MAIN LOOP
// ====================================================================

void loop()
{
    server.handleClient();

    unsigned long now = millis();


    // Print Wi-Fi status every 5 seconds
    if (now - lastWiFiPrint >= 5000)
    {
        lastWiFiPrint = now;

        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("Wi-Fi disconnected. Reconnecting...");
            WiFi.reconnect();
        }

        printWiFiStatus();
    }

    // ---------------------------------------------------------------
    // GPS ALWAYS RUNS
    // ---------------------------------------------------------------

    updateGPS();


    // ---------------------------------------------------------------
    // THEFT ALARM (BUZZER + LED) - runs every loop, non-blocking
    // ---------------------------------------------------------------

    updateTheftAlarm();


    // ---------------------------------------------------------------
    // SENSOR PROCESSING
    // ---------------------------------------------------------------

    if (
        now - lastSensorTime >=
        SENSOR_INTERVAL_MS
    )
    {
        lastSensorTime =
            now;


        // Read vehicle state

        vehicleOn =
            readVehicleState();


        // MPU

        updateMPU();


        // Vibration

        updateVibration();


        // Process state transition

        processVehicleState();


        // Vehicle ON

        if (vehicleOn)
        {
            processVehicleON();
        }

        // Vehicle OFF

        else
        {
            processVehicleOFF();
        }


        previousVehicleOn =
            vehicleOn;
    }


    // ---------------------------------------------------------------
    // DASHBOARD
    // ---------------------------------------------------------------

    if (
        now - lastDashboardTime >=
        DASHBOARD_INTERVAL_MS
    )
    {
        lastDashboardTime =
            now;

        printDashboard();
    }
}


// ====================================================================
// VEHICLE VOLTAGE READING
// ====================================================================

bool readVehicleState()
{
    voltageADC =
        analogRead(VOLTAGE_PIN);


    sensorVoltage =
        (voltageADC / 4095.0) *
        3.3;


    // ---------------------------------------------------------------
    // HYSTERESIS
    // ---------------------------------------------------------------

    if (vehicleOn)
    {
        // Already ON

        if (
            sensorVoltage <
            VEHICLE_OFF_LEVEL_V
        )
        {
            return false;
        }

        return true;
    }
    else
    {
        // Currently OFF

        if (
            sensorVoltage >=
            VEHICLE_ON_LEVEL_V
        )
        {
            return true;
        }

        return false;
    }
}


// ====================================================================
// ULTRASONIC SINGLE READING
// ====================================================================

float readDistance()
{
    digitalWrite(
        TRIG_PIN,
        LOW
    );

    delayMicroseconds(2);


    digitalWrite(
        TRIG_PIN,
        HIGH
    );

    delayMicroseconds(10);


    digitalWrite(
        TRIG_PIN,
        LOW
    );


    long duration =
        pulseIn(
            ECHO_PIN,
            HIGH,
            30000
        );


    if (duration == 0)
    {
        return -1.0;
    }


    float distance =
        duration *
        0.0343 /
        2.0;


    return distance;
}


// ====================================================================
// FILTERED ULTRASONIC
// ====================================================================

float getFilteredDistance()
{
    float readings[ULTRASONIC_SAMPLES];

    int validCount = 0;


    for (
        int i = 0;
        i < ULTRASONIC_SAMPLES;
        i++
    )
    {
        float d =
            readDistance();


        if (
            d > 0 &&
            d < 400
        )
        {
            readings[validCount] =
                d;

            validCount++;
        }


        delay(
            ULTRASONIC_SAMPLE_DELAY_MS
        );
    }


    // No valid reading

    if (validCount == 0)
    {
        return -1.0;
    }


    // ---------------------------------------------------------------
    // SORT
    // ---------------------------------------------------------------

    for (
        int i = 1;
        i < validCount;
        i++
    )
    {
        float key =
            readings[i];

        int j =
            i - 1;


        while (
            j >= 0 &&
            readings[j] > key
        )
        {
            readings[j + 1] =
                readings[j];

            j--;
        }


        readings[j + 1] =
            key;
    }


    return readings[
        validCount / 2
    ];
}


// ====================================================================
// DISTANCE -> LITRES
// ====================================================================

float calculateLitres(float d)
{
    if (d < 0)
    {
        return currentFuelLevel;
    }


    // ---------------------------------------------------------------
    // FULL
    // ---------------------------------------------------------------

    if (d <= 2.88)
    {
        return 20.0;
    }


    // ---------------------------------------------------------------
    // 20 -> 15 L
    // ---------------------------------------------------------------

    else if (d <= 13.62)
    {
        return
            20.0 -
            (d - 2.88) *
            (
                5.0 /
                (13.62 - 2.88)
            );
    }


    // ---------------------------------------------------------------
    // 15 -> 10 L
    // ---------------------------------------------------------------

    else if (d <= 20.65)
    {
        return
            15.0 -
            (d - 13.62) *
            (
                5.0 /
                (20.65 - 13.62)
            );
    }


    // ---------------------------------------------------------------
    // 10 -> 5 L
    // ---------------------------------------------------------------

    else if (d <= 30.01)
    {
        return
            10.0 -
            (d - 20.65) *
            (
                5.0 /
                (30.01 - 20.65)
            );
    }


    // ---------------------------------------------------------------
    // 5 -> 3 L
    // ---------------------------------------------------------------

    else if (d <= 36.99)
    {
        return
            5.0 -
            (d - 30.01) *
            (
                2.0 /
                (36.99 - 30.01)
            );
    }


    // ---------------------------------------------------------------
    // BELOW 3 L
    // ---------------------------------------------------------------

    else
    {
        return 3.0;
    }
}


// ====================================================================
// READ FUEL LEVEL
// ====================================================================

float readFuelLevel()
{
    float distance =
        getFilteredDistance();


    if (distance < 0)
    {
        Serial.println(
            "ULTRASONIC: NO ECHO"
        );

        return currentFuelLevel;
    }


    currentDistance =
        distance;


    float litres =
        calculateLitres(
            distance
        );


    // Prevent impossible values

    if (litres < 0)
    {
        litres = 0;
    }


    if (litres > TANK_CAPACITY_L)
    {
        litres =
            TANK_CAPACITY_L;
    }


    return litres;
}


// ====================================================================
// MPU6050
// ====================================================================

void updateMPU()
{
    if (!mpuAvailable)
    {
        mpuMovementDetected =
            false;

        return;
    }


    sensors_event_t a;

    sensors_event_t g;

    sensors_event_t temp;


    mpu.getEvent(
        &a,
        &g,
        &temp
    );

    // ---------------------------------------------------------------
    // GYRO Z TILT DETECTION
    // ---------------------------------------------------------------
    gyroZ = g.gyro.z;

    if (gyroZ < -0.01)
    {
        mpuTilted = true;
    }
    else
    {
        mpuTilted = false;
    }


    // ---------------------------------------------------------------
    // ROLL
    // ---------------------------------------------------------------

    roll =
        atan2(
            a.acceleration.y,
            a.acceleration.z
        )
        *
        180.0 /
        PI;


    // ---------------------------------------------------------------
    // PITCH
    // ---------------------------------------------------------------

    pitch =
        atan2(
            -a.acceleration.x,
            sqrt(
                a.acceleration.y *
                a.acceleration.y
                +
                a.acceleration.z *
                a.acceleration.z
            )
        )
        *
        180.0 /
        PI;


    // ---------------------------------------------------------------
    // TOTAL ACCELERATION
    // ---------------------------------------------------------------

    accelerationMagnitude =
        sqrt(
            a.acceleration.x *
            a.acceleration.x
            +
            a.acceleration.y *
            a.acceleration.y
            +
            a.acceleration.z *
            a.acceleration.z
        );


    // ---------------------------------------------------------------
    // SIMPLE MOTION DETECTION
    // ---------------------------------------------------------------

    float deviation =
        abs(
            accelerationMagnitude - 9.81
        );


    if (deviation > 1.0)
    {
        mpuMovementDetected =
            true;
    }
    else
    {
        mpuMovementDetected =
            false;
    }


    // ---------------------------------------------------------------
    // STATIC INCLINE / SLOPE DETECTION
    // ---------------------------------------------------------------

    if (
        abs(roll) > INCLINE_THRESHOLD_DEG
        ||
        abs(pitch) > INCLINE_THRESHOLD_DEG
    )
    {
        vehicleInclined = true;
    }
    else
    {
        vehicleInclined = false;
    }
}


// ====================================================================
// VIBRATION SENSOR
// ====================================================================

void updateVibration()
{
    vibrationDetected =
        (
            digitalRead(
                VIBRATION_PIN
            )
            ==
            HIGH
        );
}


// ====================================================================
// GPS UPDATE
// ====================================================================

void updateGPS()
{
    while (
        GPS_Serial.available() > 0
    )
    {
        gps.encode(
            GPS_Serial.read()
        );
    }


    if (
        gps.location.isValid()
        &&
        gps.location.isUpdated()
    )
    {
        gpsLatitude =
            gps.location.lat();

        gpsLongitude =
            gps.location.lng();

        gpsFixAvailable =
            true;

        // Feed the new fix into the live trip-distance accumulator.
        updateTripDistance();
    }
}


// ====================================================================
// THEFT ALARM (BUZZER + LED)
// ====================================================================
//
// Non-blocking: blinks the LED and pulses the buzzer together while
// theftAlarmActive is true, using millis() instead of delay() so the
// web server stays responsive. Both outputs turn fully off as soon
// as theftAlarmActive clears (on the next vehicle start).
// ====================================================================

void updateTheftAlarm()
{
    if (!theftAlarmActive)
    {
        if (alarmOutputState)
        {
            digitalWrite(BUZZER_PIN, LOW);
            digitalWrite(ALARM_LED_PIN, LOW);
            alarmOutputState = false;
        }

        return;
    }

    unsigned long now = millis();

    if (now - lastAlarmToggleTime >= ALARM_BLINK_MS)
    {
        lastAlarmToggleTime = now;

        alarmOutputState = !alarmOutputState;

        digitalWrite(BUZZER_PIN, alarmOutputState ? HIGH : LOW);
        digitalWrite(ALARM_LED_PIN, alarmOutputState ? HIGH : LOW);
    }
}


// ====================================================================
// HAVERSINE DISTANCE (km)
// ====================================================================

double haversineKm(double lat1, double lon1, double lat2, double lon2)
{
    double dLat = radians(lat2 - lat1);
    double dLon = radians(lon2 - lon1);

    double a =
        sin(dLat / 2.0) * sin(dLat / 2.0)
        +
        cos(radians(lat1)) * cos(radians(lat2))
        * sin(dLon / 2.0) * sin(dLon / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return 6371.0 * c; // Earth radius in km
}


// ====================================================================
// LIVE TRIP DISTANCE ACCUMULATION (WITH GPS JUMP PROTECTION)
// ====================================================================

void updateTripDistance()
{
    if (!tripActive)
    {
        return;
    }

    unsigned long now = millis();

    // First valid point of the trip - just anchor it, no distance yet.
    if (!lastTripPointValid)
    {
        lastTripLat = gpsLatitude;
        lastTripLon = gpsLongitude;
        lastGpsPointTime = now;
        lastTripPointValid = true;
        return;
    }

    double segmentKm = haversineKm(lastTripLat, lastTripLon, gpsLatitude, gpsLongitude);

    unsigned long dtMs = now - lastGpsPointTime;

    bool pointAccepted = true;

    if (dtMs > 0)
    {
        double hours = dtMs / 3600000.0;
        double impliedSpeedKmh = segmentKm / hours;

        if (impliedSpeedKmh > MAX_VALID_SPEED_KMH)
        {
            pointAccepted = false;

            Serial.print("GPS jump ignored - implied speed ");
            Serial.print(impliedSpeedKmh, 1);
            Serial.println(" km/h");
        }
    }

    if (pointAccepted)
    {
        tripDistanceKm += segmentKm;

        lastTripLat = gpsLatitude;
        lastTripLon = gpsLongitude;
        lastGpsPointTime = now;
    }
    // If rejected, keep the previous anchor point so a single bad fix
    // doesn't shift the whole accumulation baseline.
}


// ====================================================================
// TRIP START
// ====================================================================

void startTrip()
{
    tripStartFuel    = currentFuelLevel;
    tripEndFuel      = 0.0f;
    tripFuelConsumed = 0.0f;
    tripDistanceKm   = 0.0f;
    tripMileage      = -1.0f;
    tripStartTime    = millis();
    tripActive       = true;

    tripEndGPSValid = false;

    if (gpsFixAvailable)
    {
        startLatitude  = gpsLatitude;
        startLongitude = gpsLongitude;
        tripStartGPSValid = true;

        lastTripLat = gpsLatitude;
        lastTripLon = gpsLongitude;
        lastTripPointValid = true;
    }
    else
    {
        tripStartGPSValid = false;
        lastTripPointValid = false;
    }

    lastGpsPointTime = millis();

    printTripStartedBanner();
}


// ====================================================================
// TRIP STOP
// ====================================================================

void stopTrip()
{
    tripEndFuel = currentFuelLevel;
    tripEndTime = millis();

    if (gpsFixAvailable)
    {
        endLatitude  = gpsLatitude;
        endLongitude = gpsLongitude;
        tripEndGPSValid = true;
    }
    else
    {
        tripEndGPSValid = false;
    }

    if (tripActive)
    {
        float rawConsumed = tripStartFuel - tripEndFuel;

        // Fuel increased during the trip (e.g. topped up mid-trip) -
        // never report negative consumption.
        tripFuelConsumed = (rawConsumed > 0.0f) ? rawConsumed : 0.0f;

        if (tripDistanceKm > 0.0f && tripFuelConsumed > 0.0f)
        {
            tripMileage = tripDistanceKm / tripFuelConsumed;
        }
        else
        {
            tripMileage = -1.0f; // N/A - shown as CALCULATING / N/A on dashboard
        }
    }

    tripActive = false;
    lastTripPointValid = false;

    printTripStoppedBanner();
}


// ====================================================================
// VEHICLE STATE TRANSITION
// ====================================================================

void processVehicleState()
{
    // ---------------------------------------------------------------
    // OFF -> ON
    // ---------------------------------------------------------------

    if (
        vehicleOn &&
        !previousVehicleOn
    )
    {
        Serial.println();

        Serial.println(
            "========================================"
        );

        Serial.println(
            "           VEHICLE STARTED"
        );

        Serial.println(
            "========================================"
        );


        // Give fuel sensor time to settle

        delay(
            STARTUP_SETTLE_MS
        );


        currentFuelLevel =
            readFuelLevel();


        storedFuelReference =
            currentFuelLevel;


        Serial.print(
            "Starting Fuel : "
        );

        Serial.print(
            currentFuelLevel,
            1
        );

        Serial.println(
            " L"
        );


        // Reset event flags

        suspiciousActivity =
            false;

        theftMonitoring =
            false;

        refuelingMonitoring =
            false;

        fuelDropMonitoring =
            false;

        // Starting the vehicle acknowledges/clears any previous
        // theft alarm or refuel-event badge - buzzer + LED go silent.
        theftAlarmActive =
            false;

        refuelEventActive =
            false;


        currentState =
            STATE_VEHICLE_ON;


        Serial.println(
            "MODE: CONTINUOUS FUEL MONITORING"
        );

        Serial.println(
            "Vibration detection ignored while vehicle is ON."
        );


        Serial.println(
            "========================================"
        );

        // NEW: capture trip-start fuel + GPS + time snapshot
        startTrip();
    }


    // ---------------------------------------------------------------
    // ON -> OFF
    // ---------------------------------------------------------------

    if (
        !vehicleOn &&
        previousVehicleOn
    )
    {
        Serial.println();

        Serial.println(
            "========================================"
        );

        Serial.println(
            "           VEHICLE STOPPED"
        );

        Serial.println(
            "========================================"
        );


        delay(
            PARKING_SETTLE_MS
        );


        currentFuelLevel =
            readFuelLevel();


        storedFuelReference =
            currentFuelLevel;


        Serial.print(
            "Parked Fuel Level : "
        );

        Serial.print(
            currentFuelLevel,
            1
        );

        Serial.println(
            " L"
        );


        suspiciousActivity =
            false;

        theftMonitoring =
            false;

        refuelingMonitoring =
            false;


        currentState =
            STATE_VEHICLE_OFF;


        Serial.println(
            "MODE: PARKED MONITORING"
        );

        Serial.println(
            "Theft + Refueling detection ACTIVE"
        );


        Serial.println(
            "========================================"
        );

        // NEW: capture trip-end fuel + GPS + time snapshot, then
        // compute fuel consumed / distance / mileage for the trip.
        stopTrip();
    }
}


// ====================================================================
// VEHICLE ON MODE
// ====================================================================

void processVehicleON()
{
    currentState =
        STATE_VEHICLE_ON;


    // ---------------------------------------------------------------
    // IMPORTANT:
    //
    // When vehicle is ON:
    //
    // Vibration is ignored.
    // MPU movement is ignored for theft.
    //
    // Only fuel level is monitored continuously.
    // Trip distance is accumulated live from GPS in updateGPS().
    // ---------------------------------------------------------------


    currentFuelLevel =
        readFuelLevel();


    Serial.print(
        "[ON] Fuel Level : "
    );

    Serial.print(
        currentFuelLevel,
        1
    );

    Serial.println(
        " L"
    );


    Serial.print(
        "[ON] MPU Roll : "
    );

    Serial.print(
        roll,
        1
    );

    Serial.print(
        " deg | Pitch : "
    );

    Serial.print(
        pitch,
        1
    );

    Serial.println(
        " deg"
    );

    Serial.print(
        "[ON] Gyro Z : "
    );

    Serial.print(
        gyroZ,
        3
    );

    Serial.print(
        " rad/s | Tilt : "
    );

    Serial.println(
        mpuTilted ?
        "TILTED" :
        "NORMAL"
    );


    Serial.println(
        "[ON] Vibration ignored."
    );

    // Live trip snapshot in the Serial log
    if (tripActive)
    {
        float liveConsumed = tripStartFuel - currentFuelLevel;
        if (liveConsumed < 0) liveConsumed = 0;

        Serial.print("[TRIP] Distance So Far : ");
        Serial.print(tripDistanceKm, 2);
        Serial.println(" km");

        Serial.print("[TRIP] Fuel Used So Far: ");
        Serial.print(liveConsumed, 2);
        Serial.println(" L");
    }
}


// ====================================================================
// VEHICLE OFF MODE
// ====================================================================

void processVehicleOFF()
{
    currentState =
        STATE_VEHICLE_OFF;


    // ---------------------------------------------------------------
    // READ CURRENT FUEL ONCE FOR THIS CYCLE
    // ---------------------------------------------------------------

    float fuelNow =
        readFuelLevel();

    float fuelDifference =
        fuelNow -
        storedFuelReference;


    // ---------------------------------------------------------------
    // CASE 1
    // FUEL LEVEL DECREASING WHILE PARKED -> THEFT
    //
    // This runs independently of vibration: any significant fuel
    // drop while the vehicle is OFF is treated as theft. A short
    // stabilization window (FUEL_STABLE_TIME_MS) is used so a single
    // noisy ultrasonic sample can't trigger a false alarm.
    // ---------------------------------------------------------------

    if (
        fuelDifference <=
        -FUEL_CHANGE_THRESHOLD_L
    )
    {
        if (!fuelDropMonitoring)
        {
            fuelDropMonitoring =
                true;

            fuelDropStartTime =
                millis();

            fuelDropInitialLevel =
                storedFuelReference;

            Serial.println();
            Serial.println("----------------------------------------");
            Serial.println("     FUEL DROP DETECTED WHILE PARKED");
            Serial.println("----------------------------------------");
            Serial.print("Reference Fuel: ");
            Serial.print(fuelDropInitialLevel, 1);
            Serial.println(" L");
            Serial.println("Confirming theft...");
        }


        if (
            millis() - fuelDropStartTime >=
            FUEL_STABLE_TIME_MS
        )
        {
            float finalReading =
                readFuelLevel();

            float finalDrop =
                fuelDropInitialLevel -
                finalReading;


            if (
                finalDrop >=
                FUEL_CHANGE_THRESHOLD_L
            )
            {
                theftInitialLevel =
                    fuelDropInitialLevel;

                theftFinalLevel =
                    finalReading;

                fuelLost =
                    finalDrop;


                printTheftAlert();


                storedFuelReference =
                    theftFinalLevel;

                currentFuelLevel =
                    theftFinalLevel;


                fuelDropMonitoring =
                    false;

                suspiciousActivity =
                    false;

                theftMonitoring =
                    false;

                // Persists (drives buzzer + LED + dashboard alert)
                // until the vehicle is next started.
                theftAlarmActive =
                    true;


                currentState =
                    STATE_THEFT;
            }
            else
            {
                // Reading settled back - treat as sensor noise, not theft.

                fuelDropMonitoring =
                    false;
            }
        }

        // While a fuel-drop is being confirmed, don't also evaluate
        // vibration/refueling this cycle.
        return;
    }
    else
    {
        fuelDropMonitoring =
            false;
    }


    // ---------------------------------------------------------------
    // CASE 2
    // VIBRATION ONLY, NO FUEL LOSS -> SUSPICIOUS (reported only,
    // does not by itself confirm theft)
    // ---------------------------------------------------------------

    if (vibrationDetected)
    {
        if (!suspiciousActivity)
        {
            suspiciousActivity =
                true;

            currentState =
                STATE_SUSPICIOUS;

            Serial.println();
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            Serial.println("        SUSPICIOUS ACTIVITY");
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            Serial.println("Vehicle State : OFF");
            Serial.println("Vibration     : DETECTED");
            Serial.println("Fuel Level    : STABLE (no theft yet)");
            Serial.println();
        }

        currentState =
            STATE_SUSPICIOUS;

        // Vibration alone does not consume the reading used for
        // refueling below - just report and continue monitoring.
        return;
    }
    else
    {
        if (suspiciousActivity)
        {
            Serial.println();
            Serial.println("----------------------------------------");
            Serial.println("     VIBRATION CLEARED - NO THEFT");
            Serial.println("     RESUMING NORMAL MONITORING");
            Serial.println("----------------------------------------");
        }

        suspiciousActivity =
            false;
    }


    // ---------------------------------------------------------------
    // CASE 3
    // REFUELING DETECTION
    // ---------------------------------------------------------------

    processRefuelingDetection();
}


// ====================================================================
// REFUELING DETECTION
// ====================================================================

void processRefuelingDetection()
{
    // ---------------------------------------------------------------
    // READ FUEL
    // ---------------------------------------------------------------

    float fuelNow =
        readFuelLevel();


    float fuelDifference =
        fuelNow -
        storedFuelReference;


    // ---------------------------------------------------------------
    // FUEL INCREASE DETECTED
    // ---------------------------------------------------------------

    if (
        fuelDifference >=
        FUEL_CHANGE_THRESHOLD_L
    )
    {
        // -----------------------------------------------------------
        // START REFUELING MONITOR
        // -----------------------------------------------------------

        if (!refuelingMonitoring)
        {
            refuelingMonitoring =
                true;


            refuelingStartTime =
                millis();


            refuelInitialLevel =
                storedFuelReference;


            currentState =
                STATE_REFUELING;


            Serial.println();

            Serial.println(
                "----------------------------------------"
            );

            Serial.println(
                "        REFUELING DETECTED"
            );

            Serial.println(
                "----------------------------------------"
            );


            Serial.print(
                "INITIAL FUEL : "
            );

            Serial.print(
                refuelInitialLevel,
                1
            );

            Serial.println(
                " L"
            );


            Serial.println(
                "Waiting for fuel level to stabilize..."
            );
        }


        // -----------------------------------------------------------
        // WAIT FOR STABILIZATION
        // -----------------------------------------------------------

        if (
            millis() -
            refuelingStartTime >=
            FUEL_STABLE_TIME_MS
        )
        {
            float finalReading =
                readFuelLevel();


            float finalDifference =
                finalReading -
                refuelInitialLevel;


            // -------------------------------------------------------
            // CONFIRM REFUELING
            // -------------------------------------------------------

            if (
                finalDifference >=
                FUEL_CHANGE_THRESHOLD_L
            )
            {
                refuelFinalLevel =
                    finalReading;


                fuelAdded =
                    refuelFinalLevel -
                    refuelInitialLevel;


                printRefuelingAlert();


                storedFuelReference =
                    refuelFinalLevel;


                currentFuelLevel =
                    refuelFinalLevel;


                refuelingMonitoring =
                    false;

                // Persists (drives the dashboard Event Status block)
                // until the vehicle is next started.
                refuelEventActive =
                    true;


                currentState =
                    STATE_REFUELING;
            }
            else
            {
                // False / unstable change

                refuelingMonitoring =
                    false;


                currentState =
                    STATE_VEHICLE_OFF;
            }
        }


        return;
    }


    // ---------------------------------------------------------------
    // NORMAL PARKED MONITORING
    // ---------------------------------------------------------------

    currentFuelLevel =
        fuelNow;
}


// ====================================================================
// THEFT ALERT
// ====================================================================

void printTheftAlert()
{
    Serial.println();

    Serial.println(
        "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    );

    Serial.println(
        "           FUEL THEFT ALERT"
    );

    Serial.println(
        "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    );


    Serial.println(
        "Vehicle State : OFF"
    );


    Serial.print(
        "Vibration     : "
    );

    Serial.println(
        vibrationDetected ?
        "DETECTED" :
        "NOT DETECTED (fuel drop alone triggered this)"
    );


    Serial.print(
        "Initial Fuel  : "
    );

    Serial.print(
        theftInitialLevel,
        1
    );

    Serial.println(
        " L"
    );


    Serial.print(
        "Final Fuel    : "
    );

    Serial.print(
        theftFinalLevel,
        1
    );

    Serial.println(
        " L"
    );


    Serial.print(
        "Fuel Lost     : "
    );

    Serial.print(
        fuelLost,
        1
    );

    Serial.println(
        " L"
    );


    Serial.println();

    Serial.println(
        "THEFT CONFIRMED"
    );


    // GPS

    printGPSLocation();


    Serial.println(
        "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    );

    Serial.println();
}


// ====================================================================
// REFUELING ALERT
// ====================================================================

void printRefuelingAlert()
{
    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        "          REFUELING CONFIRMED"
    );

    Serial.println(
        "========================================"
    );


    Serial.println(
        "Vehicle State : OFF"
    );


    Serial.print(
        "Initial Fuel  : "
    );

    Serial.print(
        refuelInitialLevel,
        1
    );

    Serial.println(
        " L"
    );


    Serial.print(
        "Final Fuel    : "
    );

    Serial.print(
        refuelFinalLevel,
        1
    );

    Serial.println(
        " L"
    );


    Serial.print(
        "Fuel Added    : "
    );

    Serial.print(
        fuelAdded,
        1
    );

    Serial.println(
        " L"
    );


    Serial.println();


    Serial.println(
        "REFUELING EVENT CONFIRMED"
    );


    // GPS

    printGPSLocation();


    Serial.println(
        "========================================"
    );

    Serial.println();
}


// ====================================================================
// TRIP STARTED BANNER
// ====================================================================

void printTripStartedBanner()
{
    Serial.println();
    Serial.println("========================================");
    Serial.println("🚛 VEHICLE STARTED");
    Serial.println("========================================");

    Serial.print("Initial Fuel : ");
    Serial.print(tripStartFuel, 2);
    Serial.println(" L");

    Serial.print("GPS          : ");
    if (tripStartGPSValid)
    {
        Serial.print(startLatitude, 6);
        Serial.print(", ");
        Serial.println(startLongitude, 6);
    }
    else
    {
        Serial.println("NO FIX");
    }

    Serial.println("Trip Started : YES");
    Serial.println("========================================");
}


// ====================================================================
// TRIP STOPPED BANNER
// ====================================================================

void printTripStoppedBanner()
{
    Serial.println();
    Serial.println("========================================");
    Serial.println("🛑 VEHICLE STOPPED");
    Serial.println("========================================");

    Serial.print("Initial Fuel  : ");
    Serial.print(tripStartFuel, 2);
    Serial.println(" L");

    Serial.print("Final Fuel    : ");
    Serial.print(tripEndFuel, 2);
    Serial.println(" L");

    Serial.print("Fuel Consumed : ");
    Serial.print(tripFuelConsumed, 2);
    Serial.println(" L");

    Serial.println();

    Serial.print("Distance      : ");
    Serial.print(tripDistanceKm, 2);
    Serial.println(" km");

    Serial.print("Mileage       : ");
    if (tripMileage > 0)
    {
        Serial.print(tripMileage, 2);
        Serial.println(" km/L");
    }
    else
    {
        Serial.println("CALCULATING");
    }

    Serial.println();

    Serial.print("Start GPS     : ");
    if (tripStartGPSValid)
    {
        Serial.print(startLatitude, 6);
        Serial.print(", ");
        Serial.println(startLongitude, 6);
    }
    else
    {
        Serial.println("NO FIX");
    }

    Serial.print("End GPS       : ");
    if (tripEndGPSValid)
    {
        Serial.print(endLatitude, 6);
        Serial.print(", ");
        Serial.println(endLongitude, 6);
    }
    else
    {
        Serial.println("NO FIX");
    }

    Serial.println("========================================");
}


// ====================================================================
// GPS LOCATION
// ====================================================================

void printGPSLocation()
{
    Serial.println();

    Serial.println(
        "GPS LOCATION"
    );

    Serial.println(
        "----------------------------------------"
    );


    if (gpsFixAvailable)
    {
        Serial.print(
            "Latitude  : "
        );

        Serial.println(
            gpsLatitude,
            6
        );


        Serial.print(
            "Longitude : "
        );

        Serial.println(
            gpsLongitude,
            6
        );


        Serial.print(
            "Google Maps: "
        );

        Serial.print(
            "https://maps.google.com/?q="
        );

        Serial.print(
            gpsLatitude,
            6
        );

        Serial.print(",");

        Serial.println(
            gpsLongitude,
            6
        );
    }
    else
    {
        Serial.println(
            "GPS FIX NOT AVAILABLE"
        );

        Serial.println(
            "Waiting for satellite fix..."
        );
    }


    Serial.println(
        "----------------------------------------"
    );
}


// ====================================================================
// DASHBOARD
// ====================================================================

void printDashboard()
{
    Serial.println();

    Serial.println(
        "================================================"
    );

    Serial.println(
        "                 FUELGUARD"
    );

    Serial.println(
        "             SYSTEM DASHBOARD"
    );

    Serial.println(
        "================================================"
    );


    // ---------------------------------------------------------------
    // VEHICLE VOLTAGE
    // ---------------------------------------------------------------

    Serial.println(
        "[VEHICLE POWER]"
    );


    Serial.print(
        "ADC Value       : "
    );

    Serial.println(
        voltageADC
    );


    Serial.print(
        "Sensor Voltage  : "
    );

    Serial.print(
        sensorVoltage,
        2
    );

    Serial.println(
        " V"
    );


    Serial.print(
        "Vehicle State   : "
    );

    if (vehicleOn)
    {
        Serial.println(
            "ON"
        );
    }
    else
    {
        Serial.println(
            "OFF"
        );
    }


    Serial.println();


    // ---------------------------------------------------------------
    // FUEL
    // ---------------------------------------------------------------

    Serial.println(
        "[FUEL MONITORING]"
    );


    Serial.print(
        "Distance        : "
    );

    if (currentDistance >= 0)
    {
        Serial.print(
            currentDistance,
            2
        );

        Serial.println(
            " cm"
        );
    }
    else
    {
        Serial.println(
            "NO ECHO"
        );
    }


    Serial.print(
        "Fuel Level      : "
    );

    Serial.print(
        currentFuelLevel,
        1
    );

    Serial.println(
        " L"
    );


    Serial.print(
        "Tank Capacity   : "
    );

    Serial.print(
        TANK_CAPACITY_L,
        1
    );

    Serial.println(
        " L"
    );


    Serial.println();


    // ---------------------------------------------------------------
    // VIBRATION
    // ---------------------------------------------------------------

    Serial.println(
        "[SECURITY]"
    );


    Serial.print(
        "Vibration       : "
    );

    if (vibrationDetected)
    {
        Serial.println(
            "DETECTED"
        );
    }
    else
    {
        Serial.println(
            "NORMAL"
        );
    }

    Serial.print(
        "Buzzer/LED Alarm: "
    );

    Serial.println(
        theftAlarmActive ?
        "ACTIVE (blinking/beeping)" :
        "OFF"
    );


    Serial.println();


    // ---------------------------------------------------------------
    // MPU
    // ---------------------------------------------------------------

    Serial.println(
        "[MPU6050]"
    );


    Serial.print(
        "Roll            : "
    );

    Serial.print(
        roll,
        1
    );

    Serial.println(
        " deg"
    );


    Serial.print(
        "Pitch           : "
    );

    Serial.print(
        pitch,
        1
    );

    Serial.println(
        " deg"
    );


    Serial.print(
        "Acceleration    : "
    );

    Serial.print(
        accelerationMagnitude,
        2
    );

    Serial.println(
        " m/s2"
    );


    Serial.print(
        "Movement        : "
    );

    Serial.println(
        mpuMovementDetected ?
        "DETECTED" :
        "NORMAL"
    );

    Serial.print(
        "Gyro Z          : "
    );

    Serial.print(
        gyroZ,
        3
    );

    Serial.println(
        " rad/s"
    );

    Serial.print(
        "Tilt Status     : "
    );

    Serial.println(
        mpuTilted ?
        "TILTED" :
        "NORMAL"
    );

    Serial.print(
        "Incline Status  : "
    );

    Serial.println(
        vehicleInclined ?
        "INCLINED (on slope)" :
        "LEVEL"
    );


    Serial.println();


    // ---------------------------------------------------------------
    // GPS
    // ---------------------------------------------------------------

    Serial.println(
        "[GPS]"
    );


    if (gpsFixAvailable)
    {
        Serial.println(
            "GPS Status      : FIXED"
        );


        Serial.print(
            "Latitude        : "
        );

        Serial.println(
            gpsLatitude,
            6
        );


        Serial.print(
            "Longitude       : "
        );

        Serial.println(
            gpsLongitude,
            6
        );
    }
    else
    {
        Serial.println(
            "GPS Status      : NO FIX"
        );
    }


    Serial.println();


    // ---------------------------------------------------------------
    // TRIP INFORMATION (NEW)
    // ---------------------------------------------------------------

    Serial.println(
        "[TRIP INFORMATION]"
    );

    Serial.println(
        "------------------------------"
    );

    Serial.print(
        "Vehicle State : "
    );
    Serial.println(vehicleOn ? "ON" : "OFF");

    Serial.print(
        "Initial Fuel  : "
    );
    Serial.print(tripStartFuel, 2);
    Serial.println(" L");

    Serial.print(
        "Current Fuel  : "
    );
    Serial.print(currentFuelLevel, 2);
    Serial.println(" L");

    {
        float shownConsumed;
        if (tripActive)
        {
            shownConsumed = tripStartFuel - currentFuelLevel;
            if (shownConsumed < 0) shownConsumed = 0;
        }
        else
        {
            shownConsumed = tripFuelConsumed;
        }

        Serial.print(
            "Fuel Consumed : "
        );
        Serial.print(shownConsumed, 2);
        Serial.println(" L");
    }

    Serial.print(
        "Trip Distance : "
    );
    Serial.print(tripDistanceKm, 2);
    Serial.println(" km");

    Serial.print(
        "Mileage       : "
    );
    if (tripMileage > 0 && !tripActive)
    {
        Serial.print(tripMileage, 2);
        Serial.println(" km/L");
    }
    else
    {
        Serial.println("CALCULATING");
    }

    Serial.print(
        "Start GPS     : "
    );
    if (tripStartGPSValid)
    {
        Serial.print(startLatitude, 6);
        Serial.print(",");
        Serial.println(startLongitude, 6);
    }
    else
    {
        Serial.println("NO FIX");
    }

    Serial.print(
        "Current GPS   : "
    );
    if (gpsFixAvailable)
    {
        Serial.print(gpsLatitude, 6);
        Serial.print(",");
        Serial.println(gpsLongitude, 6);
    }
    else
    {
        Serial.println("NO FIX");
    }

    Serial.println(
        "------------------------------"
    );

    Serial.println();


    // ---------------------------------------------------------------
    // SYSTEM MODE
    // ---------------------------------------------------------------

    Serial.println(
        "[SYSTEM MODE]"
    );


    switch (currentState)
    {
        case STATE_VEHICLE_OFF:

            Serial.println(
                "VEHICLE OFF"
            );

            Serial.println(
                "Parking monitoring ACTIVE"
            );

            break;


        case STATE_VEHICLE_ON:

            Serial.println(
                "VEHICLE ON"
            );

            Serial.println(
                "Continuous fuel monitoring ACTIVE"
            );

            Serial.println(
                "Vibration security check IGNORED"
            );

            break;


        case STATE_SUSPICIOUS:

            Serial.println(
                "SUSPICIOUS ACTIVITY"
            );

            Serial.println(
                "Checking fuel for theft"
            );

            break;


        case STATE_REFUELING:

            Serial.println(
                "REFUELING MONITORING"
            );

            break;


        case STATE_THEFT:

            Serial.println(
                "THEFT CONFIRMED"
            );

            break;
    }


    Serial.println(
        "================================================"
    );

    Serial.println();
}
