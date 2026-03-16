#include <ETH.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <FastLED.h>  // include FastLED *before* Artnet
#include <ArtnetETH.h>

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#define ETH_PHY_POWER 16

WebServer server(80);

ArtnetETHReceiver artnet;

// Configuration
struct Config {
  uint8_t universe = 1;
  IPAddress ip = IPAddress(192, 168, 1, 201);
  IPAddress gateway = IPAddress(192, 168, 1, 1);
  IPAddress subnet = IPAddress(255, 255, 255, 0);
};
Config config;

// FastLED
#define NUM_LEDS 64
CRGB leds[NUM_LEDS];
const uint8_t PIN_LED_DATA = 12;

// Servos
#define NUM_SERVOS 16
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 150   // Pulse minimum (0°)
#define SERVOMAX 500   // Pulse maximum (180°) - Ajusté selon observation butée à DMX 80
#define SERVO_FREQ 50  // Fréquence 50Hz pour servos
unsigned long lastServoUpdate = 0;
uint8_t servoValues[NUM_SERVOS];


bool ledsChanged = false;
bool servoChanged = false;
bool testActive = false;
int testPos = 0;
unsigned long lastTestStep = 0;
const unsigned long SERVO_UPDATE_INTERVAL = 20; // 50Hz max pour servos

bool i2cHealthy = true;
unsigned long lastI2CCheck = 0;
const unsigned long I2C_CHECK_INTERVAL = 5000; // Vérification toutes les 5s


void setup() {
  // Initialiser I2C avec les pins spécifiés
  Wire.begin(14, 15); // (SDA, SCL)
  
  // Initialiser le contrôleur de servo PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000); // Calibration oscillateur interne
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  
  // Initialiser les servos à la position centrale
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoValues[i] = 127; // Position centrale (0-255 → 127 = milieu)
    setServoPosition(i, 127);
  }
  // Initialiser SPIFFS
  SPIFFS.begin(true);
  
  // Charger la configuration
  loadConfig();

  WiFi.onEvent(WiFiEvent);

  ETH.begin(1, 16, 23, 18, ETH_PHY_LAN8720, ETH_CLK_MODE);

  ETH.config(config.ip, config.gateway, config.subnet);

  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/test-servos", HTTP_POST, handleServoTest);

  server.begin();

  // Initialiser FastLED
  FastLED.addLeds<WS2812, PIN_LED_DATA, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  
  // Initialiser Art-Net avec callback personnalisé pour LEDs + Servos
  artnet.begin();
  artnet.subscribeArtDmxUniverse(config.universe, [&](const uint8_t* data, uint16_t size, const ArtDmxMetadata &metadata, const ArtNetRemoteInfo &remote) {
    // Traitement des LEDs (canaux 1-192)
    for (size_t pixel = 0; pixel < NUM_LEDS && (pixel * 3 + 2) < size; ++pixel) {
      size_t idx = pixel * 3;
      leds[pixel].r = data[idx + 0];
      leds[pixel].g = data[idx + 1];
      leds[pixel].b = data[idx + 2];
    }
    ledsChanged = true;
    
    // Traitement des servos (canaux 193-208)
    for (int servo = 0; servo < NUM_SERVOS; ++servo) {
      size_t servoChannelIndex = (NUM_LEDS * 3) + servo; // Canal 193+ pour servos
      if (servoChannelIndex < size) {
        uint8_t newValue = data[servoChannelIndex];
        if (servoValues[servo] != newValue) {
          servoValues[servo] = newValue;
          servoChanged = true; // Marquer pour mise à jour différée
        }
      }
    }
  });
}

void loop() {
  server.handleClient();
  artnet.parse();  // check if artnet packet has come and execute callback
  if (ledsChanged) {
    FastLED.show();
    ledsChanged = false;
  }
  
  // Mise à jour optimisée des servos (limitée à 50Hz)
  unsigned long currentTime = millis();
  if (!testActive && servoChanged && (currentTime - lastServoUpdate >= SERVO_UPDATE_INTERVAL)) {
    updateAllServos();
    lastServoUpdate = currentTime;
    servoChanged = false;
  }
  
  // Vérification périodique de la santé I2C
  if (currentTime - lastI2CCheck >= I2C_CHECK_INTERVAL) {
    checkI2CHealth();
    lastI2CCheck = currentTime;
  }

  // Test servo non-bloquant
  if (testActive && (currentTime - lastTestStep >= SERVO_UPDATE_INTERVAL)) {
    lastTestStep = currentTime;
    if (testPos <= 255) {
      for (int i = 0; i < NUM_SERVOS; i++) setServoPosition(i, testPos);
      testPos += 5;
    } else {
      for (int i = 0; i < NUM_SERVOS; i++) setServoPosition(i, 127);
      testActive = false;
    }
  }
}

void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, F("text/html; charset=utf-8"), "");

  server.sendContent(F("<!DOCTYPE html><html lang='fr'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>WT32-ETH01 Art-Net Controller</title><style>"
    "*{margin:0;padding:0;box-sizing:border-box}"
    "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',system-ui,sans-serif;background:linear-gradient(135deg,#1a1a2e 0%,#16213e 100%);color:#fff;min-height:100vh;padding:20px}"
    ".container{max-width:800px;margin:0 auto}"
    ".header{text-align:center;margin-bottom:40px}"
    ".header h1{font-size:2.5rem;font-weight:300;margin-bottom:10px;background:linear-gradient(45deg,#e94560,#f39c12);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text}"
    ".status-bar{display:flex;justify-content:center;gap:30px;margin:20px 0;flex-wrap:wrap}"
    ".status-item{background:rgba(255,255,255,0.1);backdrop-filter:blur(10px);padding:15px 25px;border-radius:12px;border:1px solid rgba(255,255,255,0.2);min-width:140px;text-align:center}"
    ".status-label{font-size:0.85rem;opacity:0.8;margin-bottom:5px;text-transform:uppercase;letter-spacing:1px}"
    ".status-value{font-size:1.2rem;font-weight:600;color:#e94560}"
    ".card{background:rgba(255,255,255,0.05);backdrop-filter:blur(20px);border:1px solid rgba(255,255,255,0.1);border-radius:20px;padding:30px;margin:20px 0;box-shadow:0 8px 32px rgba(0,0,0,0.3)}"
    ".card h2{font-size:1.5rem;margin-bottom:25px;color:#fff;display:flex;align-items:center;gap:10px}"
    ".form-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:20px}"
    ".form-group{margin-bottom:20px}"
    ".form-group label{display:block;margin-bottom:8px;font-weight:500;color:#b8c5d6;font-size:0.9rem;text-transform:uppercase;letter-spacing:0.5px}"
    ".form-group input{width:100%;padding:12px 16px;border:2px solid rgba(255,255,255,0.1);border-radius:8px;background:rgba(255,255,255,0.05);color:#fff;font-size:1rem;transition:all 0.3s ease}"
    ".form-group input:focus{outline:none;border-color:#e94560;background:rgba(255,255,255,0.1);box-shadow:0 0 0 3px rgba(233,69,96,0.2)}"
    ".form-group input:valid{border-color:#27ae60}"
    ".btn{background:linear-gradient(45deg,#e94560,#f39c12);border:none;padding:15px 30px;border-radius:12px;color:#fff;font-size:1rem;font-weight:600;cursor:pointer;transition:all 0.3s ease;text-transform:uppercase;letter-spacing:1px;width:100%;margin-top:20px}"
    ".btn:hover{transform:translateY(-2px);box-shadow:0 10px 25px rgba(233,69,96,0.4)}"
    ".btn:active{transform:translateY(0)}"
    ".pulse{animation:pulse 2s infinite}"
    "@keyframes pulse{0%{opacity:1}50%{opacity:0.7}100%{opacity:1}}"
    "@media(max-width:600px){.status-bar{flex-direction:column;align-items:center}.form-grid{grid-template-columns:1fr}.header h1{font-size:2rem}}"
    ".ip-validator{font-size:0.8rem;margin-top:5px;opacity:0.8}"
    ".valid{color:#27ae60}.invalid{color:#e74c3c}"
    "</style></head><body><div class='container'><header class='header'>"
    "<h1>&#9632; ART-NET CONTROLLER</h1><div class='status-bar'>"
    "<div class='status-item'><div class='status-label'>Adresse IP</div><div class='status-value'>"));
  server.sendContent(config.ip.toString());
  server.sendContent(F("</div></div><div class='status-item'><div class='status-label'>Univers</div><div class='status-value'>"));
  server.sendContent(String(config.universe));
  server.sendContent(F("</div></div><div class='status-item pulse'><div class='status-label'>LEDs</div><div class='status-value'>64</div></div>"
    "<div class='status-item'><div class='status-label'>Servos</div><div class='status-value'>16</div></div>"
    "</div></header>"
    "<div class='card'><h2><span style='font-size:1.2em'>&#128225;</span> Mapping Art-Net</h2><div class='form-grid'>"
    "<div class='form-group'><label>Canaux LEDs</label>"
    "<div style='background:rgba(52,152,219,0.1);border:1px solid rgba(52,152,219,0.3);border-radius:8px;padding:12px;color:#3498db;font-weight:600'>Canaux 1-192 (64 LEDs &times; RGB)</div></div>"
    "<div class='form-group'><label>Canaux Servos</label>"
    "<div style='background:rgba(46,204,113,0.1);border:1px solid rgba(46,204,113,0.3);border-radius:8px;padding:12px;color:#2ecc71;font-weight:600'>Canaux 193-208 (16 Servos)</div></div>"
    "</div><div style='background:rgba(241,196,15,0.1);border:1px solid rgba(241,196,15,0.3);border-radius:8px;padding:15px;margin-top:15px'>"
    "<div style='color:#f1c40f;font-weight:600;margin-bottom:8px'>&#128161; Informations Techniques</div>"
    "<div style='color:#ecf0f1;font-size:0.9rem;line-height:1.4'>"
    "&bull; Servos: Valeurs 0-255 &rarr; Angles 0&deg;-180&deg;<br>"
    "&bull; Fr&eacute;quence PWM: 50Hz (standard servo)<br>"
    "&bull; I2C: SDA=Pin14, SCL=Pin15<br>"
    "&bull; Contr&ocirc;leur: PCA9685</div></div></div>"
    "<div class='card'><h2><span style='font-size:1.2em'>&#128295;</span> Test Servos</h2>"
    "<div style='text-align:center'>"
    "<p style='color:#b8c5d6;margin-bottom:20px'>Testez le bon fonctionnement des 16 servo-moteurs</p>"
    "<form method='POST' action='/test-servos' style='display:inline'>"
    "<button type='submit' class='btn' style='background:linear-gradient(45deg,#27ae60,#2ecc71);width:auto;padding:12px 30px;display:inline-block'>&#9654; Lancer Test Servos</button>"
    "</form><div style='font-size:0.85rem;color:#7f8c8d;margin-top:15px'>Le test effectue un balayage complet de tous les servos<br>Dur&eacute;e: ~10 secondes</div>"
    "</div></div>"
    "<div class='card'><h2><span style='font-size:1.2em'>&#9881;</span> Configuration R&eacute;seau</h2>"
    "<form action='/save' method='POST' id='configForm'><div class='form-grid'>"
    "<div class='form-group'><label for='universe'>Univers Art-Net</label>"
    "<input type='number' id='universe' name='universe' value='"));
  server.sendContent(String(config.universe));
  server.sendContent(F("' min='0' max='15' required>"
    "</div>"
    "<div class='form-group'><label for='ip'>Adresse IP</label>"
    "<input type='text' id='ip' name='ip' value='"));
  server.sendContent(config.ip.toString());
  server.sendContent(F("' pattern='^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$' required>"
    "<div class='ip-validator' id='ip-status'></div></div>"
    "<div class='form-group'><label for='gateway'>Passerelle</label>"
    "<input type='text' id='gateway' name='gateway' value='"));
  server.sendContent(config.gateway.toString());
  server.sendContent(F("' pattern='^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$' required></div>"
    "<div class='form-group'><label for='subnet'>Masque R&eacute;seau</label>"
    "<input type='text' id='subnet' name='subnet' value='"));
  server.sendContent(config.subnet.toString());
  server.sendContent(F("' pattern='^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$' required></div>"
    "</div><button type='submit' class='btn'>&#9632; Sauvegarder Configuration</button>"
    "</form></div></div>"
    "<script>"
    "function validateIP(ip){const regex=/^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;return regex.test(ip)}"
    "document.getElementById('ip').addEventListener('input',function(){const status=document.getElementById('ip-status');if(validateIP(this.value)){status.textContent='\\u2713 Adresse IP valide';status.className='ip-validator valid'}else{status.textContent='\\u2717 Format IP invalide';status.className='ip-validator invalid'}});"
    "document.getElementById('configForm').addEventListener('submit',function(e){const inputs=['ip','gateway','subnet'];for(let i of inputs){if(!validateIP(document.getElementById(i).value)){e.preventDefault();alert('Veuillez corriger les adresses IP invalides');return}}});"
    "</script></body></html>"));
}

void handleConfig() {
  JsonDocument doc;
  doc["universe"] = config.universe;
  doc["ip"]       = config.ip.toString();
  doc["gateway"]  = config.gateway.toString();
  doc["subnet"]   = config.subnet.toString();
  String json;
  serializeJson(doc, json);
  server.send(200, F("application/json"), json);
}

void handleSave() {
  if (server.hasArg("universe") && server.hasArg("ip") && server.hasArg("gateway") && server.hasArg("subnet")) {
    config.universe = server.arg("universe").toInt();
    
    if (config.universe < 1) config.universe = 1;
    if (config.universe > 15) config.universe = 15;
    
    config.ip.fromString(server.arg("ip"));
    config.gateway.fromString(server.arg("gateway"));
    config.subnet.fromString(server.arg("subnet"));
    
    saveConfig();

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, F("text/html; charset=utf-8"), "");
    server.sendContent(F("<!DOCTYPE html><html lang='fr'><head><meta charset='UTF-8'><title>Configuration Sauvegard&eacute;e</title>"
      "<meta http-equiv='refresh' content='5;url=http://"));
    server.sendContent(config.ip.toString());
    server.sendContent(F("/'><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
      "<body style='font-family:-apple-system,BlinkMacSystemFont,system-ui,sans-serif;margin:0;background:linear-gradient(135deg,#1a1a2e 0%,#16213e 100%);color:#fff;min-height:100vh;display:flex;align-items:center;justify-content:center'>"
      "<div style='background:rgba(255,255,255,0.05);backdrop-filter:blur(20px);border:1px solid rgba(255,255,255,0.1);border-radius:20px;padding:40px;text-align:center;max-width:500px;box-shadow:0 8px 32px rgba(0,0,0,0.3)'>"
      "<div style='font-size:4rem;margin-bottom:20px;background:linear-gradient(45deg,#27ae60,#2ecc71);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text'>&#10003;</div>"
      "<h1 style='color:#27ae60;margin-bottom:20px;font-size:1.8rem;font-weight:300'>Configuration Sauvegard&eacute;e</h1>"
      "<p style='margin:20px 0;color:#b8c5d6;font-size:1.1rem'>Le syst&egrave;me red&eacute;marre avec la nouvelle configuration...</p>"
      "<div style='background:rgba(233,69,96,0.1);border:1px solid rgba(233,69,96,0.3);border-radius:12px;padding:15px;margin:20px 0'>"
      "<p style='color:#e94560;font-weight:600;font-size:1.2rem'>Nouvelle adresse: "));
    server.sendContent(config.ip.toString());
    server.sendContent(F("</p></div>"
      "<p style='color:#7f8c8d;font-size:0.9rem;margin-top:30px'>Redirection automatique dans 5 secondes...</p>"
      "<div style='width:100%;height:4px;background:rgba(255,255,255,0.1);border-radius:2px;margin-top:20px;overflow:hidden'>"
      "<div style='height:100%;background:linear-gradient(45deg,#e94560,#f39c12);border-radius:2px;animation:progress 5s linear'></div></div>"
      "<style>@keyframes progress{0%{width:0}100%{width:100%}}</style>"
      "</div></body></html>"));
    
    delay(1000);
    ESP.restart();
  } else {
    server.send(400, "text/plain", "Paramètres manquants");
  }
}

void handleServoTest() {
  server.send_P(200, "text/html; charset=utf-8",
    "<!DOCTYPE html><html lang='fr'><head><meta charset='UTF-8'><title>Test Servos</title>"
    "<meta http-equiv='refresh' content='12;url=/'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'></head>"
    "<body style='font-family:-apple-system,BlinkMacSystemFont,system-ui,sans-serif;margin:0;background:linear-gradient(135deg,#1a1a2e 0%,#16213e 100%);color:#fff;min-height:100vh;display:flex;align-items:center;justify-content:center'>"
    "<div style='background:rgba(255,255,255,0.05);backdrop-filter:blur(20px);border:1px solid rgba(255,255,255,0.1);border-radius:20px;padding:40px;text-align:center;max-width:500px;box-shadow:0 8px 32px rgba(0,0,0,0.3)'>"
    "<div style='font-size:4rem;margin-bottom:20px;background:linear-gradient(45deg,#27ae60,#2ecc71);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text'>&#9889;</div>"
    "<h1 style='color:#27ae60;margin-bottom:20px;font-size:1.8rem;font-weight:300'>Test Servos en Cours</h1>"
    "<p style='margin:20px 0;color:#b8c5d6;font-size:1.1rem'>Test de balayage des 16 servo-moteurs...</p>"
    "<div style='background:rgba(46,204,113,0.1);border:1px solid rgba(46,204,113,0.3);border-radius:12px;padding:15px;margin:20px 0'>"
    "<p style='color:#2ecc71;font-weight:600;font-size:1.2rem'>V&eacute;rifiez le mouvement des servos</p>"
    "</div>"
    "<p style='color:#7f8c8d;font-size:0.9rem;margin-top:30px'>Retour automatique dans 12 secondes...</p>"
    "<div style='width:100%;height:4px;background:rgba(255,255,255,0.1);border-radius:2px;margin-top:20px;overflow:hidden'>"
    "<div style='height:100%;background:linear-gradient(45deg,#27ae60,#2ecc71);border-radius:2px;animation:progress 12s linear'></div></div>"
    "<style>@keyframes progress{0%{width:0}100%{width:100%}}</style>"
    "</div></body></html>");

  testServos();
}

void loadConfig() {
  if (SPIFFS.exists("/config.json")) {
    File file = SPIFFS.open("/config.json", "r");
    if (file) {
      JsonDocument doc;
      deserializeJson(doc, file);
      config.universe = doc["universe"] | 1;
      
      String ip = doc["ip"] | "192.168.1.201";
      String gateway = doc["gateway"] | "192.168.1.1";
      String subnet = doc["subnet"] | "255.255.255.0";
      
      config.ip.fromString(ip);
      config.gateway.fromString(gateway);
      config.subnet.fromString(subnet);
      
      file.close();
    }
  }
}

void saveConfig() {
  File file = SPIFFS.open("/config.json", "w");
  if (file) {
    JsonDocument doc;
    doc["universe"] = config.universe;
    doc["ip"] = config.ip.toString();
    doc["gateway"] = config.gateway.toString();
    doc["subnet"] = config.subnet.toString();
    serializeJson(doc, file);
    file.close();
  }
}

// Fonction de vérification de la santé I2C
void checkI2CHealth() {
  Wire.beginTransmission(0x40); // Adresse par défaut du PCA9685
  uint8_t error = Wire.endTransmission();
  
  bool wasHealthy = i2cHealthy;
  i2cHealthy = (error == 0);
  
  if (wasHealthy && !i2cHealthy) {
    Wire.begin(14, 15); // (SDA, SCL)
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
  }
}

// Fonction de mise à jour optimisée de tous les servos
void updateAllServos() {
  if (!i2cHealthy) return;
  
  for (int servo = 0; servo < NUM_SERVOS; servo++) {
    setServoPosition(servo, servoValues[servo]);
  }
}

// Fonction de mapping Art-Net (0-255) vers position servo
void setServoPosition(uint8_t servoNum, uint8_t artnetValue) {
  if (servoNum >= NUM_SERVOS || !i2cHealthy) return;
  
  uint16_t pulseLength = SERVOMIN + ((uint32_t)(artnetValue) * (SERVOMAX - SERVOMIN)) / 255;
  pulseLength = constrain(pulseLength, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulseLength);
}

// Fonction de test servo (pour validation)
void testServos() {
  testPos = 0;
  lastTestStep = millis();
  testActive = true;
}

void WiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_ETH_START) {
    ETH.setHostname("wt32-eth01");
  }
}