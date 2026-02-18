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

ArtnetReceiver artnet;

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


bool servoChanged = false;
const unsigned long SERVO_UPDATE_INTERVAL = 20; // 50Hz max pour servos

bool i2cHealthy = true;
unsigned long lastI2CCheck = 0;
const unsigned long I2C_CHECK_INTERVAL = 5000; // Vérification toutes les 5s


void setup() {
  Serial.begin(115200);

  // Initialiser I2C avec les pins spécifiés
  Wire.begin(14, 15); // (SDA, SCL)
  Serial.println("I2C initialisé (SDA=14, SCL=15)");
  
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
  Serial.println("Contrôleur PCA9685 initialisé avec 16 servos");

  // Initialiser SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("Erreur SPIFFS");
  }
  
  // Charger la configuration
  loadConfig();

  WiFi.onEvent(WiFiEvent);

  ETH.begin(ETH_PHY_LAN8720, 1, 23, 18, 16, ETH_CLK_MODE);

  ETH.config(config.ip, config.gateway, config.subnet);

  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/test-servos", HTTP_POST, handleServoTest);

  server.begin();
  Serial.println("Serveur web démarré sur http://192.168.1.201");

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
  FastLED.show();
  
  // Mise à jour optimisée des servos (limitée à 50Hz)
  unsigned long currentTime = millis();
  if (servoChanged && (currentTime - lastServoUpdate >= SERVO_UPDATE_INTERVAL)) {
    updateAllServos();
    lastServoUpdate = currentTime;
    servoChanged = false;
  }
  
  // Vérification périodique de la santé I2C
  if (currentTime - lastI2CCheck >= I2C_CHECK_INTERVAL) {
    checkI2CHealth();
    lastI2CCheck = currentTime;
  }
}

void handleRoot() {
  String html = "<!DOCTYPE html>";
  html += "<html lang='fr'>";
  html += "<head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>WT32-ETH01 Art-Net Controller</title>";
  html += "<style>";
  
  // CSS moderne et optimisé pour ESP32
  html += "*{margin:0;padding:0;box-sizing:border-box}";
  html += "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',system-ui,sans-serif;background:linear-gradient(135deg,#1a1a2e 0%,#16213e 100%);color:#fff;min-height:100vh;padding:20px}";
  html += ".container{max-width:800px;margin:0 auto}";
  html += ".header{text-align:center;margin-bottom:40px}";
  html += ".header h1{font-size:2.5rem;font-weight:300;margin-bottom:10px;background:linear-gradient(45deg,#e94560,#f39c12);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text}";
  html += ".status-bar{display:flex;justify-content:center;gap:30px;margin:20px 0;flex-wrap:wrap}";
  html += ".status-item{background:rgba(255,255,255,0.1);backdrop-filter:blur(10px);padding:15px 25px;border-radius:12px;border:1px solid rgba(255,255,255,0.2);min-width:140px;text-align:center}";
  html += ".status-label{font-size:0.85rem;opacity:0.8;margin-bottom:5px;text-transform:uppercase;letter-spacing:1px}";
  html += ".status-value{font-size:1.2rem;font-weight:600;color:#e94560}";
  html += ".card{background:rgba(255,255,255,0.05);backdrop-filter:blur(20px);border:1px solid rgba(255,255,255,0.1);border-radius:20px;padding:30px;margin:20px 0;box-shadow:0 8px 32px rgba(0,0,0,0.3)}";
  html += ".card h2{font-size:1.5rem;margin-bottom:25px;color:#fff;display:flex;align-items:center;gap:10px}";
  html += ".form-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:20px}";
  html += ".form-group{margin-bottom:20px}";
  html += ".form-group label{display:block;margin-bottom:8px;font-weight:500;color:#b8c5d6;font-size:0.9rem;text-transform:uppercase;letter-spacing:0.5px}";
  html += ".form-group input{width:100%;padding:12px 16px;border:2px solid rgba(255,255,255,0.1);border-radius:8px;background:rgba(255,255,255,0.05);color:#fff;font-size:1rem;transition:all 0.3s ease}";
  html += ".form-group input:focus{outline:none;border-color:#e94560;background:rgba(255,255,255,0.1);box-shadow:0 0 0 3px rgba(233,69,96,0.2)}";
  html += ".form-group input:valid{border-color:#27ae60}";
  html += ".btn{background:linear-gradient(45deg,#e94560,#f39c12);border:none;padding:15px 30px;border-radius:12px;color:#fff;font-size:1rem;font-weight:600;cursor:pointer;transition:all 0.3s ease;text-transform:uppercase;letter-spacing:1px;width:100%;margin-top:20px}";
  html += ".btn:hover{transform:translateY(-2px);box-shadow:0 10px 25px rgba(233,69,96,0.4)}";
  html += ".btn:active{transform:translateY(0)}";
  html += ".pulse{animation:pulse 2s infinite}";
  html += "@keyframes pulse{0%{opacity:1}50%{opacity:0.7}100%{opacity:1}}";
  html += "@media(max-width:600px){.status-bar{flex-direction:column;align-items:center}.form-grid{grid-template-columns:1fr}.header h1{font-size:2rem}}";
  html += ".ip-validator{font-size:0.8rem;margin-top:5px;opacity:0.8}";
  html += ".valid{color:#27ae60}.invalid{color:#e74c3c}";
  
  html += "</style>";
  html += "</head>";
  html += "<body>";
  
  html += "<div class='container'>";
  html += "<header class='header'>";
  html += "<h1>■ ART-NET CONTROLLER</h1>";
  html += "<div class='status-bar'>";
  html += "<div class='status-item'>";
  html += "<div class='status-label'>Adresse IP</div>";
  html += "<div class='status-value'>" + config.ip.toString() + "</div>";
  html += "</div>";
  html += "<div class='status-item'>";
  html += "<div class='status-label'>Univers</div>";
  html += "<div class='status-value'>" + String(config.universe) + "</div>";
  html += "</div>";
  html += "<div class='status-item pulse'>";
  html += "<div class='status-label'>LEDs</div>";
  html += "<div class='status-value'>" + String(NUM_LEDS) + "</div>";
  html += "</div>";
  html += "<div class='status-item'>";
  html += "<div class='status-label'>Servos</div>";
  html += "<div class='status-value'>" + String(NUM_SERVOS) + "</div>";
  html += "</div>";
  html += "</div>";
  html += "</header>";
  
  html += "<div class='card'>";
  html += "<h2><span style='font-size:1.2em'>📡</span> Mapping Art-Net</h2>";
  html += "<div class='form-grid'>";
  html += "<div class='form-group'>";
  html += "<label>Canaux LEDs</label>";
  html += "<div style='background:rgba(52,152,219,0.1);border:1px solid rgba(52,152,219,0.3);border-radius:8px;padding:12px;color:#3498db;font-weight:600'>";
  html += "Canaux 1-192 (64 LEDs × RGB)";
  html += "</div>";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label>Canaux Servos</label>";
  html += "<div style='background:rgba(46,204,113,0.1);border:1px solid rgba(46,204,113,0.3);border-radius:8px;padding:12px;color:#2ecc71;font-weight:600'>";
  html += "Canaux 193-208 (16 Servos)";
  html += "</div>";
  html += "</div>";
  html += "</div>";
  html += "<div style='background:rgba(241,196,15,0.1);border:1px solid rgba(241,196,15,0.3);border-radius:8px;padding:15px;margin-top:15px'>";
  html += "<div style='color:#f1c40f;font-weight:600;margin-bottom:8px'>💡 Informations Techniques</div>";
  html += "<div style='color:#ecf0f1;font-size:0.9rem;line-height:1.4'>";
  html += "• Servos: Valeurs 0-255 → Angles 0°-180°<br>";
  html += "• Fréquence PWM: 50Hz (standard servo)<br>";
  html += "• I2C: SDA=Pin14, SCL=Pin15<br>";
  html += "• Contrôleur: PCA9685";
  html += "</div>";
  html += "</div>";
  html += "</div>";
  
  html += "<div class='card'>";
  html += "<h2><span style='font-size:1.2em'>🔧</span> Test Servos</h2>";
  html += "<div style='text-align:center'>";
  html += "<p style='color:#b8c5d6;margin-bottom:20px'>Testez le bon fonctionnement des 16 servo-moteurs</p>";
  html += "<form method='POST' action='/test-servos' style='display:inline'>";
  html += "<button type='submit' class='btn' style='background:linear-gradient(45deg,#27ae60,#2ecc71);width:auto;padding:12px 30px;display:inline-block'>▶ Lancer Test Servos</button>";
  html += "</form>";
  html += "<div style='font-size:0.85rem;color:#7f8c8d;margin-top:15px'>";
  html += "Le test effectue un balayage complet de tous les servos<br>";
  html += "Durée: ~10 secondes";
  html += "</div>";
  html += "</div>";
  html += "</div>";
  
  html += "<div class='card'>";
  html += "<h2><span style='font-size:1.2em'>⚙</span> Configuration Réseau</h2>";
  html += "<form action='/save' method='POST' id='configForm'>";
  html += "<div class='form-grid'>";
  
  html += "<div class='form-group'>";
  html += "<label for='universe'>Univers Art-Net</label>";
  html += "<input type='number' id='universe' name='universe' value='" + String(config.universe) + "' min='1' max='15' required>";
  html += "<div style='font-size:0.8rem;color:#f39c12;margin-top:5px;opacity:0.9'>Note: L'univers 0 n'est pas utilisable (reserve systeme)</div>";
  html += "</div>";
  
  html += "<div class='form-group'>";
  html += "<label for='ip'>Adresse IP</label>";
  html += "<input type='text' id='ip' name='ip' value='" + config.ip.toString() + "' pattern='^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$' required>";
  html += "<div class='ip-validator' id='ip-status'></div>";
  html += "</div>";
  
  html += "<div class='form-group'>";
  html += "<label for='gateway'>Passerelle</label>";
  html += "<input type='text' id='gateway' name='gateway' value='" + config.gateway.toString() + "' pattern='^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$' required>";
  html += "</div>";
  
  html += "<div class='form-group'>";
  html += "<label for='subnet'>Masque Réseau</label>";
  html += "<input type='text' id='subnet' name='subnet' value='" + config.subnet.toString() + "' pattern='^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$' required>";
  html += "</div>";
  
  html += "</div>";
  html += "<button type='submit' class='btn'>■ Sauvegarder Configuration</button>";
  html += "</form>";
  html += "</div>";
  
  html += "</div>";
  
  // JavaScript minimal pour validation
  html += "<script>";
  html += "function validateIP(ip){const regex=/^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;return regex.test(ip)}";
  html += "document.getElementById('ip').addEventListener('input',function(){const status=document.getElementById('ip-status');if(validateIP(this.value)){status.textContent='✓ Adresse IP valide';status.className='ip-validator valid'}else{status.textContent='✗ Format IP invalide';status.className='ip-validator invalid'}});";
  html += "document.getElementById('configForm').addEventListener('submit',function(e){const inputs=['ip','gateway','subnet'];for(let i of inputs){if(!validateIP(document.getElementById(i).value)){e.preventDefault();alert('Veuillez corriger les adresses IP invalides');return}}});";
  html += "</script>";
  
  html += "</body>";
  html += "</html>";
  
  server.send(200, "text/html; charset=utf-8", html);
}

void handleConfig() {
  String json = "{\"universe\":" + String(config.universe) + ",\"ip\":\"" + config.ip.toString() + "\",\"gateway\":\"" + config.gateway.toString() + "\",\"subnet\":\"" + config.subnet.toString() + "\"}";
  server.send(200, "application/json", json);
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
    
    artnet.forwardArtDmxDataToFastLED(config.universe, leds, NUM_LEDS);
    
    String html = "<!DOCTYPE html><html lang='fr'><head><meta charset='UTF-8'><title>Configuration Sauvegardée</title>";
    html += "<meta http-equiv='refresh' content='5;url=http://" + config.ip.toString() + "/'>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'></head>";
    html += "<body style='font-family:-apple-system,BlinkMacSystemFont,system-ui,sans-serif;margin:0;background:linear-gradient(135deg,#1a1a2e 0%,#16213e 100%);color:#fff;min-height:100vh;display:flex;align-items:center;justify-content:center'>";
    html += "<div style='background:rgba(255,255,255,0.05);backdrop-filter:blur(20px);border:1px solid rgba(255,255,255,0.1);border-radius:20px;padding:40px;text-align:center;max-width:500px;box-shadow:0 8px 32px rgba(0,0,0,0.3)'>";
    html += "<div style='font-size:4rem;margin-bottom:20px;background:linear-gradient(45deg,#27ae60,#2ecc71);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text'>✓</div>";
    html += "<h1 style='color:#27ae60;margin-bottom:20px;font-size:1.8rem;font-weight:300'>Configuration Sauvegardée</h1>";
    html += "<p style='margin:20px 0;color:#b8c5d6;font-size:1.1rem'>Le système redémarre avec la nouvelle configuration...</p>";
    html += "<div style='background:rgba(233,69,96,0.1);border:1px solid rgba(233,69,96,0.3);border-radius:12px;padding:15px;margin:20px 0'>";
    html += "<p style='color:#e94560;font-weight:600;font-size:1.2rem'>Nouvelle adresse: " + config.ip.toString() + "</p>";
    html += "</div>";
    html += "<p style='color:#7f8c8d;font-size:0.9rem;margin-top:30px'>Redirection automatique dans 5 secondes...</p>";
    html += "<div style='width:100%;height:4px;background:rgba(255,255,255,0.1);border-radius:2px;margin-top:20px;overflow:hidden'>";
    html += "<div style='height:100%;background:linear-gradient(45deg,#e94560,#f39c12);border-radius:2px;animation:progress 5s linear'></div></div>";
    html += "<style>@keyframes progress{0%{width:0}100%{width:100%}}</style>";
    html += "</div></body></html>";
    
    server.send(200, "text/html; charset=utf-8", html);
    
    delay(1000);
    ESP.restart();
  } else {
    server.send(400, "text/plain", "Paramètres manquants");
  }
}

void handleServoTest() {
  String html = "<!DOCTYPE html><html lang='fr'><head><meta charset='UTF-8'><title>Test Servos</title>";
  html += "<meta http-equiv='refresh' content='12;url=/'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'></head>";
  html += "<body style='font-family:-apple-system,BlinkMacSystemFont,system-ui,sans-serif;margin:0;background:linear-gradient(135deg,#1a1a2e 0%,#16213e 100%);color:#fff;min-height:100vh;display:flex;align-items:center;justify-content:center'>";
  html += "<div style='background:rgba(255,255,255,0.05);backdrop-filter:blur(20px);border:1px solid rgba(255,255,255,0.1);border-radius:20px;padding:40px;text-align:center;max-width:500px;box-shadow:0 8px 32px rgba(0,0,0,0.3)'>";
  html += "<div style='font-size:4rem;margin-bottom:20px;background:linear-gradient(45deg,#27ae60,#2ecc71);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text'>⚡</div>";
  html += "<h1 style='color:#27ae60;margin-bottom:20px;font-size:1.8rem;font-weight:300'>Test Servos en Cours</h1>";
  html += "<p style='margin:20px 0;color:#b8c5d6;font-size:1.1rem'>Test de balayage des 16 servo-moteurs...</p>";
  html += "<div style='background:rgba(46,204,113,0.1);border:1px solid rgba(46,204,113,0.3);border-radius:12px;padding:15px;margin:20px 0'>";
  html += "<p style='color:#2ecc71;font-weight:600;font-size:1.2rem'>Vérifiez le mouvement des servos</p>";
  html += "</div>";
  html += "<p style='color:#7f8c8d;font-size:0.9rem;margin-top:30px'>Retour automatique dans 12 secondes...</p>";
  html += "<div style='width:100%;height:4px;background:rgba(255,255,255,0.1);border-radius:2px;margin-top:20px;overflow:hidden'>";
  html += "<div style='height:100%;background:linear-gradient(45deg,#27ae60,#2ecc71);border-radius:2px;animation:progress 12s linear'></div></div>";
  html += "<style>@keyframes progress{0%{width:0}100%{width:100%}}</style>";
  html += "</div></body></html>";
  
  server.send(200, "text/html; charset=utf-8", html);
  
  // Lancer le test des servos en arrière-plan
  Serial.println("🔧 Test servos lancé via interface web");
  testServos();
}

void loadConfig() {
  if (SPIFFS.exists("/config.json")) {
    File file = SPIFFS.open("/config.json", "r");
    if (file) {
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, file);
      config.universe = doc["universe"] | 1;
      
      String ip = doc["ip"] | "192.168.1.201";
      String gateway = doc["gateway"] | "192.168.1.1";
      String subnet = doc["subnet"] | "255.255.255.0";
      
      config.ip.fromString(ip);
      config.gateway.fromString(gateway);
      config.subnet.fromString(subnet);
      
      file.close();
      Serial.println("Configuration chargée");
    }
  }
}

void saveConfig() {
  File file = SPIFFS.open("/config.json", "w");
  if (file) {
    DynamicJsonDocument doc(1024);
    doc["universe"] = config.universe;
    doc["ip"] = config.ip.toString();
    doc["gateway"] = config.gateway.toString();
    doc["subnet"] = config.subnet.toString();
    serializeJson(doc, file);
    file.close();
    Serial.println("Configuration sauvegardée");
  }
}

// Fonction de vérification de la santé I2C
void checkI2CHealth() {
  Wire.beginTransmission(0x40); // Adresse par défaut du PCA9685
  uint8_t error = Wire.endTransmission();
  
  bool wasHealthy = i2cHealthy;
  i2cHealthy = (error == 0);
  
  if (wasHealthy && !i2cHealthy) {
    Serial.println("⚠️  Erreur I2C détectée - Tentative de récupération");
    // Tentative de réinitialisation
    Wire.begin(14, 15); // (SDA, SCL)
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
  } else if (!wasHealthy && i2cHealthy) {
    Serial.println("✅ I2C récupéré");
  }
}

// Fonction de mise à jour optimisée de tous les servos
void updateAllServos() {
  if (!i2cHealthy) {
    Serial.println("I2C non disponible - servos ignorés");
    return;
  }
  
  for (int servo = 0; servo < NUM_SERVOS; servo++) {
    setServoPosition(servo, servoValues[servo]);
  }
}

// Fonction de mapping Art-Net (0-255) vers position servo
void setServoPosition(uint8_t servoNum, uint8_t artnetValue) {
  if (servoNum >= NUM_SERVOS || !i2cHealthy) return;
  
  // Debug: afficher les valeurs reçues
  Serial.printf("Servo %d: DMX=%d ", servoNum, artnetValue);
  
  // Mapping Art-Net (0-255) vers pulse PWM (SERVOMIN - SERVOMAX)
  // Utiliser une conversion plus précise pour éviter les erreurs d'arrondi
  uint16_t pulseLength = SERVOMIN + ((uint32_t)(artnetValue) * (SERVOMAX - SERVOMIN)) / 255;
  
  // Sécurité : s'assurer que la valeur reste dans les limites
  pulseLength = constrain(pulseLength, SERVOMIN, SERVOMAX);
  
  Serial.printf("PWM=%d\n", pulseLength);
  
  // Appliquer la position au servo via PCA9685
  pwm.setPWM(servoNum, 0, pulseLength);
  
  // Debug optionnel (désactivable en production)
  #ifdef DEBUG_SERVO
  Serial.printf("Servo %d: Art-Net=%d → Pulse=%d\n", servoNum, artnetValue, pulseLength);
  #endif
}

// Fonction de test servo (pour validation)
void testServos() {
  Serial.println("Test des servos - balayage complet");
  for (int pos = 0; pos <= 255; pos += 5) {
    for (int servo = 0; servo < NUM_SERVOS; servo++) {
      setServoPosition(servo, pos);
    }
    delay(20);
  }
  
  // Retour à la position centrale
  for (int servo = 0; servo < NUM_SERVOS; servo++) {
    setServoPosition(servo, 127);
  }
  Serial.println("Test servos terminé");
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("wt32-eth01");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      Serial.print(", Full Duplex: ");
      Serial.print(ETH.fullDuplex() ? "YES" : "NO");
      Serial.print(", Link Speed: ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      break;
    default:
      break;
  }
}