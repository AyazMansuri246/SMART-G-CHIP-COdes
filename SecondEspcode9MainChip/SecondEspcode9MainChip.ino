// Touch Sensor, ESP-NOW communication, I2S mic input, and OLED display code for ESP32 WROOM  

#include <WiFi.h>
#include <driver/i2s.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HTTPClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <esp_now.h>

// for differentiating between the audio and command
#define PKT_CONTROL 0x01
#define PKT_AUDIO 0x02
#define PKT_IMAGE 0x03

#define RXD2 16
#define TXD2 17

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

AsyncWebServer asyncServer(80);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Bitmap buffer
// uint8_t imageBuffer[1024];

// for mic
#define I2S_WS 25
#define I2S_SCK 26
#define I2S_SD 33

#define SAMPLE_RATE 16000
#define RECORD_SECONDS 30
#define BYTES_PER_SEC (SAMPLE_RATE * 2)
#define TOTAL_BYTES (BYTES_PER_SEC * RECORD_SECONDS)

#define CHUNK_SIZE 244  // bytes per ESP-NOW packet

float currentGain = 4.0;
#define TARGET_LEVEL 8000
#define MAX_GAIN 4.0
#define MIN_GAIN 1.5

bool liveMode = false;  // true when triple tap active
bool videoRecording = false;  // global

int16_t audioSamples[CHUNK_SIZE / 2];

size_t totalSent = 0;

uint16_t audioPacketSeq = 0;


// -------- TOUCH CONFIG --------
#define TOUCH_PIN 13
#define DEBOUNCE_TIME 50
#define DOUBLE_TAP_TIME 400
#define LONG_PRESS_TIME 2000
#define CMD_TRIPLE_TAP "TRIPLE_TAP"


bool isTouching = false;
bool isRecording = false;
unsigned long touchStartTime = 0;
unsigned long lastTouchChange = 0;
uint8_t tapCount = 0;
unsigned long lastTapTime = 0;

// for audio 
#define AUDIO_RING_SIZE 8192
uint8_t audioRingBuffer[AUDIO_RING_SIZE];

volatile int audioWritePos = 0;
volatile int audioReadPos = 0;
volatile int audioBufferedBytes = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// for live conversation, establishing the web socket
AsyncWebSocket ws("/ws");

bool wsClientConnected = false;
bool liveConversation = false;     // true when triple tap streaming



// -------- CAM MAC (CHANGE THIS) --------
// ESP32 cam MAC: 88:57:21:C3:9C:28
uint8_t camMAC[] = { 0x88, 0x57, 0x21, 0xC3, 0x9C, 0x28 };

// -------- Message structure --------
typedef struct {
  char cmd[16];
} ControlMsg;

// -------- Receive callback (ACKs) --------
void onReceive(const esp_now_recv_info* info, const uint8_t* incomingData, int len) {

  ControlMsg msg;
  memcpy(&msg, incomingData, sizeof(msg));

  Serial.print("CAM says: ");
  Serial.println(msg.cmd);
}


// -------- Init ESP-NOW (blocking) --------
void initEspNow() {
  while (true) {
    if (esp_now_init() == ESP_OK) {
      Serial.println("ESP-NOW initialized");
      break;
    }
    Serial.println("ESP-NOW init failed, retrying...");
    delay(1000);
  }
}

// -------- Add peer (blocking) --------
void addPeer() {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, camMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  while (true) {
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      Serial.println("Peer added");
      break;
    }
    Serial.println("Peer add failed, retrying...");
    delay(1000);
  }
}

// -------- Send command --------
void sendCmd(const char *cmd) {
  uint8_t packet[1 + sizeof(ControlMsg)];
  packet[0] = PKT_CONTROL;

  ControlMsg msg;
  memset(&msg, 0, sizeof(msg));
  strncpy(msg.cmd, cmd, sizeof(msg.cmd) - 1);

  memcpy(packet + 1, &msg, sizeof(msg));
  esp_now_send(camMAC, packet, sizeof(packet));
}



String camIPStr = "";  // CAM ip

uint32_t activeClientId = 0;
uint8_t wroomMac[6] = {0};
uint32_t camClientId = 0;      // existing phone client

uint8_t jpegBuffer[25536];
size_t jpegBytesReceived = 0;

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WS Client connected: %u\n", client->id());
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("WS Client disconnected: %u\n", client->id());

      if (client->id() == activeClientId) {
        wsClientConnected = false;
        activeClientId = 0;
        if (liveMode) {
          Serial.println("Phone disconnected → stopping live mode");
          liveMode = false;
          isRecording = false;
          totalSent = 0;
        }
      }

      if (client->id() == camClientId) {
        camClientId = 0;
        Serial.println("CAM disconnected");
      }
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo*)arg;

      if (info->opcode == WS_TEXT) {
        // null terminate for safe string compare
        data[len] = 0;

        if (strcmp((char*)data, "I_AM_CAM") == 0) {
          camClientId = client->id();
          camIPStr = client->remoteIP().toString();  // ← grab real IP
          Serial.println("CAM IP: " + camIPStr);
          Serial.println("CAM connected via WS");

        } else if (strcmp((char*)data, "PING") == 0) {
          client->text("PONG");
          Serial.println("WS: Received PING, sent PONG");
          activeClientId = client->id();
          wsClientConnected = true;
          Serial.println("Phone connected via WS");


        } else if (strncmp((char*)data, "TRANS:", 6) == 0) {
          Serial.printf("WS Text Translation: %s\n", (char*)data + 6);
        } else if (strcmp((char*)data, "IMG_CAPTURED") == 0) {
            Serial.println("CAM confirmed Img captured");
            display.clearDisplay();
            display.setTextSize(2);
            display.setTextColor(WHITE);
            display.setCursor(10,10);
            display.println("Image Captured..");
            display.display();
        } 
        // else if (strcmp((char*)data, "REC_STARTED") == 0) {
        //     Serial.println("CAM confirmed recording started");
        //     isRecording = true;
        //     videoRecording = true;   // ← start audio to CAM
        //     totalSent = 0;
            // display.clearDisplay();
            // display.setTextSize(2);
            // display.setTextColor(WHITE);
            // display.setCursor(10,10);
            // display.println("Video Recording");
            // display.println("Started..");
            // display.display();
        // } else if (strcmp((char*)data, "REC_STOPPED") == 0) {
        //     Serial.println("CAM confirmed recording stopped");
            
        //     display.clearDisplay();
        //     display.setTextSize(2);
        //     display.setTextColor(WHITE);
        //     display.setCursor(10,10);
        //     display.println("Video Recording");
        //     display.println("Stopped..");
        //     display.display();
        // }


      } else if (info->opcode == WS_BINARY) {

        if (client->id() == camClientId) {
          // First fragment — reset
          if (info->index == 0) {
            jpegBytesReceived = 0;
          }

          // Guard against overflow
          if (jpegBytesReceived + len <= sizeof(jpegBuffer)) {
            memcpy(jpegBuffer + jpegBytesReceived, data, len);
            jpegBytesReceived += len;
          } else {
            Serial.println("❌ JPEG buffer overflow, dropping");
            jpegBytesReceived = 0;
            return;
          }

          // Last fragment — forward to phone
          if (info->index + len == info->len) {
            AsyncWebSocketClient *phone = ws.client(activeClientId);
            if (phone && phone->canSend()) {
              phone->binary(jpegBuffer, jpegBytesReceived);
              Serial.printf("✅ Full JPEG forwarded: %u bytes\n", jpegBytesReceived);
            } else {
              Serial.println("Phone not connected, dropping image");
            }
            jpegBytesReceived = 0;
          }

        } else if (client->id() == activeClientId) {
          // OLED bitmap from phone
          if (len == 1024) {
            display.clearDisplay();
            display.drawBitmap(0, 0, data, 128, 64, WHITE);
            display.display();
            Serial.println("✅ Image displayed on OLED");
          }
        }
      }
      break;
    }

    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void sendToCAM(const char* cmd) {
  AsyncWebSocketClient* camClient = ws.client(camClientId);
  if (camClient && camClient->canSend()) {
    camClient->text(cmd);
  } else {
    Serial.println("CAM not connected");
  }
}



void setupRoutes() {

  // /images → small JSON, safe to proxy through HTTPClient (no change needed)
  asyncServer.on("/images", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (camIPStr == "") { req->send(503, "text/plain", "CAM not connected"); return; }
    HTTPClient http;
    http.begin("http://" + camIPStr + "/images");
    int code = http.GET();
    req->send(code == 200 ? 200 : 502, "application/json",
              code == 200 ? http.getString() : "CAM error");
    http.end();
  });

  // /image → redirect phone directly to CAM (avoids large buffer proxying)
  asyncServer.on("/image", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("name")) {
      req->send(400, "text/plain", "Missing name");
      return;
    }
    if (camIPStr == "") {
      req->send(503, "text/plain", "CAM not connected");
      return;
    }
    String name = req->getParam("name")->value();
    // Direct redirect to CAM — phone can reach it since all are on same AP
    req->redirect("http://" + camIPStr + "/image?name=" + name);
  });

  // /videos → small JSON, safe to proxy (no change needed)
  asyncServer.on("/videos", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (camIPStr == "") { req->send(503, "text/plain", "CAM not connected"); return; }
    HTTPClient http;
    http.begin("http://" + camIPStr + "/videos");
    int code = http.GET();
    req->send(code == 200 ? 200 : 502, "application/json",
              code == 200 ? http.getString() : "CAM error");
    http.end();
  });

  // /download → redirect phone directly to CAM (videos/audio can be very large)
  asyncServer.on("/download", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("folder") || !req->hasParam("type")) {
      req->send(400, "text/plain", "Missing params");
      return;
    }
    if (camIPStr == "") {
      req->send(503, "text/plain", "CAM not connected");
      return;
    }
    String folder = req->getParam("folder")->value();
    String type   = req->getParam("type")->value();
    // Direct redirect to CAM for large file download
    req->redirect("http://" + camIPStr + "/download?folder=" + folder + "&type=" + type);
  });

  asyncServer.onNotFound([](AsyncWebServerRequest *req) {
    req->send(404, "text/plain", "Not found");
  });
}


#define MAX_IMG_SIZE 20000
uint8_t incomingImage[MAX_IMG_SIZE];
volatile size_t imageSize = 0;
volatile bool imageReady = false;




bool waitingForSecondTap = false;
unsigned long firstTapTime = 0;



//for live translation
void handleTripleTap() {

  if (!wsClientConnected) {
    Serial.println("Cannot start live mode → Phone not connected");
    return;
  }

  // ---------- TOGGLE LIVE MODE ----------
  if (liveMode) {
    Serial.println("Stopping Live Mode");

    liveMode = false;
    isRecording = false;
    totalSent = 0;

  } else {
    Serial.println("Starting Live Mode");

    liveMode = true;
    isRecording = true;
    totalSent = 0;
    audioPacketSeq = 0;

  }
}

// -------- Touch logic --------
void handleTouch(){

  if (millis() - lastTouchChange < DEBOUNCE_TIME) return;

  bool touchState = digitalRead(TOUCH_PIN);

  // ---------- TOUCH START ----------
  if (touchState && !isTouching) {
    isTouching = true;
    touchStartTime = millis();
    lastTouchChange = millis();
  }

  // ---------- TOUCH RELEASE ----------
  if (!touchState && isTouching) {
    isTouching = false;
    lastTouchChange = millis();

    unsigned long pressDuration = millis() - touchStartTime;

    // ===== LONG PRESS =====
    if (pressDuration >= LONG_PRESS_TIME) {
      tapCount = 0;  // cancel taps

      if (liveMode) {
        Serial.println("Cannot start video during live mode");
        return;
      }

      Serial.println("Long press");
      handleLongPress();
      return;
    }

    // ===== SHORT TAP =====
    if (!isRecording) {
      tapCount++;
      lastTapTime = millis();
    }
  }

  // ---------- TAP EVALUATION ----------
  if (tapCount > 0 && (millis() - lastTapTime > DOUBLE_TAP_TIME)) {

    if (tapCount == 2) {
      Serial.println("Double tap");
      handleTap();

    } else if (tapCount == 3) {
      if (isRecording && !liveMode) {
        Serial.println("Cannot start live mode during video recording");
      } else {
        Serial.println("Triple tap");
        handleTripleTap();
      }
    }

    tapCount = 0;
  }
}



void handleTap() {
  if (camClientId != 0) {
    if (wsClientConnected) {
      sendToCAM("REQ_IMG");
    } else {
      sendToCAM("CAPTURE_SD");
    }
  } else {
    Serial.println("CAM not connected, ignoring tap");
  }
}



void handleLongPress() {
  if (isRecording) {
    sendCmd("STOP_REC");
    isRecording = false;
    videoRecording = false;   // ← stop audio path
    totalSent = 0;
  } else {
    sendCmd("START_REC");
    isRecording = true;
    videoRecording = true;    // ← start audio path
    totalSent = 0;
  }
}

// functions for mic
void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}



// void audioTask(void *pvParameters) {

//   uint8_t audioSamples[512];

//   while (true) {

//     if (!liveMode && !isRecording) {
//       vTaskDelay(10);
//       continue;
//     }

//     size_t bytesRead = 0;

//     i2s_read(I2S_NUM_0, audioSamples, 512, &bytesRead, portMAX_DELAY);

//     if (bytesRead > 0) {

//       portENTER_CRITICAL(&mux);

//       for (int i = 0; i < bytesRead; i++) {

//         if (audioBufferedBytes < AUDIO_RING_SIZE) {
//           audioRingBuffer[audioWritePos] = audioSamples[i];
//           audioWritePos = (audioWritePos + 1) % AUDIO_RING_SIZE;
//           audioBufferedBytes++;
//         }
//         // else: overflow → drop data (important)

//       }

//       portEXIT_CRITICAL(&mux);
//     }
//   }
// }

void setup() {
  Serial.begin(115200);
  Serial.flush();
  delay(1000);

  Serial.println("\n\n===== SYSTEM START =====");
  WiFi.mode(WIFI_AP_STA); //for both ESP-NOW and AP
  delay(100);

  initEspNow();
  addPeer();
  esp_now_register_recv_cb(onReceive);

  // Start Access Point
  const char *ap_ssid = "ESP32_Wroom_AP";
  const char *ap_pass = "12345678";

  WiFi.softAP(ap_ssid, ap_pass);

  pinMode(TOUCH_PIN, INPUT_PULLUP);
  setupI2S();

  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP Address: ");
  Serial.println(ip);
  ws.onEvent(onWsEvent);
  asyncServer.addHandler(&ws);
  setupRoutes();
  asyncServer.begin();

  Serial.println("Initializing I2C...");
  Wire.begin(21, 22);   // SDA, SCL

  Serial.println("Scanning for OLED...");

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("❌ OLED not found at 0x3C");
    while(true);
  }

  Serial.println("✅ OLED initialized successfully");

  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10,10);

  display.println("Hello");
  display.println(WiFi.softAPIP());

  display.display();

  Serial.println("Initial text displayed");

  // xTaskCreatePinnedToCore(
  //   audioTask,
  //   "audioTask",
  //   4096,
  //   NULL,
  //   3,
  //   NULL,
  //   0
  // );


  Serial.println("HTTP server started at wroom (AP mode)");
  Serial.println("Everything good to go !!! Ayazzz..");
}


void loop() {

  handleTouch();
  ws.cleanupClients();

  if (imageReady && activeClientId != 0) {
    AsyncWebSocketClient *client = ws.client(activeClientId);
    if (client && client->canSend()) {
      client->binary(incomingImage, imageSize);
      delay(10);
      Serial.println("📡 Image sent to phone");
      imageReady = false;
      imageSize = 0;
    }
  }

  // do nothing if not recording, it is for audio
  if (!isRecording) {
    delay(5);
    return;
  }

  static unsigned long lastSend = 0;
  const unsigned long CHUNK_INTERVAL_MS = 16;

  // neither mode active → nothing to do
  if (!liveMode && !videoRecording)
    return;

  // if (millis() - lastSend < CHUNK_INTERVAL_MS)
  //   return;

  // if (audioBufferedBytes < CHUNK_SIZE)
  //   return;

  // read chunk from ring buffer once
  // uint8_t chunk[CHUNK_SIZE];

  // portENTER_CRITICAL(&mux);
  // for (int i = 0; i < CHUNK_SIZE; i++) {
  //   chunk[i] = audioRingBuffer[audioReadPos];
  //   audioReadPos = (audioReadPos + 1) % AUDIO_RING_SIZE;
  //   audioBufferedBytes--;
  // }
  // portEXIT_CRITICAL(&mux);

  // int16_t* samples = (int16_t*)chunk;
  size_t bytesRead;
  i2s_read(I2S_NUM_0, audioSamples, CHUNK_SIZE, &bytesRead, portMAX_DELAY);

  int samples = bytesRead / 2;

  // ---------- measure peak ----------
  int32_t peak = 0;
  for (int i = 0; i < samples; i++) {
    int32_t s = abs(audioSamples[i]);
    if (s > peak) peak = s;
  }

  // ---------- AGC ----------
  if (peak > 0) {
    float desiredGain = (float)TARGET_LEVEL / peak;
    currentGain = currentGain * 0.9 + desiredGain * 0.1;

    if (currentGain > MAX_GAIN) currentGain = MAX_GAIN;
    if (currentGain < MIN_GAIN) currentGain = MIN_GAIN;
  }

  // ---------- apply gain ----------
  for (int i = 0; i < samples; i++) {
    int32_t sample = audioSamples[i] * currentGain;

    if (sample > 32767) sample = 32767;
    if (sample < -32768) sample = -32768;

    audioSamples[i] = (int16_t)sample;
  }


  // send to phone if live mode
  if (liveMode && activeClientId != 0) {
    AsyncWebSocketClient *phone = ws.client(activeClientId);
    if (phone && phone->canSend()) {
      phone->binary((const uint8_t*)audioSamples, bytesRead);
    }
  }

  // send to CAM if video recording
  if (videoRecording) {
    
    // ---------- send audio with sequence number ----------
    uint8_t packet[1 + 2 + CHUNK_SIZE];

    packet[0] = PKT_AUDIO;

    // add sequence number (little endian)
    packet[1] = audioPacketSeq & 0xFF;
    packet[2] = (audioPacketSeq >> 8) & 0xFF;

    memcpy(packet + 3, audioSamples, bytesRead);

    esp_now_send(camMAC, packet, bytesRead + 3);

    audioPacketSeq++;
    totalSent += bytesRead;

    // pacing (VERY IMPORTANT)
    // delay(2);  // ~512 bytes at 16kHz ≈ 16ms

  }
  if (millis() - lastSend < 14) return;
  lastSend = millis();
}