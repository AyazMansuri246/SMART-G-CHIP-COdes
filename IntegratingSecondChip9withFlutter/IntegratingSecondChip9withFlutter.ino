#include "esp_camera.h"
#include <WiFi.h>
#include <stdio.h>
#include <string.h>
#include "FS.h"
#include "SD_MMC.h"
#include <esp_now.h>
#include <WebSocketsClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

WebSocketsClient wsClient;
bool wsConnected = false;

String currentVideoDir = "";
uint32_t imageCounter = 0;
uint32_t videoCounter = 0;
#define COUNTER_FILE "/counter.txt"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#define LED_PIN 4  
#define CHUNK_SIZE 240

#define PKT_CONTROL 0x01
#define PKT_AUDIO 0x02
#define PKT_IMAGE 0x03

SemaphoreHandle_t sdMutex = NULL;
volatile bool sdBusy = false;

#define MAX_RECORD_TIME 30000 // 30s

volatile bool doCapture = false;
volatile bool doSendImg = false;

// AsyncWebSocket ws("/ws");


volatile bool stopRequested = false;
volatile bool isRecording = false;

volatile bool recordingDone = false;
TaskHandle_t recordTaskHandle = NULL;  //parallel execution

#ifndef AUDIO_RING_SIZE
  #define AUDIO_RING_SIZE 16384   // 16KB, tune down to 8192 if memory tight
#endif

uint8_t audioRingBuffer[AUDIO_RING_SIZE];
volatile size_t audioWritePos = 0;
volatile size_t audioReadPos  = 0;
volatile size_t audioBufferedBytes = 0; // number of bytes currently in buffer

// for audio
File audioFile;
uint32_t audioBytes = 0;
bool audioRecording = false;

#define SAMPLE_RATE 16000


// ---------------- WAV HEADER ----------------
void writeWavHeader(File &file, uint32_t dataSize) {
  uint32_t fileSize = dataSize + 36;

  uint16_t audioFormat   = 1;     // PCM
  uint16_t numChannels   = 1;     // mono
  uint32_t sampleRate    = SAMPLE_RATE;
  uint32_t byteRate      = SAMPLE_RATE * 2;
  uint16_t blockAlign    = 2;
  uint16_t bitsPerSample = 16;

  file.seek(0);

  file.write((const uint8_t *)"RIFF", 4);
  file.write((uint8_t *)&fileSize, 4);
  file.write((const uint8_t *)"WAVE", 4);
  file.write((const uint8_t *)"fmt ", 4);

  uint32_t subChunk1Size = 16;
  file.write((uint8_t *)&subChunk1Size, 4);
  file.write((uint8_t *)&audioFormat, 2);
  file.write((uint8_t *)&numChannels, 2);
  file.write((uint8_t *)&sampleRate, 4);
  file.write((uint8_t *)&byteRate, 4);
  file.write((uint8_t *)&blockAlign, 2);
  file.write((uint8_t *)&bitsPerSample, 2);

  file.write((const uint8_t *)"data", 4);
  file.write((uint8_t *)&dataSize, 4);
}


// Video constraints
const int fps = 10;        // frames per second (target)
const int frame_interval = 1000 / fps;

// Global variables
File videoFile;
#define PATH_LEN 96
char tmpFileName[PATH_LEN];
char finalFileName[PATH_LEN];
int frameCount = 0;
unsigned long startTime;
uint32_t movi_size = 0;
uint32_t jpeg_size = 0;

// Index entry structure for AVI
struct avi_idx1_entry {
  uint32_t chunk_id;
  uint32_t flags;
  uint32_t offset;
  uint32_t size;
};

// We will store the index in a temporary file to save RAM
File indexFile;

// Async web server 
AsyncWebServer asyncServer(80);


void recordVideoTask(void *pvParameters); // forward prototype

// Quick helper: recording-in-progress response  
void respondRecording(AsyncWebServerRequest *req) {
  req->send(503, "application/json", "{\"status\":\"recording_in_progress\",\"message\":\"Recording in progress. Try again later.\"}");
}

typedef struct {
  char cmd[20];
} ControlMsg;


void onWsEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    wsConnected = true;
    wsClient.sendTXT("I_AM_CAM");   // ✅ identify itself
    Serial.println("WS Connected to WROOM");
  }
  else if (type == WStype_DISCONNECTED) {
    wsConnected = false;
    Serial.println("WS Disconnected");
  }
  else if (type == WStype_TEXT) {
    Serial.printf("CMD: %s\n", payload);

    if (strcmp((char*)payload, "REQ_IMG") == 0) {
      doSendImg = true;
    }
    else if (strcmp((char*)payload, "CAPTURE_SD") == 0) {
      doCapture = true;
      wsClient.sendTXT("IMG_CAPTURED");
    }
    else if (strcmp((char*)payload, "START_REC") == 0) {
      Serial.println("Video start Request");
      startVideo();
      audioRecording = true;   // enable audio ring buffer
      wsClient.sendTXT("REC_STARTED");
    }
    else if (strcmp((char*)payload, "STOP_REC") == 0) {
      Serial.println("Video stop Request");
      stopVideo();
      Serial.println("Video stop Request");
      audioRecording = false;
      wsClient.sendTXT("REC_STOPPED");
    }
  }

  // ✅ Audio chunks from WROOM mic
  else if (type == WStype_BIN) {
    if (!audioRecording) return;  // ignore if not recording

    for (int i = 0; i < length; i++) {
      if (audioBufferedBytes >= AUDIO_RING_SIZE) {
        audioReadPos = (audioReadPos + 1) % AUDIO_RING_SIZE;
        audioBufferedBytes--;
      }
      audioRingBuffer[audioWritePos] = payload[i];
      audioWritePos = (audioWritePos + 1) % AUDIO_RING_SIZE;
      audioBufferedBytes++;
    }
  }
}

uint8_t wroomMac[6] = {0};
// bool liveConversation = false;


void captureAndSendImage() {
  flashTwice();
  delay(60);
  for (int i = 0; i < 3; i++) {
    camera_fb_t* temp = esp_camera_fb_get();
    if (temp) esp_camera_fb_return(temp);
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    return;
  }

  Serial.printf("JPEG: %u bytes\n", fb->len);

  if (wsConnected) {
    wsClient.sendBIN(fb->buf, fb->len);  // ✅ single send, no chunking needed
    Serial.println("Image sent over WebSocket");
    Serial.println(fb->len);
  } else {
    Serial.println("WS not connected, skipping send");
  }

  esp_camera_fb_return(fb);
}


void flashTwice() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }
}

void captureImage() {

  // Flash
  flashTwice();
  delay(60);   //for stability
  //  STEP 1: Flush old frames
  for (int i = 0; i < 3; i++) {
    camera_fb_t * temp = esp_camera_fb_get();
    if (temp) esp_camera_fb_return(temp);
  }
  // STEP 2: Capture fresh frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("❌ Camera capture failed");
    return;
  }
  
  // SAVE TO SD
  Serial.println("Phone NOT connected → saving to SD");

  imageCounter++;

  char buffer[30];
  sprintf(buffer, "/Images/img_%03d.jpg", imageCounter);
  String path = String(buffer);

  File file = SD_MMC.open(path, FILE_WRITE);

  if (!file) {
    Serial.println("❌ Failed to open file");

    // 🔥 IMPORTANT: release before returning
    esp_camera_fb_return(fb);
    return;
  }

  file.write(fb->buf, fb->len);
  file.flush();
  file.close();

  Serial.println("✅ Image saved: " + path);

  saveCounters();



  // 🔥 ALWAYS release buffer
  esp_camera_fb_return(fb);
}


void loadCounters() {
  File file = SD_MMC.open(COUNTER_FILE);

  if (!file) {
    Serial.println("No counter file, starting from 0");
    imageCounter = 0;
    videoCounter = 0;
    return;
  }

  imageCounter = file.readStringUntil('\n').toInt();
  videoCounter = file.readStringUntil('\n').toInt();
  
  file.close();

  Serial.printf("Loaded Counters -> Image: %u, Video: %u\n", imageCounter, videoCounter);
}

void saveCounters() {
  
  //  Ensure overwrite (no append confusion)
  // SD_MMC.remove(COUNTER_FILE);

  File file = SD_MMC.open(COUNTER_FILE, FILE_WRITE);

  if (!file) {
    Serial.println("Failed to save counters");
    return;
  }

  file.println(imageCounter);
  file.println(videoCounter);
  file.flush();
  file.close();
  Serial.println("Counters saved");
}

// ---------- Recording control ----------
void startVideo() {

  // Create timestamped directory
  // String ts = getTimeStamp();
  videoCounter++;
  char buffer[20];
  sprintf(buffer, "/Videos/Video_%03d", videoCounter);
  currentVideoDir = String(buffer);
  // currentVideoDir = "/Videos/Video_" + String(videoCounter);

  if (!SD_MMC.exists(currentVideoDir)) {
    if (!SD_MMC.mkdir(currentVideoDir)) {
      Serial.println("❌ Failed to create video directory");
      return;
    }
  }

  Serial.println("📁 Video dir: " + currentVideoDir);
  saveCounters();

  // Prepare file paths
  snprintf(tmpFileName, sizeof(tmpFileName),
           "%s/video.tmp", currentVideoDir.c_str());

  snprintf(finalFileName, sizeof(finalFileName),
           "%s/video.avi", currentVideoDir.c_str());

  if (SD_MMC.exists(tmpFileName) || SD_MMC.exists(finalFileName)) {
    Serial.println("❌ Video file already exists");
    return;
  }

  Serial.printf("🎥 Recording to (tmp): %s\n", tmpFileName);

  // Open temp video file ONLY
  videoFile = SD_MMC.open(tmpFileName, FILE_WRITE);
  if (!videoFile) {
    Serial.println("❌ Failed to open temp video file");
    return;
  }

  // Reset audio ring buffer state
  audioWritePos = 0;
  audioReadPos  = 0;
  audioBufferedBytes = 0;
  audioBytes = 0;

  audioRecording = true;   // allow ESP-NOW audio packets

  // Recording state
  isRecording = true;
  recordingDone = false;

  // Start recording task (single SD writer)
  xTaskCreatePinnedToCore(
    recordVideoTask,
    "recTask",
    4096 * 4,
    NULL,
    2,
    &recordTaskHandle,
    1
  );

  Serial.println("✅ Video + audio capture started");
}


// The recording task: capture loop and finalization (only this task writes final AVI header)
void recordVideoTask(void *pvParameters) {

  digitalWrite(LED_PIN, HIGH);

  /* ================== OPEN INDEX FILE ================== */
  if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
  indexFile = SD_MMC.open("/Videos/idx.tmp", FILE_WRITE);
  if (sdMutex) xSemaphoreGive(sdMutex);

  if (!indexFile) {
    Serial.println("❌ Failed to open idx.tmp");
    videoFile.close();
    isRecording = false;
    recordingDone = true;
    vTaskDelete(NULL);
    return;
  }

  /* ================== OPEN AUDIO FILE ================== */
  String audioPath = currentVideoDir + "/audio.wav";
  if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
  audioFile = SD_MMC.open(audioPath, FILE_WRITE);
  if (audioFile) {
    writeWavHeader(audioFile, 0);  // placeholder
    Serial.println("🎧 Audio file opened");
  } else {
    Serial.println("❌ Failed to open audio.wav");
  }
  if (sdMutex) xSemaphoreGive(sdMutex);

  /* ================== AVI HEADER PLACEHOLDER ================== */
  uint8_t headerBuf[250];
  memset(headerBuf, 0, sizeof(headerBuf));
  if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
  videoFile.write(headerBuf, sizeof(headerBuf));
  videoFile.flush();
  if (sdMutex) xSemaphoreGive(sdMutex);

  movi_size = 0;
  frameCount = 0;
  startTime = millis();
  unsigned long lastFrameTimeLocal = 0;

  Serial.println("🎥 Recording started");

  /* ================== MAIN RECORD LOOP ================== */
  while ((millis() - startTime < MAX_RECORD_TIME) && isRecording) {

    /* ---------- VIDEO FRAME ---------- */
    if (millis() - lastFrameTimeLocal >= frame_interval) {
      lastFrameTimeLocal = millis();

      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) continue;

      uint32_t dc_id = 0x63643030; // '00dc'
      uint32_t chunk_size = fb->len;
      uint32_t padding = (4 - (chunk_size % 4)) % 4;
      uint32_t total_chunk_size = chunk_size + padding;

      // write video chunk (take mutex only for the actual writes)
      if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
      videoFile.write((uint8_t*)&dc_id, 4);
      videoFile.write((uint8_t*)&total_chunk_size, 4);
      videoFile.write(fb->buf, fb->len);
      if (padding) {
        uint8_t zero = 0;
        for (int i = 0; i < padding; i++) videoFile.write(&zero, 1);
      }
      if (sdMutex) xSemaphoreGive(sdMutex);

      // write index entry (also protected)
      struct avi_idx1_entry entry;
      entry.chunk_id = dc_id;
      entry.flags = 0x10;
      entry.offset = movi_size + 4;
      entry.size = chunk_size;
      if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
      indexFile.write((uint8_t*)&entry, sizeof(entry));
      if (sdMutex) xSemaphoreGive(sdMutex);

      movi_size += (8 + total_chunk_size);
      frameCount++;

      esp_camera_fb_return(fb);
    }

    /* ---------- AUDIO DRAIN (wrap-safe) ---------- */
    if (audioFile && audioBufferedBytes > 0) {

      // write up to N bytes per iteration to avoid starving video
      size_t want = min((size_t)256, (size_t)audioBufferedBytes);

      // write first contiguous chunk
      size_t firstChunk = min(want, (size_t)(AUDIO_RING_SIZE - audioReadPos));
      if (firstChunk > 0) {
        if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
        audioFile.write(&audioRingBuffer[audioReadPos], firstChunk);
        if (sdMutex) xSemaphoreGive(sdMutex);

        audioReadPos = (audioReadPos + firstChunk) % AUDIO_RING_SIZE;
        audioBufferedBytes -= firstChunk;
        audioBytes += firstChunk;
        want -= firstChunk;
      }

      // write wrapped remainder
      if (want > 0) {
        if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
        audioFile.write(&audioRingBuffer[audioReadPos], want);
        if (sdMutex) xSemaphoreGive(sdMutex);

        audioReadPos = (audioReadPos + want) % AUDIO_RING_SIZE;
        audioBufferedBytes -= want;
        audioBytes += want;
      }
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  /* ================== FINALIZE ================== */
  digitalWrite(LED_PIN, LOW);
  audioRecording = false;

  /* ---- flush remaining audio (wrap-safe) ---- */
  while (audioBufferedBytes > 0 && audioFile) {
    size_t want = min((size_t)256, (size_t)audioBufferedBytes);
    size_t firstChunk = min(want, (size_t)(AUDIO_RING_SIZE - audioReadPos));
    if (firstChunk > 0) {
      if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
      audioFile.write(&audioRingBuffer[audioReadPos], firstChunk);
      if (sdMutex) xSemaphoreGive(sdMutex);

      audioReadPos = (audioReadPos + firstChunk) % AUDIO_RING_SIZE;
      audioBufferedBytes -= firstChunk;
      audioBytes += firstChunk;
      want -= firstChunk;
    }
    if (want > 0) {
      if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
      audioFile.write(&audioRingBuffer[audioReadPos], want);
      if (sdMutex) xSemaphoreGive(sdMutex);

      audioReadPos = (audioReadPos + want) % AUDIO_RING_SIZE;
      audioBufferedBytes -= want;
      audioBytes += want;
    }
  }

  if (audioFile) {
    if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
    writeWavHeader(audioFile, audioBytes);
    audioFile.close();
    if (sdMutex) xSemaphoreGive(sdMutex);
    Serial.println("🎧 Audio finalized");
  }

  if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
  indexFile.close();
  if (sdMutex) xSemaphoreGive(sdMutex);

  /* ================== WRITE idx1 ================== */
  if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
  indexFile = SD_MMC.open("/Videos/idx.tmp", FILE_READ);
  if (indexFile) {
    uint32_t idx1_id = 0x31786469;
    uint32_t idx1_size = frameCount * sizeof(struct avi_idx1_entry);
    videoFile.write((uint8_t*)&idx1_id, 4);
    videoFile.write((uint8_t*)&idx1_size, 4);

    uint8_t buf[512];
    while (indexFile.available()) {
      int r = indexFile.read(buf, sizeof(buf));
      if (r > 0) videoFile.write(buf, r);
    }
    indexFile.close();
    SD_MMC.remove("/Videos/idx.tmp");
  } else {
    Serial.println("Warning: idx.tmp not found during finalization");
  }
  if (sdMutex) xSemaphoreGive(sdMutex);

  /* ================== AVI HEADER FIXUP ================== */
  if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
  uint32_t total_size = videoFile.size();
  videoFile.seek(0, SeekSet);

  uint32_t riff_size = total_size - 8;
  videoFile.write((uint8_t*)"RIFF", 4);
  videoFile.write((uint8_t*)&riff_size, 4);
  videoFile.write((uint8_t*)"AVI ", 4);

  // LIST hdrl
  videoFile.write((uint8_t*)"LIST", 4);
  uint32_t hdrl_size = 172; // Approximate
  videoFile.write((uint8_t*)&hdrl_size, 4);
  videoFile.write((uint8_t*)"hdrl", 4);
  // avih
  videoFile.write((uint8_t*)"avih", 4);
  uint32_t avih_size = 56;
  videoFile.write((uint8_t*)&avih_size, 4);

  uint32_t us_per_frame = 1000000 / fps;
  videoFile.write((uint8_t*)&us_per_frame, 4);
  uint32_t max_bytes = 0; videoFile.write((uint8_t*)&max_bytes, 4);
  uint32_t paddingLocal = 0; videoFile.write((uint8_t*)&paddingLocal, 4);
  uint32_t flags = 0x10; videoFile.write((uint8_t*)&flags, 4); // HASINDEX
  videoFile.write((uint8_t*)&frameCount, 4);
  uint32_t initial_frames = 0; videoFile.write((uint8_t*)&initial_frames, 4);
  uint32_t streams = 1; videoFile.write((uint8_t*)&streams, 4);
  uint32_t buf_size = 102400; videoFile.write((uint8_t*)&buf_size, 4);
  uint32_t width = 640; videoFile.write((uint8_t*)&width, 4);
  uint32_t height = 480; videoFile.write((uint8_t*)&height, 4);
  uint32_t reserved[4] = {0,0,0,0}; videoFile.write((uint8_t*)reserved, 16);

  // LIST strl + strh + strf (same as before)
  videoFile.write((uint8_t*)"LIST", 4);
  uint32_t strl_size = 108;
  videoFile.write((uint8_t*)&strl_size, 4);
  videoFile.write((uint8_t*)"strl", 4);
  // strh
  videoFile.write((uint8_t*)"strh", 4);
  uint32_t strh_size = 56;
  videoFile.write((uint8_t*)&strh_size, 4);
  videoFile.write((uint8_t*)"vids", 4);
  videoFile.write((uint8_t*)"MJPG", 4);
  videoFile.write((uint8_t*)&paddingLocal, 4); // flags
  videoFile.write((uint8_t*)&paddingLocal, 4); // priority
  videoFile.write((uint8_t*)&paddingLocal, 4); // initial frames
  uint32_t scale = 1; videoFile.write((uint8_t*)&scale, 4);
  videoFile.write((uint8_t*)&fps, 4);
  uint32_t start = 0; videoFile.write((uint8_t*)&start, 4);
  videoFile.write((uint8_t*)&frameCount, 4);
  videoFile.write((uint8_t*)&buf_size, 4);
  int32_t quality = -1; videoFile.write((uint8_t*)&quality, 4);
  videoFile.write((uint8_t*)&paddingLocal, 4); // sample size
  uint32_t rc_frame[2] = {0, 0}; videoFile.write((uint8_t*)rc_frame, 8);

  // strf
  videoFile.write((uint8_t*)"strf", 4);
  uint32_t strf_size = 40;
  videoFile.write((uint8_t*)&strf_size, 4);
  uint32_t bi_size = 40; videoFile.write((uint8_t*)&bi_size, 4);
  videoFile.write((uint8_t*)&width, 4);
  videoFile.write((uint8_t*)&height, 4);
  uint16_t planes = 1; videoFile.write((uint8_t*)&planes, 2);
  uint16_t bit_count = 24; videoFile.write((uint8_t*)&bit_count, 2);
  videoFile.write((uint8_t*)"MJPG", 4);
  uint32_t img_size = width * height * 3; videoFile.write((uint8_t*)&img_size, 4);
  videoFile.write((uint8_t*)&paddingLocal, 4); // xpels
  videoFile.write((uint8_t*)&paddingLocal, 4); // ypels
  videoFile.write((uint8_t*)&paddingLocal, 4); // colors used
  videoFile.write((uint8_t*)&paddingLocal, 4); // colors important

  // movi list header - write at the saved position (this mirrors your earlier logic)
  videoFile.seek(224, SeekSet); // Exact position depends on previous writes
  videoFile.write((uint8_t*)"LIST", 4);
  uint32_t movi_list_size = movi_size + 4;
  videoFile.write((uint8_t*)&movi_list_size, 4);
  videoFile.write((uint8_t*)"movi", 4);

  videoFile.flush();
  videoFile.close();
  if (sdMutex) xSemaphoreGive(sdMutex);

  /* ================== RENAME FILE ================== */
  if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
  if (SD_MMC.exists(tmpFileName)) {
    SD_MMC.rename(tmpFileName, finalFileName);
  }
  if (sdMutex) xSemaphoreGive(sdMutex);

  Serial.printf("✅ Video saved: %s | Frames: %d\n", finalFileName, frameCount);

  recordingDone = true;
  recordTaskHandle = NULL;
  vTaskDelete(NULL);
}


void stopVideo() {
  if (!isRecording) return;
  isRecording = false;

  unsigned long waitStart = millis();
  const unsigned long WAIT_TIMEOUT = 7000;
  while (!recordingDone && (millis() - waitStart) < WAIT_TIMEOUT) {
    delay(10);
  }

  if (!recordingDone) {
    Serial.println("Warning: recording task didn't finish within timeout");
  } else {
    Serial.println("Recording finalized");
  }

  // Do not write WAV header here — recordVideoTask() finalizes and closes audioFile.
  recordingDone = false; // ready for next recording
}


void setup() {
  sdMutex = xSemaphoreCreateMutex();
  Serial.begin(115200);
  delay(500);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Init with high specs for better video if PSRAM found
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    Serial.println("Found PSRAM...");
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    Serial.println("PSRAM NOT Found.");
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  Serial.println("Camera Initialized..");

  WiFi.mode(WIFI_STA);
  WiFi.begin("ESP32_Wroom_AP", "12345678");

  Serial.print("Connecting to WROOM AP");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");

  wsClient.begin("192.168.4.1", 80, "/ws");
  wsClient.onEvent(onWsEvent);


  // SD card init in 1-bit mode (frees some GPIOs) when second parameter true
  if(!SD_MMC.begin("/sdcard", true)){
    Serial.println("SD Card Mount Failed");
    return;
  }

  Serial.println(" SD initialized");

  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }

  if(!SD_MMC.exists("/Images")){
    SD_MMC.mkdir("/Images");
  }

  if(!SD_MMC.exists("/Videos")){
    SD_MMC.mkdir("/Videos");
  }
  
  loadCounters();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);   // LED OFF initially


  // Setup AsyncWebServer routes
  asyncServer.on("/images", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (isRecording) { respondRecording(req); return; }
    if (sdBusy) {
      req->send(503, "text/plain", "SD Busy");
      return;
    }
    if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);

    String payload = "[";
    File root = SD_MMC.open("/Images");
    bool first = true;

    if (root && root.isDirectory()) {
      File f = root.openNextFile();
      while (f) {
        String name = f.name();   // /Images/photo1.jpg

        if (!f.isDirectory() && (name.endsWith(".jpg") || name.endsWith(".jpeg") || name.endsWith(".png"))) {
          int slash = name.lastIndexOf('/');
          String cleanName = name.substring(slash + 1);

          if (!first) payload += ",";
          payload += "\"" + cleanName + "\"";
          first = false;
        }

        f.close();
        f = root.openNextFile();
      }
      root.close();
    }

    if (sdMutex) xSemaphoreGive(sdMutex);

    payload += "]";
    req->send(200, "application/json", payload);
  });

  asyncServer.on("/image", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (isRecording) { respondRecording(req); return; }
    if (sdBusy) {
      req->send(503, "text/plain", "SD Busy");
      return;
    }

    if (!req->hasParam("name")) {
      req->send(400, "text/plain", "Missing image name");
      return;
    }

    String fileName = req->getParam("name")->value();
    String path = "/Images/" + fileName;

    if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
    bool exists = SD_MMC.exists(path);
    if (sdMutex) xSemaphoreGive(sdMutex);

    if (!exists) {
      req->send(404, "text/plain", "Image not found");
      return;
    }

    AsyncWebServerResponse *response = req->beginResponse(SD_MMC, path, "image/jpeg");

    // Improve app performance (caching)
    response->addHeader("Cache-Control", "public, max-age=86400");

    // Uncomment if you want forced download instead of preview
    // response->addHeader("Content-Disposition", "attachment; filename=" + fileName);

    req->send(response);
  });


  
  // /videos -> JSON array of available video folders
  asyncServer.on("/videos", HTTP_GET, [](AsyncWebServerRequest *req){
    if (isRecording) { respondRecording(req); return; }

    if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);

    String payload = "[";
    File root = SD_MMC.open("/Videos");
    File f = root.openNextFile();
    bool first = true;

    while (f) {
      if (f.isDirectory()) {
        String folderName = String(f.name());   // e.g. /Videos/Video_BOOT_xxx
        int lastSlash = folderName.lastIndexOf('/');
        if (lastSlash >= 0)
          folderName = folderName.substring(lastSlash + 1);

        if (!first) payload += ",";
        payload += "\"" + folderName + "\"";
        first = false;
      }
      f.close();
      f = root.openNextFile();
    }

    root.close();
    if (sdMutex) xSemaphoreGive(sdMutex);

    payload += "]";
    req->send(200, "application/json", payload);
  });


  // /download?folder=Video_BOOT_xxx&type=video
  // /download?folder=Video_BOOT_xxx&type=audio
  asyncServer.on("/download", HTTP_GET, [](AsyncWebServerRequest *req){

    if (isRecording) {
      req->send(503, "application/json",
        "{\"status\":\"recording_in_progress\"}");
      return;
    }

    if (!req->hasParam("folder") || !req->hasParam("type")) {
      req->send(400, "text/plain", "Missing params");
      return;
    }

    String folder = req->getParam("folder")->value();
    String type   = req->getParam("type")->value();

    while (folder.startsWith("/"))
      folder = folder.substring(1);

    String fileName;

    if (type == "video")
      fileName = "video.avi";
    else if (type == "audio")
      fileName = "audio.wav";
    else {
      req->send(400, "text/plain", "Invalid type");
      return;
    }

    String fullPath = "/Videos/" + folder + "/" + fileName;

    if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY);
    bool exists = SD_MMC.exists(fullPath.c_str());
    if (sdMutex) xSemaphoreGive(sdMutex);

    if (!exists) {
      req->send(404, "text/plain", "File not found");
      return;
    }

    sdBusy = true;

    req->onDisconnect([]() {
      sdBusy = false;
      Serial.println("Transfer finished, SD free");
    });

    String mime = (type == "video") ? "video/x-msvideo" : "audio/wav";

    AsyncWebServerResponse *response =
      req->beginResponse(SD_MMC, fullPath, mime);

    response->addHeader("Cache-Control", "no-cache");
    req->send(response);

    Serial.println("Sent: " + fullPath);
  });


  // Not found
  asyncServer.onNotFound([](AsyncWebServerRequest *req){
    req->send(404, "text/plain", "Not found");
  });

  asyncServer.begin();
  Serial.println("HTTP server started (AP mode)");

}

void loop() {
  wsClient.loop();  // ✅ must be called every loop

  // Auto reconnect WiFi
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin("ESP32_Wroom_AP", "12345678");
    delay(500);
  }

  if (doSendImg) { doSendImg = false; captureAndSendImage(); }
  if (doCapture) { doCapture = false; captureImage(); }
}
