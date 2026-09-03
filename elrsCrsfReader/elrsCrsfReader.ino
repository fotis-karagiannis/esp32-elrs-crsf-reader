#include <Arduino.h>
#include <HardwareSerial.h>

/*
 * ESP32 ELRS CRSF Reader
 *
 * Reads CRSF RC channel data from an ExpressLRS receiver through UART2 and
 * exposes the live channel values and reading workflow to the companion
 * browser interface over the ESP32 USB serial connection.
 *
 * CRSF receiver interface:
 *   RX: GPIO 21
 *   TX: GPIO 22
 *   Baud rate: 420000
 *
 * Browser serial interface:
 *   USB Serial: 115200 baud
 *
 * The reading sequence identifies the primary control axes, arm/disarm
 * channel, and any additional active channels before reporting the collected
 * channel ranges to the browser interface.
 */

// CRSF UART interface
#define CRSF_RX_PIN 21
#define CRSF_TX_PIN 22

HardwareSerial CRSFSerial(2);

// Reading data structures
// Stores the detected channel index and measured range for one control.
struct AxisData {
  int ch = -1;
  int min = 9999;
  int mid = 0;
  int max = 0;
  const char* func;
};

// Latest decoded CRSF values for channels 1-16.
uint16_t channels[16] = {0};

// Per-step minimum/maximum tracking used while detecting channel movement.
uint16_t stepMin[16];
uint16_t stepMax[16];
bool     stepInit[16];

void resetStepTracking() {
  for (int i = 0; i < 16; i++) {
    stepInit[i] = false;
    stepMin[i]  = 0;
    stepMax[i]  = 0;
  }
}


// CRSF frame parsing
// Extracts one packed 11-bit RC channel value from the CRSF payload.
uint16_t getChannel(const uint8_t *payload, int payloadLen, int ch) {
  if (ch < 0 || ch >= 16 || payloadLen <= 0) return 0;

  int bitIndex  = ch * 11;
  int byteIndex = bitIndex / 8;
  int bitOffset = bitIndex % 8;

  // An 11-bit channel can span two or three payload bytes. Channel 16 starts
  // at bit 5 of byte 20 and ends in byte 21, so only two bytes remain there.
  int bytesNeeded = (bitOffset + 11 + 7) / 8;
  if (byteIndex + bytesNeeded > payloadLen) return 0;

  uint32_t value = 0;
  for (int i = 0; i < bytesNeeded; i++) {
    value |= (uint32_t)payload[byteIndex + i] << (8 * i);
  }

  return (value >> bitOffset) & 0x07FF;
}

// Reads the CRSF byte stream and updates the channel array when a packed
// RC-channel frame is received.
bool readCRSF() {
  static uint8_t buf[64];
  static uint8_t idx = 0;

  while (CRSFSerial.available()) {
    uint8_t b = CRSFSerial.read();

    if (idx == 0 && b != 0xC8) continue;

    buf[idx++] = b;

    if (idx >= 3) {
      uint8_t len       = buf[1];
      uint8_t frameSize = len + 2;

      if (idx == frameSize) {
        uint8_t type = buf[2];

        if (type != 0x16) { idx = 0; continue; }
        if (len < 24)     { idx = 0; continue; }

        const uint8_t *payload = &buf[3];
        int payloadLen = len - 2;

        for (int i = 0; i < 16; i++) {
          channels[i] = getChannel(payload, payloadLen, i);
        }

        idx = 0;
        return true;
      }
    }

    if (idx >= sizeof(buf)) idx = 0;
  }

  return false;
}

// State machine
enum CalState {
  WAIT_READY,
  THR_UP, THR_MID, THR_DOWN,
  ROLL_LEFT, ROLL_CENTER, ROLL_RIGHT,
  PITCH_UP, PITCH_CENTER, PITCH_DOWN,
  YAW_LEFT, YAW_CENTER, YAW_RIGHT,
  ARM_STEP,
  EXTRA_CH,
  FINISHED
};

CalState state = WAIT_READY;

// Primary control reading data.
AxisData throttleData, rollData, pitchData, yawData;
AxisData armData;
int armCh = -1;

// Remaining channels are checked sequentially after the primary controls.
int extraChannels[16];
int extraCount = 0;
int extraIndex = 0;

// Results are stored until reading is complete, then sorted by channel.
AxisData results[32];
int resultCount = 0;

// Clears all values collected by a previous reading session while preserving
// the current live CRSF channel values.
void resetReadingSession() {
  throttleData = AxisData();
  rollData     = AxisData();
  pitchData    = AxisData();
  yawData      = AxisData();
  armData      = AxisData();

  armCh = -1;
  extraCount = 0;
  extraIndex = 0;
  resultCount = 0;

  resetStepTracking();
}

// Browser serial protocol helpers
/*
 * ESP32 -> browser messages:
 *   STEP:<instruction>   Current reading instruction
 *   DATA:<channel list>  Live CRSF channel values
 *   RESULT:<values>      One completed reading result
 *   DONE                 Reading sequence complete
 *
 * Browser -> ESP32 messages:
 *   ACK:READY            Begin the reading sequence
 *   ACK:NEXT             Confirm the current step and continue
 */

void sendStep(const char *msg) {
  Serial.print("STEP:");
  Serial.println(msg);
}

void sendData() {
  Serial.print("DATA:");

  for (int i = 0; i < 16; i++) {
    Serial.print("CH");
    Serial.print(i+1);
    Serial.print("=");
    Serial.print(channels[i]);
    if (i < 15) Serial.print(",");
  }

  Serial.println();
}

void storeResult(const char* func, AxisData &d) {
  AxisData r = d;
  r.func = func;
  results[resultCount++] = r;
}

void outputSortedResults() {
  for (int i = 0; i < resultCount - 1; i++) {
    for (int j = i + 1; j < resultCount; j++) {
      if (results[j].ch < results[i].ch) {
        AxisData tmp = results[i];
        results[i] = results[j];
        results[j] = tmp;
      }
    }
  }

  for (int i = 0; i < resultCount; i++) {
    Serial.print("RESULT:");
    Serial.print("CH=");
    Serial.print(results[i].ch + 1);
    Serial.print(",FUNC=");
    Serial.print(results[i].func);
    Serial.print(",MIN=");
    Serial.print(results[i].min);
    Serial.print(",MID=");
    Serial.print(results[i].mid);
    Serial.print(",MAX=");
    Serial.println(results[i].max);
  }
}

// Reading tracking
// Tracks the observed minimum and maximum of every channel during a primary
// axis or arm-switch detection step.
void trackAllChannelsForStep() {
  for (int i = 0; i < 16; i++) {
    int v = channels[i];

    if (!stepInit[i]) {
      stepInit[i] = true;
      stepMin[i]  = v;
      stepMax[i]  = v;
    } else {
      if (v < stepMin[i]) stepMin[i] = v;
      if (v > stepMax[i]) stepMax[i] = v;
    }
  }
}

// Tracks only the currently selected additional channel.
void trackCurrentExtraChannel() {
  if (extraIndex >= extraCount) return;

  int ch = extraChannels[extraIndex];
  int v  = channels[ch];

  if (!stepInit[ch]) {
    stepInit[ch] = true;
    stepMin[ch]  = v;
    stepMax[ch]  = v;
  } else {
    if (v < stepMin[ch]) stepMin[ch] = v;
    if (v > stepMax[ch]) stepMax[ch] = v;
  }
}

// Updates the active reading trackers whenever CRSF data is processed.
void updateReadingFromChannels() {
  switch (state) {
    case THR_UP:
    case ROLL_LEFT:
    case PITCH_UP:
    case YAW_LEFT:
      trackAllChannelsForStep();
      break;

    case ARM_STEP:
      trackAllChannelsForStep();
      break;

    case EXTRA_CH:
      trackCurrentExtraChannel();
      break;

    default:
      break;
  }
}

// Browser command handling
String uiBuffer = "";

void handleUICommand(String cmd) {
  const int AXIS_THRESHOLD   = 200;
  const int BUTTON_THRESHOLD = 20;

  if (cmd.startsWith("ACK:READY")) {
    resetReadingSession();
    state = THR_UP;
    sendStep("THROTTLE_UP");
  }

  if (cmd.startsWith("ACK:NEXT")) {
    switch (state) {

      // Detect the throttle channel from the largest observed movement.
      case THR_UP: {
        int bestCh = -1, bestDelta = 0;

        for (int i = 0; i < 16; i++) {
          int delta = stepMax[i] - stepMin[i];

          if (delta > AXIS_THRESHOLD && delta > bestDelta) {
            bestDelta = delta;
            bestCh = i;
          }
        }

        if (bestCh == -1) {
          resetStepTracking();
          sendStep("THROTTLE_UP");
          break;
        }

        throttleData.ch  = bestCh;
        throttleData.max = stepMax[bestCh];
        throttleData.min = stepMin[bestCh];

        state = THR_MID;
        sendStep("THROTTLE_CENTER");
        break;
      }

      case THR_MID:
        throttleData.mid = channels[throttleData.ch];
        state = THR_DOWN;
        sendStep("THROTTLE_DOWN");
        break;

      case THR_DOWN:
        if (channels[throttleData.ch] < throttleData.min)
          throttleData.min = channels[throttleData.ch];

        state = ROLL_LEFT;
        resetStepTracking();
        sendStep("ROLL_LEFT");
        break;

      // Detect and read the roll channel.
      case ROLL_LEFT: {
        int bestCh = -1, bestDelta = 0;

        for (int i = 0; i < 16; i++) {
          int delta = stepMax[i] - stepMin[i];

          if (delta > AXIS_THRESHOLD && delta > bestDelta) {
            bestDelta = delta;
            bestCh = i;
          }
        }

        if (bestCh == -1) {
          resetStepTracking();
          sendStep("ROLL_LEFT");
          break;
        }

        rollData.ch  = bestCh;
        rollData.min = stepMin[bestCh];
        rollData.max = stepMax[bestCh];

        state = ROLL_CENTER;
        sendStep("ROLL_CENTER");
        break;
      }

      case ROLL_CENTER:
        rollData.mid = channels[rollData.ch];
        state = ROLL_RIGHT;
        sendStep("ROLL_RIGHT");
        break;

      case ROLL_RIGHT:
        if (channels[rollData.ch] > rollData.max)
          rollData.max = channels[rollData.ch];

        state = PITCH_UP;
        resetStepTracking();
        sendStep("PITCH_UP");
        break;

      // Detect and read the pitch channel.
      case PITCH_UP: {
        int bestCh = -1, bestDelta = 0;

        for (int i = 0; i < 16; i++) {
          int delta = stepMax[i] - stepMin[i];

          if (delta > AXIS_THRESHOLD && delta > bestDelta) {
            bestDelta = delta;
            bestCh = i;
          }
        }

        if (bestCh == -1) {
          resetStepTracking();
          sendStep("PITCH_UP");
          break;
        }

        pitchData.ch  = bestCh;
        pitchData.max = stepMax[bestCh];
        pitchData.min = stepMin[bestCh];

        state = PITCH_CENTER;
        sendStep("PITCH_CENTER");
        break;
      }

      case PITCH_CENTER:
        pitchData.mid = channels[pitchData.ch];
        state = PITCH_DOWN;
        sendStep("PITCH_DOWN");
        break;

      case PITCH_DOWN:
        if (channels[pitchData.ch] < pitchData.min)
          pitchData.min = channels[pitchData.ch];

        state = YAW_LEFT;
        resetStepTracking();
        sendStep("YAW_LEFT");
        break;

      // Detect and read the yaw channel.
      case YAW_LEFT: {
        int bestCh = -1, bestDelta = 0;

        for (int i = 0; i < 16; i++) {
          int delta = stepMax[i] - stepMin[i];

          if (delta > AXIS_THRESHOLD && delta > bestDelta) {
            bestDelta = delta;
            bestCh = i;
          }
        }

        if (bestCh == -1) {
          resetStepTracking();
          sendStep("YAW_LEFT");
          break;
        }

        yawData.ch  = bestCh;
        yawData.min = stepMin[bestCh];
        yawData.max = stepMax[bestCh];

        state = YAW_CENTER;
        sendStep("YAW_CENTER");
        break;
      }

      case YAW_CENTER:
        yawData.mid = channels[yawData.ch];
        state = YAW_RIGHT;
        sendStep("YAW_RIGHT");
        break;

      case YAW_RIGHT:
        if (channels[yawData.ch] > yawData.max)
          yawData.max = channels[yawData.ch];

        state = ARM_STEP;
        resetStepTracking();
        sendStep("FLIP_ARM_SWITCH_REPEATEDLY");
        break;

      // Identify the arm/disarm channel while excluding the detected axes.
      case ARM_STEP: {
        int axisCh[4] = {
          throttleData.ch,
          rollData.ch,
          pitchData.ch,
          yawData.ch
        };

        auto isAxis = [&](int ch) {
          for (int i = 0; i < 4; i++)
            if (axisCh[i] == ch) return true;
          return false;
        };

        armCh = -1;
        int bestDelta = 0;

        for (int i = 0; i < 16; i++) {
          if (isAxis(i)) continue;

          int delta = stepMax[i] - stepMin[i];

          if (delta > AXIS_THRESHOLD && delta > bestDelta) {
            bestDelta = delta;
            armCh = i;
          }
        }

        if (armCh != -1) {
          armData.ch  = armCh;
          armData.min = stepMin[armCh];
          armData.max = stepMax[armCh];
          armData.mid = 0;
        }

        storeResult("THROTTLE", throttleData);
        storeResult("ROLL",     rollData);
        storeResult("PITCH",    pitchData);
        storeResult("YAW",      yawData);

        if (armCh != -1)
          storeResult("ARM/DISARM", armData);

        extraCount = 0;
        extraIndex = 0;

        for (int i = 0; i < 16; i++) {
          if (isAxis(i)) continue;
          if (i == armCh) continue;
          extraChannels[extraCount++] = i;
        }

        if (extraCount > 0) {
          state = EXTRA_CH;
          resetStepTracking();

          String msg = "MOVE_CHANNEL_";
          msg += (extraChannels[0] + 1);
          sendStep(msg.c_str());
        } else {
          state = FINISHED;
          outputSortedResults();
          Serial.println("DONE");
        }

        break;
      }

      // Check remaining channels individually and store active ones.
      case EXTRA_CH: {
        int ch = extraChannels[extraIndex];
        int delta = stepMax[ch] - stepMin[ch];

        if (delta > BUTTON_THRESHOLD) {
          AxisData btn;
          btn.ch  = ch;
          btn.min = stepMin[ch];
          btn.max = stepMax[ch];
          btn.mid = 0;
          storeResult("BUTTON", btn);
        }

        extraIndex++;

        if (extraIndex < extraCount) {
          resetStepTracking();

          String msg = "MOVE_CHANNEL_";
          msg += (extraChannels[extraIndex] + 1);
          sendStep(msg.c_str());
        } else {
          state = FINISHED;
          outputSortedResults();
          Serial.println("DONE");
        }

        break;
      }

      default:
        break;
    }
  }
}

// Arduino entry points
void setup() {
  Serial.begin(115200);
  CRSFSerial.begin(420000, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
  resetStepTracking();
}

void loop() {
  if (readCRSF()) {
    updateReadingFromChannels();
  }

  static unsigned long lastSend = 0;

  if (millis() - lastSend >= 50) {
    sendData();
    lastSend = millis();
  }

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      handleUICommand(uiBuffer);
      uiBuffer = "";
    } else {
      uiBuffer += c;
    }
  }
}
