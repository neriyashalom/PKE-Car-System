#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ==========================================
// 1. הגדרות מצבי עבודה
// ==========================================
bool USE_PHYSICAL_INPUTS = true;

// ==========================================
// 2. הגדרת פינים (GPIO מדויק)
// ==========================================
const int PIN_BRAKE_SIG    = 10;
const int PIN_HB_SIG       = 11;
const int PIN_DOOR_SIG     = 12;
const int PIN_IGN_SIG      = 13;
const int PIN_BTN_SIG      = 14;

const int PIN_LOCK         = 4;
const int PIN_UNLOCK       = 9;
const int PIN_START        = 1;
const int PIN_ACC          = 2;
const int PIN_IGN          = 42;
const int PIN_MIRROR_CLOSE = 41;
const int PIN_MIRROR_OPEN  = 40;
const int PIN_L_BLINK      = 48;
const int PIN_R_BLINK      = 47;
const int PIN_BTN_LED      = 21;

// ==========================================
// 3. ערכי זמן קבועים
// ==========================================
const unsigned long DEBOUNCE_DELAY            = 50;
const unsigned long LOCK_PULSE                = 500;
const unsigned long UNLOCK_PULSE              = 500;
const unsigned long LOCK_BLINK_DURATION       = 3000;
const int           UNLOCK_BLINK_COUNT        = 3;
const unsigned long UNLOCK_BLINK_ON_TIME      = 333;
const unsigned long UNLOCK_BLINK_OFF_TIME     = 333;
const int           LOCK_FAIL_BLINK_COUNT     = 5;
const unsigned long LOCK_FAIL_BLINK_ON_TIME   = 333;
const unsigned long LOCK_FAIL_BLINK_OFF_TIME  = 333;
const unsigned long LED_BLINK_ON_TIME         = 333;
const unsigned long LED_BLINK_OFF_TIME        = 333;
const int           LED_ERROR_BLINK_COUNT     = 4;
const unsigned long LED_ERROR_BLINK_ON_TIME   = 333;
const unsigned long LED_ERROR_BLINK_OFF_TIME  = 333;
const unsigned long MIRROR_OPEN_PULSE         = 3000;
const unsigned long MIRROR_CLOSE_PULSE        = 3000;
const unsigned long PRE_START_DELAY           = 1000;    // תמיד 1s לפני START — FR-17, FR-18
const unsigned long START_PULSE               = 1500;
const unsigned long EMERGENCY_STOP_PRESS      = 5000;
const unsigned long ACC_IGN_AUTO_OFF_TIMEOUT  = 300000; // 5 דקות — FR-35
const unsigned long IGN_OFF_FILTER_TIME       = 200;    // סינון יציאה ממפתח מקורי — FR-15

// ==========================================
// 4. הגדרות BLE (כולל סינון FR-02/03)
// ==========================================
// TODO: >>> חובה להזין את ה-MAC האמיתי של תג ה-Feasycom כאן לפני צריבה לחומרה <
const String TARGET_MAC = "dc:0d:30:2b:84:8f";

const int RSSI_NEAR_THRESHOLD = -70;
const int RSSI_FAR_THRESHOLD  = -85;
const unsigned long BLE_TIMEOUT     = 2000;
const unsigned long RSSI_FILTER_TIME = 1000; // סינון תנודות RSSI — FR-02, FR-03

unsigned long lastBleSeenTime = 0;
int currentRssi = -100;
bool isKeyPresent = false;

unsigned long rssiThresholdStartTime = 0;
bool isRssiInNearZone = false;
bool isRssiInFarZone  = false;

// ==========================================
// 5. הגדרת מצבי המערכת
// ==========================================
// IDLE: מצב מעבר רגעי בלבד — אינו מצב שהייה. מנתב ל-LOCKED/UNLOCKED מיד.
enum SystemState {
  STATE_IDLE,
  STATE_LOCKED,
  STATE_UNLOCKED,
  STATE_ACC_ON,
  STATE_IGN_ON,
  STATE_CRANKING,
  STATE_ENGINE_RUNNING,
  STATE_ORIGINAL_KEY_ACTIVE
};

SystemState currentState = STATE_IDLE;

// ==========================================
// 6. משתנים גלובליים
// ==========================================
bool brakePressed       = false;
bool hbPulled           = false;
bool isDoorOpen         = false;
bool originalIgnActive  = false;
bool btnPressed         = false;
bool lastBtnPressed     = false;

bool ledSuppressedAfterShutdown = false;
bool prevBrakeState = false;
bool prevKeyPresent = false;

bool espControlsIgnition = false;
bool lockFailNotified    = false;

unsigned long btnPressStartTime       = 0;
unsigned long stateEnterTime          = 0;
unsigned long keyLostStartTime        = 0; // FR-35
unsigned long originalIgnOffStartTime = 0; // FR-15
unsigned long ignReleasedByEspTime = 0;
const unsigned long IGN_TAKEOVER_GUARD_TIME = 700;

struct InputState {
  bool currentState;
  bool lastReading;
  unsigned long lastDebounceTime;
};
InputState brakeState = {false, false, 0};
InputState hbState    = {false, false, 0};
InputState doorState  = {false, false, 0};
InputState ignState   = {false, false, 0};
InputState btnState   = {false, false, 0};

// -- ניהול פלטים ברקע --
unsigned long lockActionStartTime = 0;
bool isLocking   = false;
bool isUnlocking = false;

bool isLockFailSequence = false;
int  lockFailStep       = 0;
unsigned long lockFailTimer = 0;

unsigned long mirrorActionStartTime = 0;
bool isMirrorOpening = false;
bool isMirrorClosing = false;

unsigned long blinkerLastToggleTime = 0;
int  blinkerToggleCount   = 0;
int  blinkerTargetToggles = 0;
bool isBlinking       = false;
bool isSolidBlink     = false;
unsigned long currentBlinkOnTime  = 0;
unsigned long currentBlinkOffTime = 0;

unsigned long btnLedLastToggle   = 0;
int  btnLedToggleCount           = 0;
int  btnLedTargetToggles         = 0;
bool isBtnLedBlinking            = false;
bool btnLedBaseState             = false;
unsigned long currentLedOnTime   = 0;
unsigned long currentLedOffTime  = 0;

// ==========================================
// 7. פונקציות עזר
// ==========================================

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      String seenMac = advertisedDevice.getAddress().toString().c_str();

      if (seenMac == TARGET_MAC) {
        currentRssi = advertisedDevice.getRSSI();
        lastBleSeenTime = millis();

        /*Serial.print("TARGET FOUND | MAC: ");
        Serial.print(seenMac);
        Serial.print(" | RSSI: ");
        Serial.println(currentRssi);*/
      }
    }
};

// סינון RSSI עם היסטרזיס — FR-02, FR-03
void updateBLELogic() {
  if (!USE_PHYSICAL_INPUTS) return;

  unsigned long now = millis();
  if (now - lastBleSeenTime > BLE_TIMEOUT) currentRssi = -100;

  if (currentRssi >= RSSI_NEAR_THRESHOLD) {
    if (!isRssiInNearZone) { isRssiInNearZone = true; rssiThresholdStartTime = now; }
    isRssiInFarZone = false;
  } else if (currentRssi <= RSSI_FAR_THRESHOLD) {
    if (!isRssiInFarZone) { isRssiInFarZone = true; rssiThresholdStartTime = now; }
    isRssiInNearZone = false;
  } else {
    // שטח מת — ללא שינוי מצב
    isRssiInNearZone = false;
    isRssiInFarZone  = false;
  }

  if (!isKeyPresent && isRssiInNearZone && (now - rssiThresholdStartTime >= RSSI_FILTER_TIME)) {
    isKeyPresent = true;
    Serial.print(">>> BLE KEY DETECTED (NEAR), RSSI = ");
    Serial.print(currentRssi);
    Serial.println(" <<<");
  } else if (isKeyPresent && isRssiInFarZone && (now - rssiThresholdStartTime >= RSSI_FILTER_TIME)) {
    isKeyPresent = false;
    Serial.print(">>> BLE KEY LOST (FAR), RSSI = ");
    Serial.print(currentRssi);
    Serial.println(" <<<");
  }
}

void setRelay(int pin, bool state) {
  if (digitalRead(pin) != state) {
    digitalWrite(pin, state ? HIGH : LOW);
    if (pin == PIN_IGN) {
      espControlsIgnition = state;
      if (!state) {
        ignReleasedByEspTime = millis();
      }
    }
  }
}

void changeState(SystemState newState) {
  currentState   = newState;
  stateEnterTime = millis();
  Serial.print(">>> NEW STATE: ");
  Serial.println(newState);
}

void debounceInput(int pin, InputState &input, bool &outputFlag, bool activeHigh) {
  bool reading        = digitalRead(pin);
  bool logicalReading = activeHigh ? reading : !reading;
  if (logicalReading != input.lastReading) input.lastDebounceTime = millis();
  if ((millis() - input.lastDebounceTime) > DEBOUNCE_DELAY) {
    if (logicalReading != input.currentState) {
      input.currentState = logicalReading;
      outputFlag         = input.currentState;
    }
  }
  input.lastReading = logicalReading;
}

void readAllInputs() {
  debounceInput(PIN_BRAKE_SIG, brakeState, brakePressed,      true);
  debounceInput(PIN_HB_SIG,    hbState,    hbPulled,          false); // טריגר שלילי
  debounceInput(PIN_DOOR_SIG,  doorState,  isDoorOpen,        true);
  debounceInput(PIN_IGN_SIG,   ignState,   originalIgnActive, true);
  debounceInput(PIN_BTN_SIG,   btnState,   btnPressed,        true);
}

// סימולציה דרך Serial לבדיקות ללא חומרה
void readSerialSimulation() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch(c) {
      case 'k': isKeyPresent      = !isKeyPresent;      break;
      case 'p': btnPressed        = !btnPressed;        break;
      case 'b': brakePressed      = !brakePressed;      break;
      case 'h': hbPulled          = !hbPulled;          break;
      case 'd': isDoorOpen        = !isDoorOpen;        break;
      case 'i': originalIgnActive = !originalIgnActive; break;
    }
  }
}

// --- טריגרים לפלטים ---

// FR-04, FR-06, FR-08: פתיחה + מראות + 3 הבהובים
void triggerUnlockSequence() {
  isUnlocking = true; lockActionStartTime = millis();
  isMirrorOpening = true; mirrorActionStartTime = millis();
  isBlinking = true; isSolidBlink = false;
  blinkerToggleCount    = 0;
  blinkerTargetToggles  = UNLOCK_BLINK_COUNT * 2;
  currentBlinkOnTime    = UNLOCK_BLINK_ON_TIME;
  currentBlinkOffTime   = UNLOCK_BLINK_OFF_TIME;
  blinkerLastToggleTime = millis();
  setRelay(PIN_L_BLINK, true); setRelay(PIN_R_BLINK, true);
}

// FR-12 (תרחיש 16): UNLOCK בלבד ליישור פיזי — ללא מראות וללא הבהובים
// מיועדת לשימוש לאחר ביטול רצף פעיל (cancelLockFailSequence) בלבד
void triggerUnlockOnlySequence() {
  isUnlocking = true;
  lockActionStartTime = millis();
}

// FR-05, FR-07, FR-09: נעילה + מראות + איתות רציף 3 שניות
void triggerLockSequence() {
  isLocking = true; lockActionStartTime = millis();
  isMirrorClosing = true; mirrorActionStartTime = millis();
  isBlinking = true; isSolidBlink = true;
  blinkerLastToggleTime = millis();
  setRelay(PIN_L_BLINK, true); setRelay(PIN_R_BLINK, true);
}

void triggerLockFailureBlinks() {
  isBlinking = true; isSolidBlink = false;
  blinkerToggleCount    = 0;
  blinkerTargetToggles  = LOCK_FAIL_BLINK_COUNT * 2;
  currentBlinkOnTime    = LOCK_FAIL_BLINK_ON_TIME;
  currentBlinkOffTime   = LOCK_FAIL_BLINK_OFF_TIME;
  blinkerLastToggleTime = millis();
  setRelay(PIN_L_BLINK, true); setRelay(PIN_R_BLINK, true);
}

// FR-10, FR-11: כישלון נעילה — LOCK פולס ואז UNLOCK פולס + 5 הבהובים
// המראות לא זזות בכלל — FR-10: בכישלון נעילה מראות לא מתקפלות
// יש לוודא שלפני הכניסה לרצף זה לא נשאר רצף PKE סותר פעיל - SR-12, SW-04
void triggerLockFailureSequence() {
  isLockFailSequence = true;
  lockFailStep       = 1;
  lockFailTimer      = millis();
  setRelay(PIN_LOCK, true);
  // ללא isMirrorClosing — FR-10: בכישלון נעילה מראות לא זזות
}

// עוצר את כל רצפי ה-PKE הפעילים ומכבה פלטים — SR-12, SW-04
// יש לקרוא לפני כל triggerLock/Unlock/LockFailure כדי למנוע חפיפות
void cancelAllPKESequences() {
  isLocking          = false;
  isUnlocking        = false;
  isLockFailSequence = false;
  lockFailStep       = 0;
  lockFailNotified   = false;
  setRelay(PIN_LOCK,   false);
  setRelay(PIN_UNLOCK, false);

  isMirrorOpening = false;
  isMirrorClosing = false;
  setRelay(PIN_MIRROR_OPEN,  false);
  setRelay(PIN_MIRROR_CLOSE, false);

  isBlinking = false;
  setRelay(PIN_L_BLINK, false);
  setRelay(PIN_R_BLINK, false);
}

// ביטול רצף כישלון נעילה — עוצר את כל ה-PKE
void cancelLockFailSequence() {
  cancelAllPKESequences();
}

void triggerLedFeedback() {
  isBtnLedBlinking    = true;
  btnLedToggleCount   = 0;
  btnLedTargetToggles = 2; // הבהוב קצר אחד (ON+OFF) — FR-26
  currentLedOnTime    = LED_BLINK_ON_TIME;
  currentLedOffTime   = LED_BLINK_OFF_TIME;
  btnLedLastToggle    = millis();
  setRelay(PIN_BTN_LED, !btnLedBaseState);
}

void triggerLedError() {
  isBtnLedBlinking    = true;
  btnLedToggleCount   = 0;
  btnLedTargetToggles = LED_ERROR_BLINK_COUNT * 2; // 4 הבהובי שגיאה — FR-27, FR-28
  currentLedOnTime    = LED_ERROR_BLINK_ON_TIME;
  currentLedOffTime   = LED_ERROR_BLINK_OFF_TIME;
  btnLedLastToggle    = millis();
  setRelay(PIN_BTN_LED, true);
}

// --- מנהל פלטים Non-blocking — SW-04 ---
void handleOutputs() {
  unsigned long currentMillis = millis();

  // רצף כישלון נעילה: LOCK pulse → UNLOCK pulse → 5 הבהובים — FR-10, FR-11
  if (isLockFailSequence) {
    if (lockFailStep == 1 && (currentMillis - lockFailTimer >= LOCK_PULSE)) {
      setRelay(PIN_LOCK, false);
      lockFailStep = 2;
      setRelay(PIN_UNLOCK, true);
      lockFailTimer = currentMillis;
    }
    else if (lockFailStep == 2 && (currentMillis - lockFailTimer >= UNLOCK_PULSE)) {
      setRelay(PIN_UNLOCK, false);
      triggerLockFailureBlinks();
      isLockFailSequence = false;
      lockFailStep       = 0;
    }
  }

  if (isUnlocking && !isLockFailSequence) {
    setRelay(PIN_UNLOCK, true);
    if (currentMillis - lockActionStartTime >= UNLOCK_PULSE) {
      setRelay(PIN_UNLOCK, false);
      isUnlocking = false;
    }
  }
  if (isLocking && !isLockFailSequence) {
    setRelay(PIN_LOCK, true);
    if (currentMillis - lockActionStartTime >= LOCK_PULSE) {
      setRelay(PIN_LOCK, false);
      isLocking = false;
    }
  }

  if (isMirrorOpening) {
    setRelay(PIN_MIRROR_OPEN, true);
    if (currentMillis - mirrorActionStartTime >= MIRROR_OPEN_PULSE) {
      setRelay(PIN_MIRROR_OPEN, false);
      isMirrorOpening = false;
    }
  }
  if (isMirrorClosing) {
    setRelay(PIN_MIRROR_CLOSE, true);
    if (currentMillis - mirrorActionStartTime >= MIRROR_CLOSE_PULSE) {
      setRelay(PIN_MIRROR_CLOSE, false);
      isMirrorClosing = false;
    }
  }

  if (isBlinking) {
    if (isSolidBlink) {
      // נעילה: וינקרים דולקים רציף 3 שניות — FR-07
      if (currentMillis - blinkerLastToggleTime >= LOCK_BLINK_DURATION) {
        setRelay(PIN_L_BLINK, false); setRelay(PIN_R_BLINK, false);
        isBlinking = false;
      }
    } else {
      // פתיחה / שגיאת נעילה: הבהובים ספורים — FR-06, FR-11
      bool isOn = digitalRead(PIN_L_BLINK);
      unsigned long waitTime = isOn ? currentBlinkOnTime : currentBlinkOffTime;
      if (currentMillis - blinkerLastToggleTime >= waitTime) {
        blinkerToggleCount++;
        if (blinkerToggleCount >= blinkerTargetToggles) {
          setRelay(PIN_L_BLINK, false); setRelay(PIN_R_BLINK, false);
          isBlinking = false;
        } else {
          setRelay(PIN_L_BLINK, !isOn); setRelay(PIN_R_BLINK, !isOn);
          blinkerLastToggleTime = currentMillis;
        }
      }
    }
  }

  // חישוב מצב בסיס LED — FR-22, FR-23, FR-27, FR-28, FR-29
  btnLedBaseState = false;

  if (currentState == STATE_ORIGINAL_KEY_ACTIVE) {
    // FR-14: ממתין לסיום הבהוב פעיל אם קיים, ואז כבוי
    btnLedBaseState = false;
  } else if (ledSuppressedAfterShutdown) {
    // FR-23: לאחר כיבוי LED לא יידלק אוטומטית
    btnLedBaseState = false;
  } else if (currentState == STATE_ACC_ON   || currentState == STATE_IGN_ON ||
             currentState == STATE_CRANKING || currentState == STATE_ENGINE_RUNNING) {
    btnLedBaseState = true;  // FR-22: מערכת פעילה
  } else if (isKeyPresent && brakePressed) {
    btnLedBaseState = true;  // FR-22: מפתח + ברקס
  } else if (!isKeyPresent && brakePressed) {
    btnLedBaseState = true;  // FR-27: אין מפתח + ברקס לחוץ = LED דולק (ללא תלות בבלם יד)
  }

  if (isBtnLedBlinking) {
    bool isOn = digitalRead(PIN_BTN_LED);
    unsigned long waitTime = isOn ? currentLedOnTime : currentLedOffTime;
    if (currentMillis - btnLedLastToggle >= waitTime) {
      btnLedToggleCount++;
      if (btnLedToggleCount >= btnLedTargetToggles) {
        isBtnLedBlinking = false;
        setRelay(PIN_BTN_LED, btnLedBaseState); // חזרה למצב בסיס
      } else {
        setRelay(PIN_BTN_LED, !isOn);
        btnLedLastToggle = currentMillis;
      }
    }
  } else {
    setRelay(PIN_BTN_LED, btnLedBaseState);
  }
}

// ==========================================
// 8. Setup
// ==========================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_BRAKE_SIG, INPUT);
  pinMode(PIN_HB_SIG,    INPUT);
  pinMode(PIN_DOOR_SIG,  INPUT);
  pinMode(PIN_IGN_SIG,   INPUT);
  pinMode(PIN_BTN_SIG,   INPUT);

  int outputs[] = {PIN_LOCK, PIN_UNLOCK, PIN_START, PIN_ACC, PIN_IGN,
                   PIN_MIRROR_CLOSE, PIN_MIRROR_OPEN, PIN_L_BLINK, PIN_R_BLINK, PIN_BTN_LED};
  for (int i = 0; i < 10; i++) {
    pinMode(outputs[i], OUTPUT);
    digitalWrite(outputs[i], LOW); // SR-08: כל הפלטים בטוחים באתחול
  }

  if (USE_PHYSICAL_INPUTS) {
    BLEDevice::init("");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
    pBLEScan->setActiveScan(false);
    pBLEScan->start(0, nullptr, false);
    Serial.println("BLE Scanner Started.");
  }

  changeState(STATE_IDLE);
}

// ==========================================
// 9. Loop — State Machine ראשי — SW-01
// ==========================================
void loop() {
  lastBtnPressed = btnPressed;

  if (USE_PHYSICAL_INPUTS) {
    readAllInputs();    // SW-02
    updateBLELogic();
  } else {
    readSerialSimulation();
  }

  bool btnJustPressed = (btnPressed && !lastBtnPressed);

  // FR-35: טיימר מבוסס רגע היעלמות המפתח (לא רגע כניסה למצב)
  if (!isKeyPresent && keyLostStartTime == 0) {
    keyLostStartTime = millis();
  } else if (isKeyPresent) {
    keyLostStartTime = 0;
  }

  // FR-23: שחרור חסימת LED בעקבות פעילות חדשה
  if (brakePressed && !prevBrakeState) ledSuppressedAfterShutdown = false;
  if (isKeyPresent  && !prevKeyPresent) ledSuppressedAfterShutdown = false;
  if (btnJustPressed)                   ledSuppressedAfterShutdown = false;

  prevBrakeState = brakePressed;
  prevKeyPresent = isKeyPresent;

  if (isKeyPresent || !isDoorOpen) lockFailNotified = false;

  // FR-13: זיהוי שליטת מפתח מקורי
  // SR-12, FR-13: מבטל את כל רצפי ה-PKE הפעילים לפני הכניסה
  if (originalIgnActive &&
    !espControlsIgnition &&
    currentState != STATE_ORIGINAL_KEY_ACTIVE &&
    (millis() - ignReleasedByEspTime > IGN_TAKEOVER_GUARD_TIME)) {
  cancelAllPKESequences();
  changeState(STATE_ORIGINAL_KEY_ACTIVE);
  handleOutputs();
  return;
  }

  switch (currentState) {

    // STATE_IDLE: מצב מעבר רגעי — FR-37, SR-08
    // מאפס פלטים ומנתב מיד ל-LOCKED או UNLOCKED, אינו מצב שהייה
    case STATE_IDLE:
      setRelay(PIN_ACC,   false);
      setRelay(PIN_IGN,   false);
      setRelay(PIN_START, false);
      if (isKeyPresent) {
        triggerUnlockSequence();
        changeState(STATE_UNLOCKED);
      } else {
        changeState(STATE_LOCKED);
      }
      break;

    // STATE_LOCKED: אין מפתח, רכב נעול
    case STATE_LOCKED:
      setRelay(PIN_ACC,   false);
      setRelay(PIN_IGN,   false);
      setRelay(PIN_START, false);
      if (isKeyPresent) {
        // SR-12: מבטל פולס LOCK שעלול עדיין לרוץ לפני פתיחה
        cancelAllPKESequences();
        triggerUnlockSequence(); // FR-04, FR-06, FR-08
        changeState(STATE_UNLOCKED);
        break;
      }
      if (btnJustPressed) triggerLedError(); // FR-27, FR-28
      break;

    // STATE_UNLOCKED: רכב פתוח וכבוי, ממתין לפעולת משתמש או להחלטת PKE
    // (אפשר להיות כאן גם ללא מפתח — אחרי אובדן מפתח, אחרי כישלון נעילה, אחרי כיבוי)
    case STATE_UNLOCKED:
      setRelay(PIN_ACC,   false);
      setRelay(PIN_IGN,   false);
      setRelay(PIN_START, false);

      // ניהול רצף כישלון נעילה פעיל — SW-05, FR-12 (תרחיש 16)
      if (isLockFailSequence) {
        if (!isDoorOpen && !isKeyPresent) {
          // דלת נסגרה באמצע — מבטל הכל ונועל רגיל — FR-12
          cancelLockFailSequence();
          triggerLockSequence(); // FR-05, FR-07, FR-09
          changeState(STATE_LOCKED);
          break;
        }
        if (isKeyPresent) {
          // מפתח חזר באמצע — UNLOCK בלבד ליישור פיזי — FR-12
          // לא triggerUnlockSequence כי המראות לא נסגרו
          cancelLockFailSequence();
          triggerUnlockOnlySequence();
          changeState(STATE_UNLOCKED);
          break;
        }
        break;
      }

      // SR-11: PKE פעיל רק כשמצב ההנעה כבוי (LOCKED/UNLOCKED)
      if (!isKeyPresent) {
        if (isDoorOpen) {
          // FR-10, FR-11, SR-04: כישלון נעילה — LOCK+UNLOCK בלבד, מראות לא זזות
          // SR-12, SW-04: מבטל רצף PKE קודם שאולי עדיין רץ לפני כניסה לרצף חדש
          if (!lockFailNotified) {
            cancelAllPKESequences();
            triggerLockFailureSequence();
            lockFailNotified = true;
          }
        } else {
          // SR-12: מבטל פולס UNLOCK שעלול עדיין לרוץ לפני נעילה
          cancelAllPKESequences();
          triggerLockSequence(); // FR-05, FR-07, FR-09
          changeState(STATE_LOCKED);
          lockFailNotified = false;
          break;
        }
      }

      if (btnJustPressed) {
        if (isKeyPresent) {
          if (brakePressed && hbPulled) {
            triggerLedFeedback();
            changeState(STATE_CRANKING); // FR-16, FR-17, FR-18: תמיד 1s
          } else {
            triggerLedFeedback();
            changeState(STATE_ACC_ON); // FR-25
          }
        } else {
          triggerLedError(); // FR-27, FR-28
        }
      }
      break;

    // STATE_ACC_ON: ACC פעיל, IGN כבוי
    case STATE_ACC_ON:
      setRelay(PIN_ACC, true);

      // FR-35: כיבוי אוטומטי אחרי 5 דקות ללא מפתח
      if (!isKeyPresent && keyLostStartTime > 0 &&
          (millis() - keyLostStartTime > ACC_IGN_AUTO_OFF_TIMEOUT)) {
        setRelay(PIN_ACC, false);
        ledSuppressedAfterShutdown = true;
        changeState(STATE_UNLOCKED);
        break;
      }

      if (btnJustPressed) {
        if (isKeyPresent) {
          if (brakePressed && hbPulled) {
            triggerLedFeedback();
            changeState(STATE_CRANKING); // FR-16, FR-17, FR-18
          } else {
            triggerLedFeedback();
            changeState(STATE_IGN_ON); // FR-25
          }
        } else {
          // FR-32: כיבוי מ-ACC ללא ברקס מותר כשהמנוע לא פועל (גם אחרי אובדן מפתח)
          triggerLedFeedback();
          setRelay(PIN_ACC, false);
          ledSuppressedAfterShutdown = true;
          changeState(STATE_UNLOCKED);
        }
      }
      break;

    // STATE_IGN_ON: ACC + IGN פעילים, מנוע לא פועל
    case STATE_IGN_ON:
      setRelay(PIN_ACC, true);
      setRelay(PIN_IGN, true);

      // FR-35: כיבוי אוטומטי אחרי 5 דקות ללא מפתח
      if (!isKeyPresent && keyLostStartTime > 0 &&
          (millis() - keyLostStartTime > ACC_IGN_AUTO_OFF_TIMEOUT)) {
        setRelay(PIN_ACC, false);
        setRelay(PIN_IGN, false);
        ledSuppressedAfterShutdown = true;
        changeState(STATE_UNLOCKED);
        break;
      }

      if (btnJustPressed) {
        if (isKeyPresent && brakePressed && hbPulled) {
          triggerLedFeedback();
          changeState(STATE_CRANKING); // FR-16, FR-17, FR-18: תמיד 1s גם מ-IGN_ON
        } else {
          // FR-32: כיבוי — תנאי התנעה לא מתקיימים במלואם (כולל אובדן מפתח)
          triggerLedFeedback();
          setRelay(PIN_ACC, false);
          setRelay(PIN_IGN, false);
          ledSuppressedAfterShutdown = true;
          changeState(STATE_UNLOCKED);
        }
      }
      break;

    // STATE_CRANKING: רצף התנעה — FR-17, FR-18, FR-19
    case STATE_CRANKING:
      {
        // FR-17, FR-18: תמיד 1s לפני START — לא משנה מאיפה הגענו
        if (millis() - stateEnterTime < PRE_START_DELAY) {
          // שלב 1: ACC + IGN, ממתינים לייצוב ולחץ דלק
          setRelay(PIN_ACC,   true);
          setRelay(PIN_IGN,   true);
          setRelay(PIN_START, false);
        } else if (millis() - stateEnterTime < (PRE_START_DELAY + START_PULSE)) {
          // שלב 2: ניתוק ACC, הפעלת סטרטר — FR-19
          setRelay(PIN_ACC,   false);
          setRelay(PIN_IGN,   true);
          setRelay(PIN_START, true);
        } else {
          // שלב 3: סיום התנעה, החזרת ACC
          setRelay(PIN_START, false);
          setRelay(PIN_ACC,   true);
          changeState(STATE_ENGINE_RUNNING);
          break;
        }
        // תרחיש 15: שחרור ברקס באמצע התנעה = עצירת סטרטר מיידית
        if (!brakePressed) {
          setRelay(PIN_START, false);
          changeState(STATE_IGN_ON);
          break;
        }
      }
      break;

    // STATE_ENGINE_RUNNING: מנוע פועל — FR-21, SR-06
    case STATE_ENGINE_RUNNING:
      setRelay(PIN_ACC,   true);
      setRelay(PIN_IGN,   true);
      setRelay(PIN_START, false);

      // SR-06: אובדן מפתח לא מכבה מנוע

      if (btnPressed) {
        if (btnPressStartTime == 0) btnPressStartTime = millis();
        // FR-33: כיבוי חירום — לחיצה רצופה 5 שניות, ללא תנאי ברקס/HB
        if (millis() - btnPressStartTime >= EMERGENCY_STOP_PRESS) {
          setRelay(PIN_ACC,   false);
          setRelay(PIN_IGN,   false);
          ledSuppressedAfterShutdown = true;
          btnPressStartTime = 0;
          changeState(STATE_UNLOCKED);
          break;
        }
      } else {
        if (btnPressStartTime > 0) {
          unsigned long pressDuration = millis() - btnPressStartTime;
          // FR-31: כיבוי רגיל — דורש ברקס + בלם יד
          // FR-34, SR-05: תנאי כיבוי לא מתקיימים = התעלמות מוחלטת
          if (pressDuration < EMERGENCY_STOP_PRESS && brakePressed && hbPulled) {
            setRelay(PIN_ACC,   false);
            setRelay(PIN_IGN,   false);
            ledSuppressedAfterShutdown = true;
            changeState(STATE_UNLOCKED);
            btnPressStartTime = 0;
            break;
          }
          btnPressStartTime = 0;
        }
      }
      break;

    // STATE_ORIGINAL_KEY_ACTIVE: מפתח מקורי שולט — FR-13, FR-14, FR-15
    case STATE_ORIGINAL_KEY_ACTIVE:
      // SR-07: הבקר לא מנסה להשתלט על ACC/IGN/START
      setRelay(PIN_ACC,   false);
      setRelay(PIN_IGN,   false);
      setRelay(PIN_START, false);
      // FR-14: BTN_LED — ממתין לסיום הבהוב פעיל אם קיים ואז כבוי. מטופל ב-handleOutputs.

      // FR-15: יציאה מסוננת — IGN_SIG חייב להישאר LOW למשך IGN_OFF_FILTER_TIME
      if (!originalIgnActive) {
        if (originalIgnOffStartTime == 0) {
          originalIgnOffStartTime = millis();
        } else if (millis() - originalIgnOffStartTime >= IGN_OFF_FILTER_TIME) {
          originalIgnOffStartTime = 0;
          // FR-15: ביציאה מ-ORIGINAL_KEY_ACTIVE לא מבצעים פעולה פיזית מיידית.
          // אם BLE קרוב - עוברים ל-UNLOCKED בלבד.
          // אם BLE לא קרוב - עוברים ל-LOCKED בלבד.
          if (isKeyPresent) {
            changeState(STATE_UNLOCKED);
          } else {
            changeState(STATE_LOCKED);
          }
          break;
        }
      } else {
        originalIgnOffStartTime = 0; // איפוס אם הסוויץ' חזר ל-ON
      }
      break;
  }

  handleOutputs(); // SW-03, SW-04
}