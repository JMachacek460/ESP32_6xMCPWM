#include <Arduino.h>
#include <EEPROM.h>
#include <ctype.h>

// --- NOVÉ HEADERY PRO ESP32/FREERTOS A RMT ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/rmt.h"
#include "driver/gpio.h"

//-------------------------------------
// DEFINICE VERZE A KONSTANT
//-------------------------------------
#define VERSION "V1.3_ESP32_8CH"
const size_t VERSION_SIZE = sizeof(VERSION);
#define NUM_CHANNELS 8
const int RMT_CLOCK_DIVIDER = 80;  // 80MHz / 80 = 1MHz -> 1 tick = 1 us

// If you uncomment the following line, you will activate debug mode
//#define DEBUG_MODE

//------------------------------------------------------------------------------
// --- DEFINICE PINU (PRO 8 KANÁLŮ A JEDEN CHYBOVÝ) ---
//------------------------------------------------------------------------------
// 8x RMT VSTUPY
const gpio_num_t INPUT_PINS[NUM_CHANNELS] = {
  GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4,
  GPIO_NUM_5, GPIO_NUM_7, GPIO_NUM_8, GPIO_NUM_9
};
// 8x DIGITÁLNÍ VÝSTUPY (LOW/HIGH flip)
const gpio_num_t OUTPUT_PINS[NUM_CHANNELS] = {
  GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_15, GPIO_NUM_16,
  GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_20
};
// 8x RMT KANÁLY
const rmt_channel_t RMT_RX_CHANNELS[NUM_CHANNELS] = {
  RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3,
  RMT_CHANNEL_4, RMT_CHANNEL_5, RMT_CHANNEL_6, RMT_CHANNEL_7
};
// 1x CHYBOVÝ VÝSTUP (společný)
const gpio_num_t ERROR_PIN = GPIO_NUM_21;

//------------------------------------------------------------------------------
// --- TEXTY (Bez PROGMEM) ---
//------------------------------------------------------------------------------
const char TEXT_T[] = "-t [number1] [number2] to set duty cycle limits in % for flipping (1-99) pin12.";
const char TEXT_I[] = "-i [number] 0/1 - no/yes invert output.";
const char TEXT_P[] = "-p [number1] [number2] to set limits for correct period in us (100-65000).";
const char TEXT_S[] = "-s [number1] [number2] to set limits for correct duty cycle in permille (1-499).";
const char TEXT_E[] = "-e [number] to set the number of consecutive errors before the error pin (0-255) pin6 flips.";
const char TEXT_ME[] = "-me [number] 0bXXXXXXXX; 1 - setting which inputs should be evaluated";
const char TEXT_TE[] = "-te [number] minimum error signaling duration (10-65000) ms.";
const char TEXT_BPS[] = "-b [number] serial buat rate 96 -> 9600, 1152 -> 115200.";
const char TEXT_L[] = "-l [number] 0/1 - no/yes lists current values of frequency and duty cycle.";
const char TEXT_DS[] = "-ds [char] decimal separator.";
const char TEXT_CS[] = "-cs [char] columns separator.";
const char TEXT_H[] = "-h for help.\r\n";
const char TEXT_HIDN[] = "*IDN? returns IDN";
const char TEXT_RST[] = "*RST sets all parameters to factory settings.";
const char TEXT_FETC[] = ":FETCh? returns the duty cycle values of the PWM signal in per mille.";
const char TEXT_PWID[] = ":MEASure:PWIDth? returns the length value of the HIGH signal.";
const char TEXT_PER[] = ":MEASure:PERiod? returns the signal period value.";
const char TEXT_EXAMPLE[] = "Example of a measurement command: :MEAS:PWID? :MEAS:PER?   answer e.g.: 0,006312; 0,020076";
const char TEXT_IDN[] = "ESP32-S3 RMT 8-Channel PWM Detector.";

const size_t MAX_COMMAND_LENGTH = 60;

const size_t EEPROM_ADRESA = 0;

//------------------------------------------------------------------------------
// ---DEFINICE STRUKTUR
//------------------------------------------------------------------------------
struct MojeSerialCommand {
  char inChar;
  const char* cmdPtr;
  byte state = 0;
  char buffer[MAX_COMMAND_LENGTH];
  size_t index = 0;
  bool isNewData = false;
};

// Sdílená struktura pro RMT Task a Loop (chráněná Mutexem)
struct MeasurementData {
  unsigned long period_us = 0;
  unsigned long pulseLength_us = 0;
  unsigned int dutyCyclePromile = 0;
};

// Všechny stavy chyb jsou pole, protože jsou specifické pro kanál
struct ControlState {
  const unsigned int signalTimeout_ms = 200;
  byte dutyCycleErrorCount[NUM_CHANNELS] = { 0 };
  byte periodErrorCount[NUM_CHANNELS] = { 0 };
  unsigned long startErrorTime[NUM_CHANNELS] = { 0 };
  const byte ERROR_OFF = 255;
};

struct MojeNastaveni {
  char verze[VERSION_SIZE];
  byte spodni_hranice;
  byte horni_hranice;
  byte input;
  unsigned char error_mask;  //  např 0b00000101  -> vyhonocuj jen kanal 3 a 1 ostatní ignoruj
  unsigned int min_perioda;
  unsigned int max_perioda;
  unsigned int min_strida;
  unsigned int max_strida;
  byte max_error;
  unsigned int t_error;
  unsigned int bps;
  byte listing;
  char decimalSeparator;
  char columnsSeparator;
};

const size_t EEPROM_SIZE = sizeof(struct MojeNastaveni);

//------------------------------------------------------------------------------
// --- GLOBALNI DEKLARACE PROMENNYCH (SDÍLENÉ)
//------------------------------------------------------------------------------
MeasurementData mereni[NUM_CHANNELS];  // POLE 8x Měření
MojeSerialCommand mSerial;
MojeNastaveni aktualniNastaveni;
ControlState control_local;  // Řídicí stav pro Task

// --- MUTEX PRO BEZPEČNÝ PŘÍSTUP K MERENÍ ---
SemaphoreHandle_t pwm_data_mutex;

//------------------------------------------------------------------------------
// --- POMOCNÉ FUNKCE ---
//------------------------------------------------------------------------------

void zobrazNastaveni() {
  Serial.print("Version: ");
  Serial.println(aktualniNastaveni.verze);
  Serial.print("Threshold LOW: ");
  Serial.print(aktualniNastaveni.spodni_hranice);
  Serial.println("%");
  Serial.print("Threshold HIGH: ");
  Serial.print(aktualniNastaveni.horni_hranice);
  Serial.println("%");
  Serial.print("Invert output: ");
  Serial.println(aktualniNastaveni.input);
  Serial.print("Error mask: ");  //Serial.println(aktualniNastaveni.error_mask);
  Serial.print("0b");
  for (int i = 7; i >= 0; i--) {
    bool bit = bitRead(aktualniNastaveni.error_mask, i);
    Serial.print(bit);
  }
  Serial.println();
  Serial.print("Min period: ");
  Serial.print(aktualniNastaveni.min_perioda);
  Serial.println(" us");
  Serial.print("Max period: ");
  Serial.print(aktualniNastaveni.max_perioda);
  Serial.println(" us");
  Serial.print("Min duty cycle: ");
  Serial.print(aktualniNastaveni.min_strida);
  Serial.println(" permille");
  Serial.print("Max duty cycle: ");
  Serial.print(aktualniNastaveni.max_strida);
  Serial.println(" permille");
  Serial.print("Max number of errors: ");
  Serial.println(aktualniNastaveni.max_error);
  Serial.print("Min error signaling duration: ");
  Serial.print(aktualniNastaveni.t_error);
  Serial.println(" ms");
  Serial.print("Serial line speed: ");
  Serial.print(aktualniNastaveni.bps);
  Serial.println("00 baud");
  Serial.print("Listing of measured frequency and duty cycle values: ");
  Serial.print(aktualniNastaveni.listing);
  Serial.println(" [0-No; 1-Yes]");
  Serial.print("Decimal separator: '");
  Serial.print(aktualniNastaveni.decimalSeparator);
  Serial.println("'");
  Serial.print("Columns separator: '");
  Serial.print(aktualniNastaveni.columnsSeparator);
  Serial.println("'");
}

void tovarniNastaveni() {
  strncpy(aktualniNastaveni.verze, VERSION, VERSION_SIZE);
  aktualniNastaveni.spodni_hranice = 40;
  aktualniNastaveni.horni_hranice = 60;
  aktualniNastaveni.input = 0;
  aktualniNastaveni.error_mask = 0b00000001;  //error se vyhodnocuje jen z kanálu 1
  aktualniNastaveni.min_perioda = 18000;
  aktualniNastaveni.max_perioda = 22000;
  aktualniNastaveni.min_strida = 280;
  aktualniNastaveni.max_strida = 330;
  aktualniNastaveni.max_error = 1;
  aktualniNastaveni.t_error = 800;
  aktualniNastaveni.bps = 1152;
  aktualniNastaveni.listing = 0;
  aktualniNastaveni.decimalSeparator = ',';
  aktualniNastaveni.columnsSeparator = ';';

  EEPROM.put(EEPROM_ADRESA, aktualniNastaveni);
  EEPROM.commit();
}

/**
 * Převede text na číslo. Podporuje:
 * - Binární: 0b101
 * - Hexadecimální: 0xFF
 * - Desítková: 123
 */
long parseValue(const char* str, char** nextPtr) {
  if (str == NULL) return -1;

  // Přeskočení úvodních mezer
  while (*str == ' ' || *str == '\t') str++;
  if (*str == '\0') return -1;
  // 1. Detekce BINÁRNÍHO tvaru (0b...)
  if (strncmp(str, "0b", 2) == 0) {
    return strtol(str + 2, nextPtr, 2);
  }

  // 2. Detekce HEX (0x), OCT (0) nebo DEC (ostatní)
  // Základ 0 zajistí automatické rozpoznání 0x a 0
  return strtol(str, nextPtr, 0);
}

/**
 * Univerzální funkce pro zpracování příkazů
 */
bool zpracujUniverzalniPrikaz(const char* command, unsigned int minValue, unsigned int maxValue,
                              byte* lowPtr, byte* highPtr,
                              unsigned int* uLowPtr = nullptr, unsigned int* uHighPtr = nullptr) {
  long nove_spodni = -1;
  long nove_horni = -1;
  char* nextPtr;

  // Najdeme začátek parametrů (první mezeru za názvem příkazu)
  char* ptr = (char*)strchr(command, ' ');
  if (ptr == NULL) return false;

  // Načtení prvního parametru
  char* currentPos = ptr;
  nove_spodni = parseValue(currentPos, &nextPtr);
  if (currentPos == nextPtr) return false;

  // Pokud se očekávají dva parametry (rozsah)
  if (uHighPtr != nullptr || highPtr != nullptr) {
    currentPos = nextPtr;
    nove_horni = parseValue(currentPos, &nextPtr);

    // Pokud druhý parametr chybí, vrátíme chybu
    if (currentPos == nextPtr) return false;
  } else {
    // Jeden parametr -> horní limit je stejný jako spodní
    nove_horni = nove_spodni;
  }

  // --- VALIDACE ---

  // Speciální případ pro BPS (příklad z vašeho kódu)
  if (strncmp(command, "-b", 2) == 0) {
    if (nove_spodni == 96 || nove_spodni == 1152) {
      if (uLowPtr) *uLowPtr = (unsigned int)nove_spodni;
      // Uložení do EEPROM
      EEPROM.put(EEPROM_ADRESA, aktualniNastaveni);
#if defined(ESP8266) || defined(ESP32)
      EEPROM.commit();
#endif

      Serial.println(F("\r\nLimits successfully updated:"));
      Serial.print(F("Low: "));
      Serial.println(nove_spodni);
      Serial.print(F("High: "));
      Serial.println(nove_horni);
      return true;
    }
    //goto error_range;
    //error_range:
    Serial.print(F("\r\nError: Values must be in range ("));
    Serial.print(minValue);
    Serial.print("-");
    Serial.print(maxValue);
    Serial.println(F("). Nothing changed."));
    return false;
  }

  // Univerzální kontrola rozsahu
  bool rangeValid = (nove_spodni >= (long)minValue && nove_spodni <= (long)maxValue && nove_horni >= (long)minValue && nove_horni <= (long)maxValue);
  // Kontrola logiky (spodní limit nesmí být větší než horní)
  if (uHighPtr != nullptr || highPtr != nullptr) {
    rangeValid = rangeValid && (nove_spodni <= nove_horni);
  }

  if (rangeValid) {
    if (uLowPtr) *uLowPtr = (unsigned int)nove_spodni;
    if (uHighPtr) *uHighPtr = (unsigned int)nove_horni;
    if (lowPtr) *lowPtr = (byte)nove_spodni;
    if (highPtr) *highPtr = (byte)nove_horni;

    // Uložení do EEPROM
    EEPROM.put(EEPROM_ADRESA, aktualniNastaveni);
#if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();
#endif

    Serial.println(F("\r\nLimits successfully updated:"));
    Serial.print(F("Low: "));
    Serial.println(nove_spodni);
    Serial.print(F("High: "));
    Serial.println(nove_horni);
    return true;

  } else {
    //error_range:
    Serial.print(F("\r\nError: Values must be in range ("));
    Serial.print(minValue);
    Serial.print("-");
    Serial.print(maxValue);
    Serial.println(F("). Nothing changed."));
    return false;
  }
}

void tiskniFloat(float cislo, int pocetDesetinychMist, char separator) {
  char buffer[13];
  dtostrf(cislo, 0, pocetDesetinychMist, buffer);
  char* tecka = strchr(buffer, '.');
  if (tecka != NULL) {
    *tecka = separator;
  }
  Serial.print(buffer);
}

//------------------------------------------------------------------------------
// --- RMT TASK (NEZÁVISLÝ DEKODÉR PRO 8 KANÁLŮ) ---
//------------------------------------------------------------------------------
void pwm_decode_and_control_task(void* pvParameters) {
  MojeNastaveni localSettings;

  while (1) {
    if (xSemaphoreTake(pwm_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      localSettings = aktualniNastaveni;
      xSemaphoreGive(pwm_data_mutex);
    }

    for (int i = 0; i < NUM_CHANNELS; i++) {
      // Získání handle pro Ring Buffer daného kanálu
      RingbufHandle_t rb = NULL;
      if (rmt_get_ringbuf_handle((rmt_channel_t)RMT_RX_CHANNELS[i], &rb) != ESP_OK || rb == NULL) {
        continue;
      }

      if ((localSettings.error_mask & (1 << i)) == 0) {
        // Kanál ignorován: vyprázdníme buffer a jdeme dál
        size_t skip_size;
        rmt_item32_t* item = (rmt_item32_t*)xRingbufferReceive(rb, &skip_size, 0);
        if (item) vRingbufferReturnItem(rb, (void*)item);
        continue;
      }

      size_t rx_size = 0;
      // POKUS O PŘEČTENÍ DAT (ekvivalent rmt_read_sample)
      // Čekáme max 0 ms (neblokující), aby task běžel svižně
      rmt_item32_t* rmt_rx_items = (rmt_item32_t*)xRingbufferReceive(rb, &rx_size, 0);

      // Pokud máme data a jsou to alespoň 2 pulzy (High + Low)
      if (rmt_rx_items != NULL && rx_size >= 2 * sizeof(rmt_item32_t)) {

        // --- VÝPOČET ---
        // rmt_item32_t obsahuje duration0 (trvání úrovně level0) a duration1 (level1)
        // Musíte si v inicializaci RMT zkontrolovat, zda level0 je HIGH nebo LOW
        uint32_t t_high_ticks = rmt_rx_items[0].duration0;
        uint32_t t_low_ticks = rmt_rx_items[0].duration1;

        unsigned long current_impuls_us = t_high_ticks;
        unsigned long current_period_us = t_high_ticks + t_low_ticks;

        if (current_period_us > 0) {
          unsigned int current_dutyCyclePromile = (current_impuls_us * 1000UL) / current_period_us;

          // --- ŘÍZENÍ OUTPUT PINU ---
          if (current_dutyCyclePromile > 10 * localSettings.horni_hranice) {
            gpio_set_level((gpio_num_t)OUTPUT_PINS[i], !localSettings.input);
          } else if (current_dutyCyclePromile < 10 * localSettings.spodni_hranice) {
            gpio_set_level((gpio_num_t)OUTPUT_PINS[i], localSettings.input);
          }

          // --- CHYBOVÉ VYHODNOCENÍ ---
          unsigned int normalizedDutyCycle = current_dutyCyclePromile;
          if (current_dutyCyclePromile > 500) normalizedDutyCycle = 1000 - current_dutyCyclePromile;

          bool periodError = (current_period_us < localSettings.min_perioda || current_period_us > localSettings.max_perioda);
          bool dutyCycleError = (normalizedDutyCycle < localSettings.min_strida || normalizedDutyCycle > localSettings.max_strida);

          if (periodError || dutyCycleError) {
            if (control_local.periodErrorCount[i] == 0) control_local.startErrorTime[i] = millis();
            if (periodError) control_local.periodErrorCount[i]++;
            if (dutyCycleError) control_local.dutyCycleErrorCount[i]++;
          } else {
            control_local.periodErrorCount[i] = 0;
            control_local.dutyCycleErrorCount[i] = 0;
          }

          // --- ZÁPIS DAT ---
          if (xSemaphoreTake(pwm_data_mutex, 0) == pdTRUE) {
            mereni[i].period_us = current_period_us;
            mereni[i].pulseLength_us = current_impuls_us;
            mereni[i].dutyCyclePromile = current_dutyCyclePromile;
            xSemaphoreGive(pwm_data_mutex);
          }
        }

        // DŮLEŽITÉ: Vrácení paměti Ring Bufferu
        vRingbufferReturnItem(rb, (void*)rmt_rx_items);

      } else {
        // --- WATCHDOG (Pokud nejsou data) ---
        // Pokud xRingbufferReceive vrátí NULL, v bufferu nic není (timeout signálu)
        gpio_set_level((gpio_num_t)OUTPUT_PINS[i], localSettings.input);

        if (xSemaphoreTake(pwm_data_mutex, 0) == pdTRUE) {
          mereni[i].period_us = 0;
          mereni[i].dutyCyclePromile = 0;
          xSemaphoreGive(pwm_data_mutex);
        }
      }
    }

    // ... zbytek vyhodnocení ERROR_PIN zůstává stejný ...
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

//------------------------------------------------------------------------------
// --- SETUP RMT DRIVER A FREERTOS TASKU ---
//------------------------------------------------------------------------------

void setup_rmt_decoder_8ch() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    // --- 1. RMT Konfigurace ---
    // Použijeme makro pro výchozí nastavení, které nastaví většinu věcí za nás
    rmt_config_t config = RMT_DEFAULT_CONFIG_RX((gpio_num_t)INPUT_PINS[i], (rmt_channel_t)RMT_RX_CHANNELS[i]);
    // NEJČASTĚJŠÍ CESTA: Invertování pinu pomocí GPIO matice
    // Pokud je signál fyzicky opačný, než potřebujete, nejjednodušší je toto:
    // gpio_matrix_in((gpio_num_t)INPUT_PINS[i], RMT_SIG_IN0_IDX + RMT_RX_CHANNELS[i], true);
    // Ten poslední parametr 'true' provede inverzi signálu na vstupu do RMT

    config.clk_div = RMT_CLOCK_DIVIDER;
    config.rx_config.filter_en = true;
    config.rx_config.filter_ticks_thresh = 10;

    // Nastavení času, po kterém se puls považuje za ukončený (IDLE)
    config.rx_config.idle_threshold = 12000;

    rmt_config(&config);

    // OPRAVA: Druhý parametr MUSÍ BÝT VĚTŠÍ NEŽ 0 (velikost Ring Bufferu v bajtech)
    // Nastavíme 1000 bajtů, aby se tam vešlo dostatek PWM vzorků
    rmt_driver_install((rmt_channel_t)RMT_RX_CHANNELS[i], 1000, 0);

    // OPRAVA: Místo rmt_read_sample spustíme RX přijímač
    rmt_rx_start((rmt_channel_t)RMT_RX_CHANNELS[i], true);

    // --- 2. GPIO Konfigurace ---
    esp_rom_gpio_pad_select_gpio((gpio_num_t)INPUT_PINS[i]);
    gpio_set_direction((gpio_num_t)INPUT_PINS[i], GPIO_MODE_INPUT);

    esp_rom_gpio_pad_select_gpio((gpio_num_t)OUTPUT_PINS[i]);
    gpio_set_direction((gpio_num_t)OUTPUT_PINS[i], GPIO_MODE_OUTPUT);
  }

  // Konfigurace ERROR pinu
  esp_rom_gpio_pad_select_gpio((gpio_num_t)ERROR_PIN);
  gpio_set_direction((gpio_num_t)ERROR_PIN, GPIO_MODE_OUTPUT);

  // Vytvoření Tasku (ponecháno beze změn, velikost stacku 8192 je v pořádku)
  xTaskCreate(pwm_decode_and_control_task, "PWM_Decoder_8ch", 8192, NULL, 5, NULL);
}

//------------------------------------------------------------------------------
// --- ARDUINO SETUP ---
//------------------------------------------------------------------------------

void setup() {

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(EEPROM_ADRESA, aktualniNastaveni);

  // Test verze
  if (strcmp(aktualniNastaveni.verze, VERSION) != 0 || aktualniNastaveni.bps == 0) {
    Serial.begin(9600);
    Serial.println("First run or version mismatch. Setting factory defaults.");
    tovarniNastaveni();
    delay(10);
  }

  Serial.begin(aktualniNastaveni.bps * 100UL);

  pwm_data_mutex = xSemaphoreCreateMutex();
  setup_rmt_decoder_8ch();

  Serial.println("\r\n\r\nThe ESP32-S3 is ready to evaluate 8 PWM channels.");
  zobrazNastaveni();
  Serial.println();
  Serial.println(TEXT_H);

  // Návod na zapojení (poprvé)
  Serial.println("--- PIN CONFIGURATION ---");
  Serial.print("RMT Inputs (PWM): ");
  for (int i = 0; i < NUM_CHANNELS; i++) { Serial.printf("GPIO%d, ", INPUT_PINS[i]); }
  Serial.println();
  Serial.print("Outputs (LOW/HIGH): ");
  for (int i = 0; i < NUM_CHANNELS; i++) { Serial.printf("GPIO%d, ", OUTPUT_PINS[i]); }
  Serial.println();
  Serial.printf("Error Pin: GPIO%d\n", ERROR_PIN);
}

//------------------------------------------------------------------------------
// --- ARDUINO LOOP (Pro komunikaci a kontrolní tisk) ---
//------------------------------------------------------------------------------

void loop() {

  // === PART 1: KONTROLNÍ TISK (LISTING) ===
  if (aktualniNastaveni.listing) {

    if (xSemaphoreTake(pwm_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

      for (int i = 0; i < NUM_CHANNELS; i++) {
        if (mereni[i].period_us > 0) {
          float frequencyHz = 1000000.0 / mereni[i].period_us;
          Serial.printf("CH%d: F: ", i + 1);
          tiskniFloat(frequencyHz, 2, aktualniNastaveni.decimalSeparator);
          Serial.print(" Hz | D: ");
          tiskniFloat(mereni[i].dutyCyclePromile / 10.0, 1, aktualniNastaveni.decimalSeparator);
          Serial.println(" %");
        } else {
          Serial.printf("CH%d: Signal Timeout/Not detected.\n", i + 1);
        }
      }
      xSemaphoreGive(pwm_data_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(aktualniNastaveni.t_error + 100));
  }

  // === PART 2: SÉRIOVÁ KOMUNIKACE A ZPRACOVÁNÍ PŘÍKAZŮ (ZACHOVÁNO) ===

  while (Serial.available() > 0) {
    mSerial.inChar = Serial.read();

    if (mSerial.inChar == '\n' || mSerial.inChar == '\r') {
      if (mSerial.index > 0) {
        if (mSerial.buffer[mSerial.index - 1] == ' ') mSerial.index--;
        mSerial.buffer[mSerial.index] = '\0';
        mSerial.isNewData = true;
        break;
      }
    } else if (mSerial.index < MAX_COMMAND_LENGTH - 1) {
      if (mSerial.index > 0 || mSerial.inChar != ' ') {
        if (mSerial.buffer[mSerial.index - 1] != ' ' || mSerial.inChar != ' ') {
          mSerial.buffer[mSerial.index++] = tolower(mSerial.inChar);
        }
      }
    }
  }

  if (mSerial.isNewData) {
    mSerial.cmdPtr = mSerial.buffer;
    mSerial.state = 0;

    while (mSerial.state != 255) {

      if (strncmp(mSerial.cmdPtr, "*idn?", 5) == 0) {
        Serial.print(TEXT_IDN);
        Serial.print(" Version: ");
        Serial.println(aktualniNastaveni.verze);
        mSerial.state = 255;
      } else if (strncmp(mSerial.cmdPtr, ":measure:pwidth?", 16) == 0 || strncmp(mSerial.cmdPtr, ":meas:pwid?", 11) == 0) {
        if (xSemaphoreTake(pwm_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          // Tisk všech 8 kanálů pro :MEASure:
          // POZOR muze cekat až 10ms než bude mit volna data
          for (int i = 0; i < NUM_CHANNELS; i++) {
            if (i > 0) Serial.print(aktualniNastaveni.columnsSeparator);
            Serial.print(" ");
            tiskniFloat(mereni[i].pulseLength_us / 1000000.0, 6, aktualniNastaveni.decimalSeparator);
          }
          xSemaphoreGive(pwm_data_mutex);
          Serial.println();
          mSerial.state = 255;
        } else {
          // Mutex NEZÍSKÁN ani po 10 ms: Chyba
          Serial.println("CHYBA: Tisk selhal (Mutex timeout). Data nejsou volná.");
          mSerial.state++;
        }

      } else if (strncmp(mSerial.cmdPtr, "-t ", 3) == 0) {
        if (!zpracujUniverzalniPrikaz(mSerial.cmdPtr, 1, 99, &aktualniNastaveni.spodni_hranice, &aktualniNastaveni.horni_hranice)) {
          Serial.println(TEXT_T);
        }
        mSerial.state = 255;
      }
      // Zbytek příkazů jako *RST, -p, -s, -e atd. zde
      else if (strncmp(mSerial.cmdPtr, "*rst", 4) == 0) {
        Serial.print(TEXT_RST);
        Serial.print(" Version: ");
        Serial.println(aktualniNastaveni.verze);
        tovarniNastaveni();
        zobrazNastaveni();
        mSerial.state = 255;
      } else {
        Serial.print("\r\nUnknown command or wrong format: ");
        Serial.println(mSerial.cmdPtr);
        Serial.println(TEXT_H);
        mSerial.state = 255;
      }
      if (mSerial.state > 20) mSerial.state = 255;
    }

    mSerial.index = 0;
    mSerial.isNewData = false;
  }
}