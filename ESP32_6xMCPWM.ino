#include "driver/mcpwm_cap.h"
#include <Preferences.h>
#include "USB.h"

#define NUM_CHANNELS 6
Preferences prefs;

#if !ARDUINO_USB_CDC_ON_BOOT
USBCDC USBSerial;
#endif

#define VERSION "V1.2"
//const size_t VERSION_SIZE = sizeof(VERSION);

// --- STRUKTURY ---
struct PwmData {
  uint32_t high_us;
  uint32_t low_us;
  uint32_t period_us;
};

struct ChannelState {
  uint32_t pos_temp;
  uint32_t neg_temp;
  unsigned long last_seen;
  uint8_t error_count;
  uint32_t last_period_us;
  uint32_t last_width_us;
};

// --- GLOBÁLNÍ PROMĚNNÉ ---
const int INPUT_PINS[NUM_CHANNELS] = { 4, 5, 6, 7, 15, 16 };
const int OUTPUT_PINS[NUM_CHANNELS] = { 10, 11, 12, 13, 1, 2 };
const int ERROR_PIN = 21;
// Seznam zbývajících volných pinů pro EMC stabilitu
const int UNUSED_PINS[] = { 17, 18, 8, 3, 46, 9, 14, 42, 41, 40, 39, 38, 37, 36, 35, 45, 48, 47 };
//const int UNUSED_PINS[] = {3, 8, 9, 14, 17, 18, 33, 34, 35, 36, 37, 38, 43, 44, 45, 46, 47, 48};

QueueHandle_t pwmQueues[NUM_CHANNELS];
ChannelState chStates[NUM_CHANNELS];

uint32_t s_min, s_max, e_low_min, e_low_max, e_high_min, e_high_max, e_maska, e_min_time;
bool invert_logic = false;
char columns_separator = ';';
char decimal_separator = ',';
uint32_t timeout_val = 500;

bool alarm_active = false;
unsigned long alarm_start = 0;

// --- ISR CALLBACK (Plnění fronty) ---
static bool on_capture_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data) {
  int ch = (int)user_data;
  uint32_t now = edata->cap_value;

  // Statické pole pro uložení času POSLEDNÍ JAKÉKOLIV hrany pro každý kanál
  static uint32_t last_any_edge[NUM_CHANNELS] = { 0 };

  // Výpočet rozdílu v tiktech (80 tiků = 1us)
  // Ošetření přetečení 32-bitového čítače
  uint32_t diff = (now >= last_any_edge[ch]) ? (now - last_any_edge[ch]) : (0xFFFFFFFF - last_any_edge[ch] + now);

  // FILTR: Pokud je změna stavu příliš rychlá (méně než 1us), ignoruj ji.
  // To odfiltruje kmity, které prošly přes optočlen nebo RC filtr.
  if (diff < 80) {
    return false;
  }
  last_any_edge[ch] = now;

  BaseType_t high_task_wakeup = pdFALSE;
  if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
    uint32_t period_ticks = now - chStates[ch].pos_temp;
    uint32_t low_ticks = now - chStates[ch].neg_temp;

    if (period_ticks > 8000000 || low_ticks > 8000000) {
      chStates[ch].pos_temp = now;
      return false;
    }

    chStates[ch].pos_temp = now;
    uint32_t period_us = period_ticks / 80;
    chStates[ch].last_period_us = period_us;

    uint32_t low_us = low_ticks / 80;

    // POKUD JE INVERT: Zajímá nás délka LOW impulsu jako "aktivní" šířka
    if (invert_logic) {
      chStates[ch].last_width_us = low_us;  // Ukládáme LOW us do proměnné pro MEAS
      if (low_us > s_max && low_us < s_max * 2) digitalWrite(OUTPUT_PINS[ch], HIGH);
      else if (low_us < s_min) digitalWrite(OUTPUT_PINS[ch], LOW);
    }

    uint32_t high_us = (period_us > low_us) ? (period_us - low_us) : 0;
    PwmData data = { .high_us = high_us, .low_us = low_us, .period_us = period_us };
    xQueueOverwriteFromISR(pwmQueues[ch], &data, &high_task_wakeup);

  } else {
    // Sestupná hrana (NEG)
    uint32_t high_ticks = now - chStates[ch].pos_temp;
    chStates[ch].neg_temp = now;

    if (high_ticks > 8000000) return false;
    uint32_t high_us = high_ticks / 80;

    // POKUD NENÍ INVERT: Zajímá nás délka HIGH impulsu jako "aktivní" šířka
    if (!invert_logic) {
      chStates[ch].last_width_us = high_us;  // Ukládáme HIGH us do proměnné pro MEAS
      if (high_us > s_max && high_us < s_max * 2) digitalWrite(OUTPUT_PINS[ch], HIGH);
      else if (high_us < s_min) digitalWrite(OUTPUT_PINS[ch], LOW);
    }
  }
  return high_task_wakeup == pdTRUE;
}

// --- PAMĚŤ A NASTAVENÍ ---
void setDefaults() {
  s_min = 8000;
  s_max = 12000;
  e_low_min = 5000;
  e_low_max = 8000;
  e_high_min = 12000;
  e_high_max = 15000;
  e_min_time = 800;   
  invert_logic = false;
  e_maska = 1;
  columns_separator = ';';
  decimal_separator = ',';
  timeout_val = 500;
  USBSerial.println(">> Hodnoty nastaveny na tovarni defaulty.");
}

void loadSettings() {
  prefs.begin("pwm-cfg", false);

  // 1. Kontrola verze
  String stored_ver = "none";
  if (prefs.isKey("ver")) {
    stored_ver = prefs.getString("ver");
  }

  if (stored_ver != VERSION) {
    USBSerial.printf("!!! ZMENA VERZE (%s -> %s). Inicializace defaultnich hodnot...\n", stored_ver.c_str(), VERSION);
    prefs.end();  // Zavřeme pro čtení

    // Nastavíme výchozí hodnoty (pokud by v proměnných bylo něco jiného)
    setDefaults();

    // Uložíme je do paměti (tím se zapíše i nová verze)
    saveSettings();
    return;  // Hotovo, data jsou v paměti i v proměnných
  }

  // 2. Pokud verze souhlasí, normálně načteme data
  s_min = prefs.getUInt("smin", 8000);
  s_max = prefs.getUInt("smax", 12000);
  e_low_min = prefs.getUInt("lmin", 5000);
  e_low_max = prefs.getUInt("lmax", 8000);
  e_high_min = prefs.getUInt("hmin", 12000);
  e_high_max = prefs.getUInt("hmax", 15000);
  e_min_time = prefs.getUInt("etim", 800);
  invert_logic = prefs.getBool("inv", false);
  e_maska = prefs.getUInt("emas", 1);
  columns_separator = prefs.getChar("csep", ';');
  decimal_separator = prefs.getChar("dsep", ',');

  prefs.end();
  USBSerial.println("Konfigurace nactena (verze souhlasi).");
}
void saveSettings() {
  prefs.begin("pwm-cfg", false);

  // Uložíme aktuální verzi
  prefs.putString("ver", VERSION);

  prefs.putUInt("smin", s_min);
  prefs.putUInt("smax", s_max);
  prefs.putUInt("lmin", e_low_min);
  prefs.putUInt("lmax", e_low_max);
  prefs.putUInt("hmin", e_high_min);
  prefs.putUInt("hmax", e_high_max);
  prefs.putUInt("etim", e_min_time);
  prefs.putBool("inv", invert_logic);
  prefs.putUInt("emas", e_maska);
  prefs.putChar("csep", columns_separator);
  prefs.putChar("dsep", decimal_separator);

  prefs.end();
  USBSerial.println(">> ULOZENO (Verze: " VERSION ")");
}


void setup() {
  // 1. EMC Ošetření nepoužitých pinů
  for (int i = 0; i < sizeof(UNUSED_PINS) / sizeof(int); i++) {
    pinMode(UNUSED_PINS[i], OUTPUT);
    digitalWrite(UNUSED_PINS[i], LOW);
  }

  USB.begin();        // Inicializace USB fyzické vrstvy
  USBSerial.begin();  // Inicializace sériového portu nad USB
  USBSerial.setDebugOutput(true);
  //USBSerial.setTxTimeoutMs(0);
  USBSerial.setDebugOutput(true);

  loadSettings();
  pinMode(ERROR_PIN, OUTPUT);
  digitalWrite(ERROR_PIN, LOW);

  mcpwm_cap_timer_handle_t cap_timers[2];
  for (int g = 0; g < 2; g++) {
    mcpwm_capture_timer_config_t t_cfg = { .group_id = g, .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT };
    mcpwm_new_capture_timer(&t_cfg, &cap_timers[g]);
  }

  for (int i = 0; i < NUM_CHANNELS; i++) {
    pinMode(OUTPUT_PINS[i], OUTPUT);
    digitalWrite(OUTPUT_PINS[i], LOW);

    pwmQueues[i] = xQueueCreate(1, sizeof(PwmData));
    chStates[i].last_seen = millis();
    chStates[i].error_count = 0;

    int g_idx = (i < 3) ? 0 : 1;
    mcpwm_cap_channel_handle_t cap_chan;
    mcpwm_capture_channel_config_t c_cfg = { .gpio_num = (gpio_num_t)INPUT_PINS[i], .prescale = 1 };
    c_cfg.flags.pos_edge = 1;
    c_cfg.flags.neg_edge = 1;
    c_cfg.flags.pull_up = 0;
    c_cfg.flags.pull_down = 0;

    mcpwm_new_capture_channel(cap_timers[g_idx], &c_cfg, &cap_chan);
    mcpwm_capture_event_callbacks_t cbs = { .on_cap = on_capture_callback };
    mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, (void *)i);
    mcpwm_capture_channel_enable(cap_chan);
  }

  for (int g = 0; g < 2; g++) {
    mcpwm_capture_timer_enable(cap_timers[g]);
    mcpwm_capture_timer_start(cap_timers[g]);
  }
  delay(1000);
  USBSerial.print("Detekce PWM ");
  USBSerial.println(VERSION);
  tiskni_parametry();
  USBSerial.println("HELP nebo ? nebo -h  : Vypise napovedu");
}

void tiskni_parametry(void) {
  USBSerial.printf("Inv:%d S:%d-%d L:%d-%d H:%d-%d E_MASKA:%d E_MIN_TIME:%d\n", invert_logic, s_min, s_max, e_low_min, e_low_max, e_high_min, e_high_max, e_maska, e_min_time);
  USBSerial.printf("Separatory: Sloupce='%c', Desetinny='%c'\n\n", columns_separator, decimal_separator);
}

void tiskni_help() {
  USBSerial.println("\n--- HARDWARE MAPOVANI ---");

  // Výpis vstupních pinů (Capture)
  USBSerial.print("VSTUPNI  PINY (CH 0-5): ");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (i > 0) USBSerial.print(", ");
    if (INPUT_PINS[i] < 10) USBSerial.print(" ");
    USBSerial.print(INPUT_PINS[i]);
  }
  USBSerial.println();

  // Výpis výstupních pinů (Digital Out)
  USBSerial.print("VYSTUPNI PINY (CH 0-5): ");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (i > 0) USBSerial.print(", ");
    if (OUTPUT_PINS[i] < 10) USBSerial.print(" ");
    USBSerial.print(OUTPUT_PINS[i]);
  }
  USBSerial.println();

  // Výpis Error pinu
  USBSerial.printf("ERROR PIN: %d\n", ERROR_PIN);

  USBSerial.println("\n--- SEZNAM PRIKAZU ---");
  USBSerial.println("SMIN=n, SMAX=n      : Nastaveni spinaciho okna (Hystereze v ISR)");
  USBSerial.println("LMIN=n, LMAX=n      : Validacni okno pro LOW signal (Error counting)");
  USBSerial.println("HMIN=n, HMAX=n      : Validacni okno pro HIGH signal (Error counting)");
  USBSerial.println("INVERT=0/1          : 0 = Logika HIGH, 1 = Logika LOW");
  USBSerial.println("EMASK=n             : Bitova maska pro Error Pin (0-63, napr. 63=vse, 1=jen CH0)");
  USBSerial.println("ETIME=n             : minimální počet ms indikace erroru");

  USBSerial.println("\n--- FORMATOVANI A MERENI ---");
  USBSerial.println("COLUMNS_SEPARATOR=c : Znak pro oddeleni sloupcu (napr. ; nebo ,)");
  USBSerial.println("DECIMAL_SEPARATOR=c : Znak pro desetinou carku (napr. . nebo ,)");
  USBSerial.println("*IDN?               : Vypise IDN");
  USBSerial.println(":MEAS:PER?          : Vypise periody vsech kanalu [s]");
  USBSerial.println(":MEAS:WID?          : Vypise sirku aktivniho pulzu [s] (dle INVERT)");

  USBSerial.println("\n--- SYSTEMOVE ---");
  USBSerial.println("SHOW                : Vypise aktualni nastaveni");
  USBSerial.println("SAVE                : Ulozi aktualni hodnoty do pameti Flash");
  USBSerial.println("FACTORY_RESET  *RST : Nastavi vychozi hodnoty a ulozi je");
  USBSerial.println("HELP nebo ?  -h     : Vypise tuto napovedu");
  USBSerial.println("-----------------------------------------------------------------------");
  USBSerial.println("Prikazy lze retezit, napr: DECIMAL_SEPARATOR=, COLUMNS_SEPARATOR=; SAVE");

  // Vypíšeme i aktuální stav
  USBSerial.print("\nAKTUALNI STAV: ");
  tiskni_parametry();
}

void loop() {
  unsigned long nyni = millis();

  if (USBSerial.available()) {
    // Čteme z USBSerial místo Serial
    String input = USBSerial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    char buf[128];
    input.toCharArray(buf, sizeof(buf));

    uint32_t n_smin = s_min, n_smax = s_max;
    uint32_t n_lmin = e_low_min, n_lmax = e_low_max;
    uint32_t n_hmin = e_high_min, n_hmax = e_high_max;
    uint32_t n_emask = e_maska, n_e_min_time = e_min_time;
    bool do_save = false, do_show = false, do_help = false, neco_zmeneno = false;
    bool neznama_vyskyt = false;

    char *p = strtok(buf, " ");
    while (p != NULL) {
      String token = String(p);
      token.toUpperCase();
      bool platny_token = false;

      int sep = token.indexOf('=');
      if (sep != -1) {
        String key = token.substring(0, sep);
        String valStr = token.substring(sep + 1);
        uint32_t val = valStr.toInt();

        if (key == "SMIN") {
          n_smin = val;
          platny_token = true;
          neco_zmeneno = true;
        } else if (key == "SMAX") {
          n_smax = val;
          platny_token = true;
          neco_zmeneno = true;
        } else if (key == "LMIN") {
          n_lmin = val;
          platny_token = true;
          neco_zmeneno = true;
        } else if (key == "LMAX") {
          n_lmax = val;
          platny_token = true;
          neco_zmeneno = true;
        } else if (key == "HMIN") {
          n_hmin = val;
          platny_token = true;
          neco_zmeneno = true;
        } else if (key == "HMAX") {
          n_hmax = val;
          platny_token = true;
          neco_zmeneno = true;
        } else if (key == "ETIME") {
          n_e_min_time = val;
          platny_token = true;
          neco_zmeneno = true;
        } else if (key == "INVERT") {
          invert_logic = (val == 1);
          platny_token = true;
          neco_zmeneno = true;
        } else if (key == "EMASK") {
          n_emask = val;
          platny_token = true;
          neco_zmeneno = true;
        } else if (key == "COLUMNS_SEPARATOR") {
          if (valStr.length() > 0) {
            columns_separator = valStr[0];
            USBSerial.printf("OK: Separator nastaven na '%c'\n", columns_separator);
            platny_token = true;
          }
        } else if (key == "DECIMAL_SEPARATOR") {
          if (valStr.length() > 0) {
            decimal_separator = valStr[0];
            USBSerial.printf("OK: Desetinny oddelovac nastaven na '%c'\n", decimal_separator);
            platny_token = true;
          }
        }
      } else {
        if (token == "SAVE") {
          do_save = true;
          platny_token = true;
        } else if (token == "SHOW") {
          do_show = true;
          platny_token = true;
        } else if (token == "HELP" || token == "?" || token == "-H") {
          do_help = true;
          platny_token = true;
        } else if (token == "FACTORY_RESET" || token == "*RST") {
          setDefaults();
          saveSettings();
          platny_token = true;
          USBSerial.println("OK: Proveden kompletni reset nastaveni.");
        } else if (token == "*IDN?") {
          platny_token = true;
          USBSerial.printf("Detekce az 6 kanalu PWM s ESP32-S3 %s\n", VERSION);
        } else if (token == ":MEAS:PER?" || token == ":MEASURE:PERIOD?") {

          for (int i = 0; i < NUM_CHANNELS; i++) {
            // Čteme přímo z paměti stavu kanálu, ne z fronty
            uint32_t p_us = chStates[i].last_period_us;

            // Pokud je kanál v timeoutu (signál zmizel), vynulujeme to
            if (nyni - chStates[i].last_seen > timeout_val) {
              p_us = 0;
            }

            float p_sec = p_us / 1000000.0f;
            String s_val = String(p_sec, 6);
            s_val.replace('.', decimal_separator);

            USBSerial.print(s_val);

            if (i < NUM_CHANNELS - 1) {
              USBSerial.print(columns_separator);
              if (columns_separator == ';') USBSerial.print(" ");
            }
          }
          USBSerial.println();

          platny_token = true;
        } else if (token == ":MEAS:WID?" || token == ":MEASURE:WIDTH?") {

          for (int i = 0; i < NUM_CHANNELS; i++) {
            uint32_t w_us = chStates[i].last_width_us;

            // Timeout pojistka
            if (nyni - chStates[i].last_seen > timeout_val) {
              w_us = 0;
            }

            float w_sec = w_us / 1000000.0f;
            String s_val = String(w_sec, 6);
            s_val.replace('.', decimal_separator);

            USBSerial.print(s_val);

            if (i < NUM_CHANNELS - 1) {
              USBSerial.print(columns_separator);
              if (columns_separator == ';') USBSerial.print(" ");
            }
          }
          USBSerial.println();

          platny_token = true;
        }
      }

      if (!platny_token) {
        USBSerial.printf("Neznamy prikaz: %s\n", p);
        neznama_vyskyt = true;
      }
      p = strtok(NULL, " ");
    }

    if (do_help) {
      tiskni_help();
      return;
    }

    bool valid = true;
    if (neco_zmeneno) {
      if (n_smin >= n_smax) {
        USBSerial.println("CHYBA: SMIN >= SMAX!");
        valid = false;
      }
      if (n_lmin >= n_lmax) {
        USBSerial.println("CHYBA: LMIN >= LMAX!");
        valid = false;
      }
      if (n_hmin >= n_hmax) {
        USBSerial.println("CHYBA: HMIN >= HMAX!");
        valid = false;
      }
    }

    if (valid && (neco_zmeneno || do_save || do_show)) {
      s_min = n_smin;
      s_max = n_smax;
      e_low_min = n_lmin;
      e_low_max = n_lmax;
      e_high_min = n_hmin;
      e_high_max = n_hmax;
      e_maska = n_emask;
      e_min_time = n_e_min_time;

      if (neco_zmeneno) USBSerial.println("OK: Hodnoty aktualizovany.");
      if (do_save) saveSettings();
      if (do_show) tiskni_parametry();  // Předpokládám, že tiskni_parametry je vaše SHOW funkce
    } else if (!valid) {
      USBSerial.println("Zmeny zamitnuty.");
    }
  }
  //********************************************************************************************************
  // 2. KONTROLA DAT Z FRONTY
  nyni = millis();
  bool any_err = false;
  PwmData data;

  for (int i = 0; i < NUM_CHANNELS; i++) {
    // Zkusíme vyzvednout data z fronty (nečekáme)
    if (xQueueReceive(pwmQueues[i], &data, 0) == pdTRUE) {
      chStates[i].last_seen = nyni;
      uint32_t val = invert_logic ? data.low_us : data.high_us;

      // Validace
      bool valid = (val >= e_low_min && val <= e_low_max) || (val >= e_high_min && val <= e_high_max);
      if (valid) {
        if (chStates[i].error_count > 0) chStates[i].error_count--;
      } else {
        if (chStates[i].error_count < 20) chStates[i].error_count++;
      }

      static unsigned long last_log = 0;
      if (nyni - last_log > 2000) {
        log_d("CH%d: P:%u H:%u L:%u Err:%d", i, data.period_us, data.high_us, data.low_us, chStates[i].error_count);
        if (i == 5) last_log = nyni;
      }
    } else {
      // Pokud ve frontě dlouho nic nebylo -> Timeout
      if (nyni - chStates[i].last_seen > timeout_val) {
        if (digitalRead(OUTPUT_PINS[i]) == HIGH) {
          digitalWrite(OUTPUT_PINS[i], LOW);
          log_w("CH%d: Timeout", i);
        }
        chStates[i].error_count = 20;
      }
    }
    if (chStates[i].error_count > 5) {
      if ((e_maska >> i) & 1) {
        any_err = true;
      }
    }
  }

  // Alarm
  // 3. PERIODICKÉ ÚLOHY
  static unsigned long last_slow_task = 0;
  if (nyni - last_slow_task >= 50) {  // Spustí se každých 50 ms
    last_slow_task = nyni;

    if (any_err) {
      if (!alarm_active) {
        digitalWrite(ERROR_PIN, HIGH);
        alarm_active = true;
        alarm_start = nyni;
      }
    } else if (alarm_active && (nyni - alarm_start > e_min_time)) {
      digitalWrite(ERROR_PIN, LOW);
      alarm_active = false;
    }
  }
}