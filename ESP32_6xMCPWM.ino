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

#define MAX_ERROR_COUNT 255

// --- GLOBÁLNÍ PROMĚNNÉ ---
const int INPUT_PINS[NUM_CHANNELS] = { 4, 5, 6, 7, 15, 16 };
const int OUTPUT_PINS[NUM_CHANNELS] = { 10, 11, 12, 13, 1, 2 };
const int ERROR_PIN = 21;
// Seznam zbývajících volných pinů pro EMC stabilitu
const int UNUSED_PINS[] = { 17, 18, 8, 3, 46, 9, 14, 42, 41, 40, 39, 38, 37, 36, 35, 45, 48, 47 };

QueueHandle_t pwmQueues[NUM_CHANNELS];
ChannelState chStates[NUM_CHANNELS];

// promene nastavitelne prikazem po seriove lince lze nastavit na tovarni nastaveni pomoci setDefaults()
uint32_t s_min, s_max, e_low_min, e_low_max, e_high_min, e_high_max, e_maska, e_min_time, e_filtr, timeout_val;
uint8_t e_quantity;
bool invert_logic;
char columns_separator, decimal_separator;

// pro loop
bool ERROR_BEZI = false;            // zda je spusten error
unsigned long alarm_start = 0;  // pomocna pro periodycke deje v loop

// definice globálně pro seriovou linku
char vstupni_buf[128];
int v_index = 0;
bool data_komplet = false;

// --- ISR CALLBACK (Plnění fronty) ---
static bool on_capture_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data) {
  int ch = (int)user_data;
  uint32_t now = edata->cap_value;

  // Statické pole pro uložení času POSLEDNÍ JAKÉKOLIV hrany pro každý kanál
  static uint32_t last_any_edge[NUM_CHANNELS] = { 0 };

  // Výpočet rozdílu v tiktech (80 tiků = 1us)
  // Ošetření přetečení 32-bitového čítače
  // uint32_t diff = (now >= last_any_edge[ch]) ? (now - last_any_edge[ch]) : (0xFFFFFFFF - last_any_edge[ch] + now);
  uint32_t diff = now - last_any_edge[ch]; 

  // FILTR: Pokud je změna stavu příliš rychlá (méně než např 1us), ignoruj ji.
  // To odfiltruje kmity, které prošly přes optočlen nebo RC filtr.
  if (diff < e_filtr) {
    return false;
  }
  last_any_edge[ch] = now;
  uint32_t t_time_out = 80000 * timeout_val;  // prevede na tiky

  BaseType_t high_task_wakeup = pdFALSE;
  if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
    uint32_t period_ticks = now - chStates[ch].pos_temp;
    uint32_t low_ticks = now - chStates[ch].neg_temp;


    if (period_ticks > t_time_out || low_ticks > t_time_out) {
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

    if (high_ticks > t_time_out) return false;
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
  s_min = 8000;          // [us]
  s_max = 12000;         // [us]
  e_low_min = 5000;      // [us]
  e_low_max = 8000;      // [us]
  e_high_min = 13000;    // [us]
  e_high_max = 15000;    // [us]
  e_min_time = 800;      // [ms]
  e_quantity = 1;        // [0-255]
  e_maska = 1;           // [0-63]
  invert_logic = false;  // [0-1]
  timeout_val = 100;     // [ms]
  e_filtr = 80;          // [80tiku=1us]

  columns_separator = ';';
  decimal_separator = ',';

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
  e_quantity = prefs.getUInt("equn", 800);
  invert_logic = prefs.getBool("inv", false);
  e_maska = prefs.getUInt("emas", 1);
  timeout_val = prefs.getUInt("tout", 100);
  e_filtr = prefs.getUInt("efil", 80);
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
  prefs.putUInt("equn", e_quantity);
  prefs.putBool("inv", invert_logic);
  prefs.putUInt("emas", e_maska);
  prefs.putUInt("tout", timeout_val);
  prefs.putUInt("efil", e_filtr);
  prefs.putChar("csep", columns_separator);
  prefs.putChar("dsep", decimal_separator);

  prefs.end();
  USBSerial.println(">> ULOZENO (Verze: " VERSION ")");
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
  USBSerial.println("SMIN=n, SMAX=n      : Nastaveni spinaciho okna (Hystereze v ISR)[us]");
  USBSerial.println("INVERT=0/1          : 0 = Logika HIGH, 1 = Logika LOW");
  USBSerial.println("LMIN=n, LMAX=n      : Validacni okno pro LOW signal (Error counting)[us]");
  USBSerial.println("HMIN=n, HMAX=n      : Validacni okno pro HIGH signal (Error counting)[us]");
  USBSerial.println("EMASK=n             : Bitova maska pro Error Pin (0-63, napr. 63=vse, 1=jen CH0)");
  USBSerial.println("EQUNT=n             : Min pocet za sebou jdoucich detekci erroru nez se nastavi ERROR PIN (255-OFF)");
  USBSerial.println("ETIME=n             : Min pocet [ms] indikace erroru");
  USBSerial.println("ETOUT=n             : Doba za jak dlouho bez signalu se vyhodnoti error a uz se na nic neceka [ms]");
  USBSerial.println("EFILTR=n            : Kolik tiku musi trvat signal, aby byl vyhodnocovan [80=1us]");

  USBSerial.println("\n--- FORMATOVANI A MERENI ---");
  USBSerial.println("Columns_SEParator=c CSEP=n : Znak pro oddeleni sloupcu (napr. ; nebo ,)");
  USBSerial.println("Decimal_SEParator=c DSEP=n : Znak pro desetinou carku (napr. . nebo ,)\n");
  USBSerial.println("*IDN?               : Vypise IDN");
  USBSerial.println(":MEASure:PERiod?    : Vypise periody vsech kanalu [s]");
  USBSerial.println(":MEASure:WIDth?     : Vypise sirku aktivniho pulzu [s] (dle INVERT)");

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

/**
 * @brief Ověří hodnotu proti rozsahu a zapíše ji do cílové proměnné.
 * * @tparam T Typ cílové proměnné (např. int, uint8_t).
 * @param target Reference na proměnnou, která má být aktualizována.
 * @param val Nová hodnota ke kontrole.
 * @param min Minimální povolená hodnota.
 * @param max Maximální povolená hodnota.
 * @param name Název parametru pro chybový výpis.
 * @param changed Příznak, který se nastaví na true při úspěšné změně.
 * @param valid Příznak, který se nastaví na false při chybě rozsahu.
 */
template<typename T>
void validateAndSet(T &target, long val, long min, long max, const char *name, bool &changed, bool &valid) {
  if (val >= min && val <= max) {
    target = (T)val;  // Přetypování na cílový typ (např. uint8_t)
    changed = true;
  } else {
    USBSerial.printf("CHYBA: %s=%ld mimo rozsah (%ld-%ld)\n", name, val, min, max);
    valid = false;
  }
}

void tiskni_parametry(void) {
  USBSerial.printf("SMIN=%d SMAX=%d INVERT=%d LMIN=%d LMAX=%d HMIN=%d HMAX=%d EMASK=%d ETIME=%d EQUNT=%d ETOUT=%d EFILTR=%d\n",
                   s_min, s_max, invert_logic, e_low_min, e_low_max, e_high_min, e_high_max, e_maska, e_min_time, e_quantity, timeout_val, e_filtr);
  USBSerial.printf("COLUMNS_SEPARATOR='%c', DECIMAL_SEPARATOR='%c'\n\n", columns_separator, decimal_separator);
}

void nactiSerial() {
  while (USBSerial.available() > 0) {
    char c = USBSerial.read();

    // 1. Konec řádku - ukončujeme příjem
    if (c == '\n' || c == '\r') {
      if (v_index > 0) {
        vstupni_buf[v_index] = '\0';  // Ukončovací znak
        data_komplet = true;
      }
      return;  // Vyskočíme z funkce, aby se loop mohl věnovat parseru
    }

    // 2. Ochrana proti přetečení a zápis znaku
    if (v_index < sizeof(vstupni_buf) - 1) {
      // Volitelně: převod na velká písmena už při příjmu ušetří práci parseru
      vstupni_buf[v_index++] = toupper(c);
    }
  }
}

void zpracujSerial() {
  data_komplet = false;
  int aktualni_len = v_index;  // zapamatujeme si délku, pokud ji budeme potřebovat
  v_index = 0;
  // Lokální kopie pro transakční zpracování
  uint32_t n_smin = s_min, n_smax = s_max;
  uint32_t n_lmin = e_low_min, n_lmax = e_low_max;
  uint32_t n_hmin = e_high_min, n_hmax = e_high_max;
  uint32_t n_emask = e_maska, n_e_min_time = e_min_time, n_timeout_val = timeout_val, n_e_filtr = e_filtr;
  uint8_t n_e_quantity = e_quantity;
  bool n_invert_logic = invert_logic;

  bool do_save = false, do_show = false, do_help = false, neco_zmeneno = false;
  bool valid = true;

  char *p = strtok(vstupni_buf, " ");
  while (p != NULL) {
    bool platny_token = false;
    char *sep = strchr(p, '=');  // Najde pozici '=' v aktuálním slově

    if (sep != NULL) {
      // ROZDĚLENÍ NA KLÍČ A HODNOTU (přímo v bufferu)
      *sep = '\0';  // Rozdělí řetězec na dvě části (klíč a hodnota)
      char *key = p;
      char *valStr = sep + 1;
      long val = atol(valStr);  // Převod na číslo

      // Porovnání klíčů bez ohledu na velikost písmen (Case Insensitive)
      if (strcasecmp(key, "SMIN") == 0) {
        platny_token = true;
        validateAndSet(n_smin, val, 0, 1000000, "SMIN", neco_zmeneno, valid);
      } else if (strcasecmp(key, "SMAX") == 0) {
        platny_token = true;
        validateAndSet(n_smax, val, 1, 1000000, "SMAX", neco_zmeneno, valid);
      } else if (strcasecmp(key, "INVERT") == 0) {
        platny_token = true;
        validateAndSet(n_invert_logic, val, 0, 1, "INVERT", neco_zmeneno, valid);
      } else if (strcasecmp(key, "LMIN") == 0) {
        platny_token = true;
        validateAndSet(n_lmin, val, 0, 1000000, "LMIN", neco_zmeneno, valid);
      } else if (strcasecmp(key, "LMAX") == 0) {
        platny_token = true;
        validateAndSet(n_lmax, val, 1, 1000000, "LMAX", neco_zmeneno, valid);
      } else if (strcasecmp(key, "HMIN") == 0) {
        platny_token = true;
        validateAndSet(n_hmin, val, 0, 1000000, "HMIN", neco_zmeneno, valid);
      } else if (strcasecmp(key, "HMAX") == 0) {
        platny_token = true;
        validateAndSet(n_hmax, val, 1, 1000000, "HMAX", neco_zmeneno, valid);
      } else if (strcasecmp(key, "EMASK") == 0) {
        platny_token = true;
        validateAndSet(n_emask, val, 0, 63, "EMASK", neco_zmeneno, valid);
      } else if (strcasecmp(key, "ETIME") == 0) {
        platny_token = true;
        validateAndSet(n_e_min_time, val, 1, 10000, "ETIME", neco_zmeneno, valid);
      } else if (strcasecmp(key, "EQUNT") == 0) {
        platny_token = true;
        validateAndSet(n_e_quantity, val, 0, 255, "EQUNT", neco_zmeneno, valid);
      } else if (strcasecmp(key, "ETOUT") == 0) {
        platny_token = true;
        validateAndSet(n_timeout_val, val, 1, 65000, "ETOUT", neco_zmeneno, valid);
      } else if (strcasecmp(key, "EFILTR") == 0) {
        platny_token = true;
        validateAndSet(n_e_filtr, val, 0, 80000, "EFILTR", neco_zmeneno, valid);
      } else if (strcasecmp(key, "CSEP") == 0 || strcasecmp(key, "COLUMNS_SEPARATOR") == 0) {
        if (strlen(valStr) > 0) {
          columns_separator = valStr[0];
          USBSerial.printf("OK: Separator nastaven na '%c'\n", columns_separator);
          platny_token = true;
        }
      } else if (strcasecmp(key, "DSEP") == 0 || strcasecmp(key, "DECIMAL_SEPARATOR") == 0) {
        if (strlen(valStr) > 0) {
          decimal_separator = valStr[0];
          USBSerial.printf("OK: Desetinny oddelovac nastaven na '%c'\n", decimal_separator);
          platny_token = true;
        }
      }
    } else {
      // PŘÍKAZY BEZ "="
      if (strcasecmp(p, "SAVE") == 0) {
        do_save = true;
        platny_token = true;
      } else if (strcasecmp(p, "SHOW") == 0) {
        do_show = true;
        platny_token = true;
      } else if (strcasecmp(p, "HELP") == 0 || strcmp(p, "?") == 0 || strcasecmp(p, "-H") == 0) {
        do_help = true;
        platny_token = true;
      } else if (strcasecmp(p, "FACTORY_RESET") == 0 || strcasecmp(p, "*RST") == 0) {
        setDefaults();
        saveSettings();
        USBSerial.println("OK: Proveden kompletni reset.");
        platny_token = true;
      } else if (strcasecmp(p, "*IDN?") == 0) {
        USBSerial.printf("PWM Detektor ESP32-S3 %s\n", VERSION);
        platny_token = true;
      } else if (strcasecmp(p, ":MEAS:PER?") == 0 || strcasecmp(p, ":MEASURE:PERIOD?") == 0) {
        for (int i = 0; i < NUM_CHANNELS; i++) {
          uint32_t p_perioda_us = (millis() - chStates[i].last_seen > n_timeout_val) ? 0 : chStates[i].last_period_us;
          float p_sec = p_perioda_us / 1000000.0f;

          // Efektivní tisk s nahrazením tečky za tvůj oddělovač
          char fbuf[16];
          snprintf(fbuf, sizeof(fbuf), "%.6f", p_sec);
          char *dot = strchr(fbuf, '.');
          if (dot) *dot = decimal_separator;

          USBSerial.print(fbuf);
          if (i < NUM_CHANNELS - 1) {
            USBSerial.print(columns_separator);
            if (columns_separator == ';') USBSerial.print(" ");
          }
        }
        USBSerial.println();
        platny_token = true;
      } else if (strcasecmp(p, ":MEAS:WID?") == 0 || strcasecmp(p, ":MEASURE:WIDTH?") == 0) {
        for (int i = 0; i < NUM_CHANNELS; i++) {
          uint32_t p_width_us = (millis() - chStates[i].last_seen > n_timeout_val) ? 0 : chStates[i].last_width_us;
          float p_sec = p_width_us / 1000000.0f;

          // Efektivní tisk s nahrazením tečky za muj oddělovač
          char fbuf[16];
          snprintf(fbuf, sizeof(fbuf), "%.6f", p_sec);
          char *dot = strchr(fbuf, '.');
          if (dot) *dot = decimal_separator;

          USBSerial.print(fbuf);
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
    }
    p = strtok(NULL, " ");
  }

  // FINÁLNÍ KONTROLA LOGIKY (Cross-parameter validation)
  if (valid && neco_zmeneno) {
    if (n_smin >= n_smax || n_lmin >= n_lmax || n_hmin >= n_hmax) {
      USBSerial.println("CHYBA: Min >= Max!");
      valid = false;
    }
  }

  // ZÁPIS DO OSTRÝCH PROMĚNNÝCH
  if (valid && (neco_zmeneno || do_save || do_show || do_help)) {
    if (do_help) {
      tiskni_help();
      return;
    }

    invert_logic = n_invert_logic;
    s_min = n_smin;
    s_max = n_smax;
    e_low_min = n_lmin;
    e_low_max = n_lmax;
    e_high_min = n_hmin;
    e_high_max = n_hmax;
    e_maska = n_emask;
    e_min_time = n_e_min_time;
    timeout_val = n_timeout_val;
    e_quantity = n_e_quantity;
    e_filtr = n_e_filtr;

    if (neco_zmeneno) USBSerial.println("OK: Hodnoty aktualizovany.");
    if (do_save) saveSettings();
    if (do_show) tiskni_parametry();
  }
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

void loop() {
  //********************************************************************************************************
  // 1. SERIOVA LINKA
  nactiSerial();
  // pokud byl načten kompletni retezec tak se zpracuje
  if (data_komplet) { zpracujSerial(); }

  //********************************************************************************************************
  // 2. KONTROLA DAT Z FRONTY
  unsigned long nyni = millis();
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
        if (chStates[i].error_count > 0) chStates[i].error_count = 0;  //chStates[i].error_count--
      } else {
        if (chStates[i].error_count < MAX_ERROR_COUNT) chStates[i].error_count++;
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
        chStates[i].error_count = MAX_ERROR_COUNT;
      }
    }
    if (chStates[i].error_count > e_quantity) {
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
      if (!ERROR_BEZI) {
        digitalWrite(ERROR_PIN, HIGH);
        log_w("Nastaven Error\n");
        ERROR_BEZI = true;
        alarm_start = nyni;
      }
    } else if (ERROR_BEZI && (nyni - alarm_start > e_min_time)) {
      digitalWrite(ERROR_PIN, LOW);
      log_w("Vypnut Error\n");
      ERROR_BEZI = false;
    }
  }
}