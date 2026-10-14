/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma GCC optimize ("Os")

#include "app.h"
#ifdef HW_HAS_LUNA_SERIAL_DISPLAY
#include "luna_display_serial.h"
#endif

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "timeout.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "comm_can.h"
#include "hw.h"
#include "mempools.h"
#include "conf_general.h"
#include <math.h>

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8
#define TC_DIFF_MAX_PASS				60  // TODO: move to app_conf

#define CTRL_USES_BUTTON(ctrl_type)(\
		ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON || \
		ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC || \
		ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER || \
		ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON || \
		ctrl_type == ADC_CTRL_TYPE_DUTY_REV_BUTTON || \
		ctrl_type == ADC_CTRL_TYPE_PID_REV_BUTTON)

// Threads
static THD_FUNCTION(adc_thread, arg);
__attribute__((section(".ram4"))) static THD_WORKING_AREA(adc_thread_wa, 512);

// Private variables
static volatile adc_config config;
static volatile float ms_without_power = 0.0;
static volatile float decoded_level = 0.0;
static volatile float read_voltage = 0.0;
static volatile float decoded_level2 = 0.0;
static volatile float read_voltage2 = 0.0;
static volatile float adc1_override = 0.0;

// --- Rekuperation ueber die Displaystufe ------------------------------
// ABSOLUTER Motorstrom in Ampere, nicht mehr als Anteil. Probefahrt vom
// 20.09.: Als Anteil des Maximums waren die unteren Stufen zu schwach,
// weil sie sich immer auf den Endwert bezogen. Absolut gesetzt bleibt
// jede Stufe das, was sie sein soll, auch wenn die Grenzen sich aendern.
// Der Akkustrom (l_in_current_min) deckelt die Leistung weiterhin - das
// ist die Aufgabe des BMS-Schutzes, nicht der Stufenkennlinie.
static volatile float luna_regen_amps = 0.0;
// Weich anlegen, damit der Hebel nicht zubeisst. Das Loesen bleibt
// bewusst hart - eine geloeste Bremse soll sofort frei sein.
static float luna_regen_ramp = 0.0;
// (volker, 10.10.2026) Hat die App zuletzt selbst Strom gesetzt? Freigegeben
// wird nur noch beim Uebergang "App treibt -> keine Anforderung", nicht in
// jedem Takt. Sonst ueberschrieb die App auch fremde Befehle, etwa den
// Motortest aus VESC Tool. Die App selbst fordert weiterhin nie Strom an,
// wo sie es nicht soll.
static bool luna_app_treibt = false;
static void luna_freigeben(void) {
	if (luna_app_treibt) {
		mc_interface_release_motor();
		luna_app_treibt = false;
	}
}
#define LUNA_REGEN_RAMP_UP_S    0.10f   // 07.10.2026: 0,30 -> 0,10 (Bremse stoerungsfrei)
#define LUNA_REGEN_RAMP_DOWN_S  0.08f

// (volker, 30.09.2026) REKU BEI KALTEM AKKU ZURUECKNEHMEN.
// Die Molicel M65A duerfen laut Datenblatt erst ab 0 Grad geladen werden, und
// Rekuperation IST Laden. Ohne Fuehler sperrt nur das BMS - hart, mitten in
// der Bremsung, mit Spannungsspitze am Controller. Mit einer NTC im Akku nimmt
// die Firmware die Reku vorher weich zurueck:
//   ab LUNA_AKKU_T_VOLL     volle Reku
//   dazwischen              linear weniger
//   ab LUNA_AKKU_T_NULL     keine Reku
// Fuehler fehlt oder defekt (Spannung am Anschlag): keine Ruecknahme, wie
// bisher - die Kaeltesperre des BMS bleibt die letzte Instanz. luna_pins
// zeigt das an.
// Der Eingang wird im Board-Header mit LUNA_AKKU_NTC_ADC festgelegt. Ist er
// nicht definiert, ist das alles abgeschaltet.
#ifndef LUNA_AKKU_NTC_BETA
#define LUNA_AKKU_NTC_BETA      3950.0f   // B-Wert der NTC, vom Datenblatt
#endif
#ifndef LUNA_AKKU_NTC_R_PULLUP
#define LUNA_AKKU_NTC_R_PULLUP  10000.0f  // Widerstand von 3,3 V an den Eingang
#endif
#define LUNA_AKKU_T_VOLL        10.0f     // Grad, ab hier volle Reku
#define LUNA_AKKU_T_NULL        2.0f      // Grad, ab hier keine Reku
// Schiebehilfe: fester Motorstrom bis zur Tempogrenze.
static volatile bool luna_walk_assist = false;
// Schiebehilfe: absoluter Motorstrom, aus der Displaystufe gesetzt.
// Tabelle in luna_display_serial.c: 16 A bei ausgeschaltetem Antrieb,
// dann in Vierer-Schritten bis 36 A. Der Startwert gilt, bis das Display
// eine Stufe geschickt hat.
static volatile float luna_walk_amps = 16.0;
// Volle Kraft bis 20 km/h, dann linear auf null bei 25 km/h. Vorher gab es
// eine harte Kante bei 20 km/h - die Unterstuetzung brach schlagartig ab,
// und in der Praxis waren nur 19 km/h erreichbar, weil der Motor am
// Grenzwert zu zappeln begann. Mit der Rampe sind 20 km/h tatsaechlich
// fahrbar, ohne zu treten.
// ======================================================================
// RECHTLICHER RAHMEN - seit 11.10.2026 nach UUID (hw_75_100_V2.c, s_pedelec)
//
// Jeder unbekannte Controller: PEDELEC 25 (Fahrrad, keine Zulassung).
//   VO (EU) 168/2013 Art. 2 Abs. 2 h: Unterstuetzung nur beim Treten,
//   wird mit steigendem Tempo verringert und bei 25 km/h abgeschaltet;
//   Nenndauerleistung hoechstens 250 W (Typschild des Motors).
//   Deutschland: Schiebe-/Anfahrhilfe ohne Treten bis 6 km/h.
//   Diese Firmware: Unterstuetzung voll bis 24, null bei 25 km/h;
//   Schiebehilfe voll bis 5, null bei 6 km/h; Akkustrom 10 A (~500 W
//   Spitze). Die 250 W sind eine Eigenschaft des Motors, nicht der
//   Firmware - ein Motor mit hoeherer Nennleistung bleibt unzulaessig.
//
// Profil mit s_pedelec = true: S-PEDELEC (Klasse L1e-B).
//   NUR mit Typgenehmigung/Einzelabnahme, Versicherungskennzeichen,
//   Fuehrerschein AM und Helm. Hoechstgeschwindigkeit 45 km/h, Toleranz
//   +5 % nach VO (EU) 134/2014 Anhang X Anlage 1 Nr. 8.1 -> null bei
//   47,25 km/h. Schiebehilfe bis 20 km/h (+5 % -> null bei 21 km/h).
//
// Gerechnet mit 23 Polpaaren und dem Raddurchmesser der Motorkonfiguration
// (VESC Tool: Motor Settings -> Additional Info -> Wheel Diameter). Seit
// 10.10.2026 gilt dieser Wert, damit ein Reifenwechsel ohne neuen Build geht.
// ACHTUNG: Er verschiebt Tacho UND Abriegelung - nach Reifenwechsel richtig
// einstellen (am besten per GPS-Vergleich). Standard 0,700 m, GPS weiss 0,695 m.
// ======================================================================
#define LUNA_KMH2ERPM(v)  ((v) / 3.6f / (3.14159f * mc_interface_get_configuration()->si_wheel_diameter) * 60.0f * 23.0f)
#define LUNA_SPED                (luna_profil()->s_pedelec)
#define LUNA_UNTERST_VOLL_KMH   (LUNA_SPED ? 45.0f : 24.0f)
#define LUNA_UNTERST_NULL_KMH   (LUNA_SPED ? 47.25f : 25.0f)
#define LUNA_WALK_ERPM_FULL     LUNA_KMH2ERPM(LUNA_SPED ? 20.0f : 5.0f)
#define LUNA_WALK_ERPM_ZERO     LUNA_KMH2ERPM(LUNA_SPED ? 21.0f : 6.0f)
#define LUNA_P25_AKKU_A         10.0f   // Pedelec 25: Akkustrom hoechstens

// --- Mitschnitt der Bremsvorgaenge -----------------------------------
// Der Flipsky hat keine SD-Karte, also RAM. Reicht fuer eine Probefahrt:
// nach der Rueckkehr mit dem Terminalbefehl luna_log auslesen. ACHTUNG -
// der Controller darf zwischen Fahrt und Auslesen nicht stromlos werden.
// --- Verlaufsaufzeichnung ueber die Lichttaste ------------------------
// Die Lichttaste am Display (16 1A F1/F0) schaltet die Aufzeichnung ein
// und aus. Gedacht fuer Bergfahrten: Taste unten druecken, hochfahren,
// oben ausschalten, danach mit "luna_trace" auslesen. Damit laesst sich
// die Verstaerkung ueber Trittfrequenz und Moment abstimmen, statt sie
// zu erfuehlen. Der Lichtausgang selbst ist nicht belegt, die Taste also
// frei verfuegbar.
//
// (volker, 26.09.) RINGPUFFER MIT MEHREREN SEQUENZEN.
// Vorher: jedes Einschalten des Lichts begann bei null und nach 512 Punkten
// stand die Aufnahme. Wer im Nachhinein wissen wollte, was am Ende der Fahrt
// war, hatte Pech. Jetzt laeuft der Puffer um: das Aelteste wird
// ueberschrieben, und jede Licht-Ein/Aus-Runde ist eine eigene Sequenz.
//
// (volker, 27.09.) DIE AUFZEICHNUNG LAEUFT IMMER, nur der Takt wechselt:
//   Licht an               : jede Sekunde   - fuer genaues Hinsehen
//   Display an, Licht aus  : alle 20 s      - fuer eine ganze Akkuladung
//   Display aus            : alle 5 min     - Rad wird nicht benutzt, es
//                                              aendert sich nur die Temperatur
// Kriterium ist das Display, nicht der Stillstand (Volkers Vorschlag):
// Display aus heisst, das Rad steht. Jeder Punkt traegt seine eigene Zeit.
// Eine Sequenz beginnt beim Start und bei jedem Wechsel dieser drei Zustaende.
// 2048 Punkte zu 16 Byte = 32 KB im CCM (ohne LispBM 35 KB frei). Das
// reicht fuer rund 11 h mit Display, 34 min mit Licht oder 7 Tage ohne Display.
// Wer mit LispBM baut, bekommt einen Linkerfehler (CCM voll) - dann
// LUNA_TRACE_N auf 256 senken.
#define LUNA_TRACE_N        2048
#define LUNA_TRACE_SEG_N    64      // Sequenzen (04.10.: 32 liefen in der Pause ueber)
#define LUNA_TRACE_T_LICHT  1       // s, Licht an
#define LUNA_TRACE_T_DISPL  10      // s, Display an, Licht aus (2048 Pkt = 5,7 h)
#define LUNA_TRACE_T_AUS    300     // s, Display aus
#define LUNA_MODUS_AUS      0
#define LUNA_MODUS_DISPLAY  1
#define LUNA_MODUS_LICHT    2

typedef struct {
	uint32_t t_s;         // Sekunden seit Start des Controllers
	int16_t  i_mot_dA;    // Motorstrom, 0,1 A - seit 13.10.2026 EFFEKTIVWERT
	                      // sqrt(Mittel I^2) im Intervall, Vorzeichen vom Mittel
	int16_t  i_in_dA;     // Akkustrom, 0,1 A
	uint16_t v_in_cV;     // Spannung, 0,01 V - fein, damit sich der
	                      // Innenwiderstand daraus rechnen laesst
	uint8_t  kmh_2;       // Tempo in 0,5 km/h (statt ERPM: spart ein Byte
	                      // fuer die Temperatur, 0,5 km/h genuegt)
	uint8_t  temp_fet;    // Endstufentemperatur in Grad, 0 = unter null
	uint8_t  pedal_rpm;   // Trittfrequenz
	uint8_t  soll_A;      // seit 13.10.2026: Spitzenstrom |I_Mot| im Intervall, A
	uint8_t  flags;       // Bit0 Bremse, 1 Schiebehilfe, 2 Reku, 3 Stoerimpuls,
	                      // Bit4 erster Punkt einer Sequenz
	uint8_t  torque_pct;  // ADC2 nach Abbildung, 0..100 %
} luna_trace_t;

// Eine Sequenz = ein Abschnitt mit gleichem Lichtzustand. Die Indizes sind
// fortlaufend gezaehlt, nicht Pufferplaetze - so bleibt die Zuordnung
// richtig, auch wenn der Anfang der Sequenz schon ueberschrieben ist.
typedef struct {
	uint32_t idx_start;
	uint32_t idx_end;     // erster Index NACH der Sequenz
	uint32_t t_start;     // Sekunden seit Start des Controllers
	uint8_t  modus;       // LUNA_MODUS_AUS / _DISPLAY / _LICHT
} luna_trace_seg_t;

__attribute__((section(".ram4"))) static luna_trace_t luna_trace_buf[LUNA_TRACE_N];
__attribute__((section(".ram4"))) static luna_trace_seg_t luna_trace_seg[LUNA_TRACE_SEG_N];
static volatile uint32_t luna_trace_total = 0;   // je geschriebene Punkte
static volatile uint32_t luna_seg_total = 0;     // je begonnene Sequenzen
static volatile bool luna_trace_on = false;
static volatile bool luna_trace_mark = false;    // naechster Punkt ist Sequenzanfang
static volatile uint32_t luna_sek = 0;           // Sekundenuhr, ueberlauffest
static volatile float luna_akku_t = 0.0;         // Akkutemperatur, gefiltert
static volatile bool luna_akku_t_ok = false;     // Fuehler liefert plausible Werte
static volatile float luna_kaelte = 1.0;         // Faktor auf den Reku-Strom

// (volker, 01.10.2026) WICKLUNGSTEMPERATUR AUS DEM WIDERSTAND.
// Ziel: herausfinden, ob am Berg der Motor oder der Controller das schwache
// Glied ist, und daraus ein thermisches Abbild bauen. Kupfer: +0,393 % je K.
// Gemessen wird mit der VESC-eigenen Routine (dieselbe wie bei der
// Motorerkennung): Statorfeld auf festen Winkel, 10 A, Spannung/Strom.
// Der Rotor richtet sich dabei nach dem Feld aus - bis 1/46 Umdrehung. Darum
// NUR bei gezogener Bremse im Stand: der Hebel haelt das Rad mechanisch fest.
//   - Bremse im Stand LUNA_RMESS_HALTEN_MS gehalten, dann Lichttaste
//     (an oder aus) -> einmal messen, Licht blinkt 0,1 s (seit 04.10.2026)
// Ohne Bezug keine Temperatur: "luna_rmess kalib" nach langem Stehen, wenn
// Wicklung, Endstufe und Umgebung gleich warm sind.
#define LUNA_RMESS_STROM        10.0f     // A
// 500 statt 2000 wie im Terminal: eine Messung dauert dann ~0,75 s statt
// 2,25 s. Gemessen 02.10.: 2000 Proben streuen 0,07 mOhm = 0,18 K, mit 500
// erwartet ~0,4 K - weit genauer als noetig, und an der Ampel praktischer.
#define LUNA_RMESS_SAMPLES      500
#define LUNA_RMESS_HALTEN_MS    2000      // Bremse im Stand so lange halten
// (volker, 04.10.2026) 20 statt 50 ERPM: Winterstein, Messungen streuten
// 27..41 C binnen 45 s, eine zeigte 100 C. 10 ERPM Gegenspannung sind schon
// ~6 % des Messwerts (~15 K). Stillstandsrauschen am Tisch: bis 14 ERPM.
#define LUNA_RMESS_RUHE_ERPM    20.0f     // darueber bewegt sich der Rotor
#define LUNA_RMESS_ALPHA        0.00393f  // Kupfer, je Kelvin
#define LUNA_RMESS_N            64

typedef struct {
	uint32_t t_s;         // Sekunden seit Start (gleiche Uhr wie luna_trace)
	float    r;           // Ohm
	float    t_fet;       // Endstufe beim Messen
	float    v_in;        // Akkuspannung beim Messen
	uint16_t dauer_ms;
	int8_t   fault;       // 0 = gut
} luna_rmess_t;

static luna_rmess_t luna_rmess_buf[LUNA_RMESS_N];
static volatile uint32_t luna_rmess_total = 0;
// (volker, 10.10.2026) Bezug je Motor aus dem Profil (hw_75_100_V2.c),
// gesetzt beim Start des App-Threads.
static float luna_rmess_r_ref = 0.0;
static float luna_rmess_t_ref = 0.0;
static bool luna_rmess_ref_ok = false;
static uint32_t luna_rmess_halt_ms = 0;
// (volker, 04.10.2026) Gemessen wird nur noch auf ausdrueckliches Zeichen:
// im Stand Bremse >= 2 s halten, dann Lichttaste (an oder aus). Vorher mass
// jeder Halt mit 2 s Bremse - Volker wusste unterwegs nicht, wann.
static volatile bool luna_rmess_anfrage = false;
// (volker, 02.10.) Nach einer Messung meldet die Drehzahlschaetzung kurz
// eine kleine Drehzahl - der Rotor wurde ja ausgerichtet. Ohne Nachlauf hielt
// die Firmware das fuer "Rad rollt", setzte den Halt zurueck und mass alle
// 2,7 s neu (Fahrt vom 02.10., Hebel festgebunden). So lange gilt das Rad
// nach einer Messung weiter als stehend.
#define LUNA_RMESS_NACHLAUF_MS  1000
static int luna_rmess_nachlauf_ms = 0;

// Ein neuer Halt beginnt nur, wenn der Hebel losgelassen wurde.
// (volker, 13.10.2026) AUTOMATIK, schaltbar mit Bremse (>= 2 s, Stand) +
// Schiebehilfe-Taste. An: sofort eine Messung, danach bei jedem Halt mit
// >= 2 s Bremse eine (hoechstens einmal je LUNA_RMESS_AUTO_ABSTAND_S).
// Aus: keine Messung. Beim Start aus. Jede Messung zeigt 10 s die
// Temperatur im Tacho; die Lichttaste hat damit nichts mehr zu tun.
#define LUNA_RMESS_AUTO_ABSTAND_S  60
static volatile bool luna_rmess_auto = false;
static bool luna_rmess_halt_gemessen = false;
static uint32_t luna_rmess_letzte_s = 0;
static bool luna_rmess_walk_alt = false;

static void luna_rmess_reset_halt(void) {
	luna_rmess_halt_ms = 0;
	luna_rmess_anfrage = false;
	luna_rmess_halt_gemessen = false;
	luna_rmess_walk_alt = luna_walk_assist;   // Taste ohne Bremse zaehlt nicht
}

// Aufruf bei jedem Durchlauf, solange die Bremse im Stand gezogen ist.
static void luna_rmess_im_halt(int ms_je_durchlauf) {
	// (volker, 03.10.2026) Tischtest: Kurbel langsam von Hand gedreht,
	// Rad rollt mit ~1 km/h, die Schaetzung springt zwischen 0 und 200 ERPM
	// und lag damit meist unter LUNA_STANDSTILL_ERPM. Im echten Stand
	// rauscht sie nur bis ~15 ERPM. Jeder Ausreisser ueber
	// LUNA_RMESS_RUHE_ERPM startet die Haltezeit neu.
	if (fabsf(mc_interface_get_rpm()) > LUNA_RMESS_RUHE_ERPM) {
		luna_rmess_halt_ms = 0;
		return;
	}
	luna_rmess_halt_ms += ms_je_durchlauf;
	if (luna_rmess_auto && !luna_rmess_halt_gemessen &&
			luna_rmess_halt_ms >= LUNA_RMESS_HALTEN_MS &&
			(luna_rmess_total == 0 || luna_sek - luna_rmess_letzte_s >= LUNA_RMESS_AUTO_ABSTAND_S)) {
		luna_rmess_anfrage = true;
	}
	if (!luna_rmess_anfrage) {
		return;
	}
	luna_rmess_halt_gemessen = true;
	luna_rmess_letzte_s = luna_sek;
	luna_rmess_anfrage = false;
	systime_t t0 = chVTGetSystemTimeX();
	float r = 0.0;
	int fault = mcpwm_foc_measure_resistance(LUNA_RMESS_STROM, LUNA_RMESS_SAMPLES,
			true, &r);
	luna_rmess_t *e = &luna_rmess_buf[luna_rmess_total % LUNA_RMESS_N];
	e->t_s = luna_sek;
	e->r = r;
	e->t_fet = mc_interface_temp_fet_filtered();
	e->v_in = mc_interface_get_input_voltage_filtered();
	e->dauer_ms = (uint16_t)ST2MS(chVTTimeElapsedSinceX(t0));
	e->fault = (int8_t)fault;
	luna_rmess_total++;
	luna_rmess_nachlauf_ms = LUNA_RMESS_NACHLAUF_MS;
#ifdef HW_HAS_LUNA_SERIAL_DISPLAY
	if (fault == 0) {
		// (13.10.2026) kein Blinken mehr - das Licht bleibt, wie es ist.
		// (volker, 12.10.2026) Wicklungstemperatur 10 s im Tacho: 85 C -> "85".
		// Nur mit Bezugswert (luna_rmess kalib); im Stand eindeutig.
		float tw = app_adc_rmess_temp(r);
		if (tw > -900.0) {
			utils_truncate_number(&tw, 0.0, 98.0);   // 99+1 waere "00"
			luna_display_serial_tacho_zeigen((int)(tw + 0.5), 10);
		}
	}
#endif
}

// (volker, 04.10.2026) LICHTTASTE ALS BEDIENELEMENT. Aufruf aus dem
// Displaytreiber bei jedem Licht-Befehl (an/aus).
//   Bremse gezogen            -> 1-s-Takt an bzw. aus (auch in der Fahrt)
//   dazu Stand, Bremse >= 2 s -> Widerstandsmessung anfordern
//   ohne Bremse               -> nur Licht
static bool luna_brake;   // definiert weiter unten
void app_adc_licht_taste(bool an) {
	// (volker, 13.10.2026) Lichttaste ist nur noch Licht. Messung und
	// Aufzeichnung haengen nicht mehr daran (Bremse + Schiebehilfe).
	(void)an;
}

// NTC mit 10 kOhm nach 3,3 V. Laeuft mit 10 Hz aus dem Threadtakt.
static void luna_akku_temp_update(void) {
#ifdef LUNA_AKKU_NTC_ADC
	float v = ADC_VOLTS(LUNA_AKKU_NTC_ADC);
	if (v < 0.05 || v > (V_REG - 0.05)) {
		// Kurzschluss oder offen: kein Fuehler. Keine Ruecknahme.
		luna_akku_t_ok = false;
		luna_kaelte = 1.0;
		return;
	}
	float r = LUNA_AKKU_NTC_R_PULLUP * v / (V_REG - v);
	float t = 1.0 / (logf(r / 10000.0) / LUNA_AKKU_NTC_BETA + 1.0 / 298.15) - 273.15;
	if (!luna_akku_t_ok) {
		luna_akku_t = t;
	} else {
		luna_akku_t += 0.1 * (t - luna_akku_t);   // ~1 s Glaettung
	}
	luna_akku_t_ok = true;
	float f = utils_map(luna_akku_t, LUNA_AKKU_T_NULL, LUNA_AKKU_T_VOLL, 0.0, 1.0);
	utils_truncate_number(&f, 0.0, 1.0);
	luna_kaelte = f;
#endif
}

static void luna_seg_open(uint8_t modus) {
	luna_trace_seg_t *sg = &luna_trace_seg[luna_seg_total % LUNA_TRACE_SEG_N];
	sg->idx_start = luna_trace_total;
	sg->idx_end = luna_trace_total;
	sg->t_start = luna_sek;
	sg->modus = modus;
	luna_seg_total++;
	luna_trace_mark = true;         // sofort einen Punkt schreiben
}
static float luna_soll_A = 0.0;     // von den Steuerzweigen gesetzt

// --- Entprellung des Bremseingangs -----------------------------------
// Messung vom 20.09.: Der Bremszustand sprang im 100-ms-Takt an und aus,
// obwohl der Hebel nicht gezogen war. In luna_log erschien eine einzige
// Bremsung als vier Ereignisse, die Schiebehilfe brach zehnmal je Sekunde
// ein, und der Antrieb ruckelte - alles derselbe Fehler.
//
// Zwei Ursachen kommen zusammen: Der interne Pull-up hat einige zehn
// Kiloohm und haengt an einem langen Kabel neben den Phasenleitungen. Und
// das zyklische palSetPadMode() schreibt mehrere Register nacheinander -
// ist der Pull-up dabei kurz inaktiv, schwebt der Eingang und liest 0,
// was bei active low "gebremst" bedeutet.
//
// Der Zustand wird deshalb erst uebernommen, wenn er mehrere Durchlaeufe
// hintereinander gleich ist. Bei 500 Hz sind das 10 ms - fuer den Fahrer
// unmerklich, gegen Stoerimpulse wirksam. Symmetrisch, damit auch ein
// kurzzeitiges falsches "geloest" abgefangen wird.
#define LUNA_BRAKE_DEBOUNCE     5
static bool luna_brake = false;          // entprellter Zustand
static uint32_t luna_brake_edges = 0;    // Flanken nach der Entprellung
static uint32_t luna_brake_glitches = 0; // verworfene Stoerimpulse
// Merker fuer die Aufzeichnung: gab es seit der letzten Abtastung einen
// Stoerimpuls? So laesst sich im Verlauf sehen, OB sie gehaeuft bei hohem
// Motorstrom auftreten (Einstrahlung von den Phasenleitungen) oder
// gleichmaessig verteilt (dann eher der Pinmodus-Takt).
static volatile bool luna_brake_glitch_seit_abtastung = false;
// (volker, 26.09.) MESSUNG DER LOESEVERZOEGERUNG.
// Beschwerde nach der Fahrt: die Reku bleibt nach dem Loslassen zu lange
// aktiv. Die Abfallrampe erklaert das nicht - sie braucht 0,08 s. Verdacht:
// die Entprellung braucht fuenf ungestoerte Abtastungen in Folge, und genau
// beim Loesen fliesst noch der volle Bremsstrom, also stoert es am meisten.
// Gemessen wird die Zeit vom ersten "Hebel los" am Pin bis zum Umschalten
// des entprellten Zustands. Drei Abtastungen mit "Hebel gezogen" brechen die
// Messung ab, ein einzelner Stoerimpuls also nicht.
static volatile uint32_t luna_rel_ms_max = 0;
static volatile uint32_t luna_rel_ms_last = 0;
static volatile uint32_t luna_rel_n = 0;
static volatile uint32_t luna_rel_ms_summe = 0;

// (volker, 26.09.) SPANNUNGSECKWERTE.
// Die kleinste Spannung UNTER ANTRIEB und die groesste WAEHREND DER
// REKUPERATION, jeweils mit dem Strom, der dabei floss. Zwei Lastpunkte
// also, und daraus der Innenwiderstand von Akku samt Verkabelung.
// Ruhespannung zaehlt absichtlich nicht mit - sonst waere das Maximum immer
// der Wert vom Einschalten und sagte nichts.
// Das Maximum bei Rekuperation ist ausserdem die Zahl, die beim BMS
// zaehlt: dort entscheidet sich, ob der Ladeschutz anspricht.
#define LUNA_V_LAST_A       0.5f
static volatile float luna_v_min = 0.0;      // 0 = noch nichts gesehen
static volatile float luna_v_min_i = 0.0;
static volatile float luna_v_max = 0.0;
static volatile float luna_v_max_i = 0.0;
// ----------------------------------------------------------------------

// Stillstandsgrenze fuer die Anfahrsperre. 200 ERPM sind rund 1,2 km/h -
// darunter gilt das Rad als stehend.
#define LUNA_STANDSTILL_ERPM    200.0f
// Zaehlt, wie oft die Sperre gegriffen hat. Ueber luna_pins ablesbar:
// steigt der Wert im Stand, will etwas Strom anfordern, das nicht darf.
static volatile uint32_t luna_block_count = 0;
// ----------------------------------------------------------------------

#define LUNA_LOG_EVENTS 32
typedef struct {
	float t_start;      // s seit Boot
	float dur;          // s
	float scaling;      // Displaystufe als Anteil 0..1
	float kmh_start;
	float kmh_end;
	float i_in_peak;    // A, negativ
	float i_mot_peak;   // A, negativ
	float wh_start;
	float wh_end;
} luna_brake_event_t;

static luna_brake_event_t luna_log_buf[LUNA_LOG_EVENTS];
static int luna_log_count = 0;      // insgesamt aufgezeichnet
static bool luna_log_open = false;  // gerade ein Vorgang offen?
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
static volatile float adc2_override = 0.0;
static volatile bool use_rx_tx_as_buttons = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile int adc_detached = 0;
static volatile bool buttons_detached = false;
static volatile bool rev_override = false;
static volatile bool cc_override = false;
static volatile bool range_ok = true;

void app_adc_configure(adc_config *conf) {
	if (!buttons_detached && (((conf->buttons >> 0) & 1) || CTRL_USES_BUTTON(conf->ctrl_type))) {
		if (use_rx_tx_as_buttons) {
			palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
			palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);
		} else {
			palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);
		}
	}

	config = *conf;
	ms_without_power = 0.0;
}

void app_adc_start(bool use_rx_tx) {
#ifdef HW_ADC_EXT_GPIO
	palSetPadMode(HW_ADC_EXT_GPIO, HW_ADC_EXT_PIN, PAL_MODE_INPUT_ANALOG);
#endif
#ifdef HW_ADC_EXT2_GPIO
	palSetPadMode(HW_ADC_EXT2_GPIO, HW_ADC_EXT2_PIN, PAL_MODE_INPUT_ANALOG);
#endif

	if (buttons_detached) {
		use_rx_tx_as_buttons = false;
	} else {
		use_rx_tx_as_buttons = use_rx_tx;
	}

	stop_now = false;
	chThdCreateStatic(adc_thread_wa, sizeof(adc_thread_wa), NORMALPRIO, adc_thread, NULL);
}

void app_adc_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

float app_adc_get_decoded_level(void) {
	return decoded_level;
}

float app_adc_get_voltage(void) {
	return read_voltage;
}

// --- Diagnose fuer den Terminalbefehl luna_pins -----------------------
// ms_without_power und luna_regen_ramp sind statisch; ohne diese Zugriffe
// laesst sich von aussen nicht sehen, warum das Tor in Zeile ~511 zu ist.
void app_adc_trace_set(bool on) {
	// Nur den Zustand merken. Die Aufzeichnung erkennt den Wechsel selbst
	// und beginnt dann eine neue Sequenz.
	luna_trace_on = on;
}

void app_adc_trace_clear(void) {
	luna_trace_on = false;
	luna_trace_mark = false;
	luna_trace_total = 0;
	luna_seg_total = 0;
}

bool app_adc_trace_running(void) {
	return luna_trace_on;
}

bool app_adc_brake_state(void) {
	return luna_brake;
}

uint32_t app_adc_brake_edges(void) {
	return luna_brake_edges;
}

uint32_t app_adc_brake_glitches(void) {
	return luna_brake_glitches;
}

int app_adc_rmess_count(void) {
	return luna_rmess_total < LUNA_RMESS_N ? (int)luna_rmess_total : LUNA_RMESS_N;
}

// Eintrag i, 0 = aeltester noch vorhandener.
bool app_adc_rmess_get(int i, uint32_t *t_s, float *r, float *t_fet, float *v_in,
		int *dauer_ms, int *fault) {
	int n = app_adc_rmess_count();
	if (i < 0 || i >= n) {
		return false;
	}
	uint32_t base = luna_rmess_total > LUNA_RMESS_N ? luna_rmess_total - LUNA_RMESS_N : 0;
	luna_rmess_t *e = &luna_rmess_buf[(base + i) % LUNA_RMESS_N];
	*t_s = e->t_s;
	*r = e->r;
	*t_fet = e->t_fet;
	*v_in = e->v_in;
	*dauer_ms = e->dauer_ms;
	*fault = e->fault;
	return true;
}

bool app_adc_rmess_ref(float *r_ref, float *t_ref) {
	*r_ref = luna_rmess_r_ref;
	*t_ref = luna_rmess_t_ref;
	return luna_rmess_ref_ok;
}

// Bezug aus dem Mittel der letzten bis zu 5 guten Messungen (Widerstand
// und Endstufentemperatur). Nur nach langem Stehen: Wicklung = Endstufe =
// Umgebung. Gibt die Zahl der gemittelten Messungen zurueck, 0 = keine.
int app_adc_rmess_kalib(void) {
	int n = app_adc_rmess_count();
	float sr = 0.0, st = 0.0;
	int k = 0;
	for (int i = n - 1; i >= 0 && k < 5; i--) {
		uint32_t t; float r, tf, v; int d, f;
		if (app_adc_rmess_get(i, &t, &r, &tf, &v, &d, &f) && f == 0 && r > 0.001) {
			sr += r;
			st += tf;
			k++;
		}
	}
	if (k == 0) {
		return 0;
	}
	luna_rmess_r_ref = sr / (float)k;
	luna_rmess_t_ref = st / (float)k;
	luna_rmess_ref_ok = true;
	return k;
}

bool app_adc_rmess_auto(void) {
	return luna_rmess_auto;
}

float app_adc_rmess_temp(float r) {
	if (!luna_rmess_ref_ok || luna_rmess_r_ref <= 0.0) {
		return -999.0;
	}
	return luna_rmess_t_ref + (r / luna_rmess_r_ref - 1.0) / LUNA_RMESS_ALPHA;
}

void app_adc_akku_temp(float *t, bool *ok, float *faktor, bool *eingebaut) {
	*t = luna_akku_t;
	*ok = luna_akku_t_ok;
	*faktor = luna_kaelte;
#ifdef LUNA_AKKU_NTC_ADC
	*eingebaut = true;
#else
	*eingebaut = false;
#endif
}

void app_adc_volt_stats(float *v_min, float *i_min, float *v_max, float *i_max) {
	*v_min = luna_v_min;
	*i_min = luna_v_min_i;
	*v_max = luna_v_max;
	*i_max = luna_v_max_i;
}

void app_adc_brake_release_stats(uint32_t *letzte_ms, uint32_t *max_ms,
		uint32_t *mittel_ms, uint32_t *n) {
	*letzte_ms = luna_rel_ms_last;
	*max_ms = luna_rel_ms_max;
	*n = luna_rel_n;
	*mittel_ms = luna_rel_n > 0 ? luna_rel_ms_summe / luna_rel_n : 0;
}

uint32_t app_adc_block_count(void) {
	return luna_block_count;
}

int app_adc_trace_capacity(void) {
	return LUNA_TRACE_N;
}

int app_adc_trace_count(void) {
	return luna_trace_total < LUNA_TRACE_N ? (int)luna_trace_total : LUNA_TRACE_N;
}

// Kleinster Index, der noch im Ring steht. Alles darunter ist ueberschrieben.
uint32_t app_adc_trace_oldest(void) {
	return luna_trace_total > LUNA_TRACE_N ? luna_trace_total - LUNA_TRACE_N : 0;
}

int app_adc_trace_seg_count(void) {
	return luna_seg_total < LUNA_TRACE_SEG_N ? (int)luna_seg_total : LUNA_TRACE_SEG_N;
}

// Sequenz i, 0 = aelteste noch bekannte. von/bis sind auf das beschnitten,
// was der Ring noch hat; ganz steht fuer "Anfang nicht ueberschrieben".
bool app_adc_trace_seg_get(int i, uint32_t *von, uint32_t *bis, float *t_start,
		bool *ganz, int *modus) {
	int n = app_adc_trace_seg_count();
	if (i < 0 || i >= n) {
		return false;
	}
	uint32_t base = luna_seg_total > LUNA_TRACE_SEG_N ?
			luna_seg_total - LUNA_TRACE_SEG_N : 0;
	luna_trace_seg_t *sg = &luna_trace_seg[(base + i) % LUNA_TRACE_SEG_N];
	uint32_t alt = app_adc_trace_oldest();
	uint32_t a = sg->idx_start;
	uint32_t b = sg->idx_end;
	*ganz = (a >= alt);
	if (a < alt) {
		a = alt;
	}
	*von = a;
	*bis = b;
	*t_start = (float)sg->t_start;
	*modus = sg->modus;
	return b > a;
}

// Punkt am fortlaufenden Index abs. t in Sekunden seit Start des Controllers.
bool app_adc_trace_get(uint32_t abs, float *t, float *imot,
		float *iin, float *kmh, float *vin, float *prpm, float *soll,
		float *torq, float *tfet, int *flags) {
	if (abs < app_adc_trace_oldest() || abs >= luna_trace_total) {
		return false;
	}
	luna_trace_t *e = &luna_trace_buf[abs % LUNA_TRACE_N];
	*t = (float)e->t_s;
	*imot = (float)e->i_mot_dA / 10.0;
	*iin = (float)e->i_in_dA / 10.0;
	*vin = (float)e->v_in_cV / 100.0;
	*prpm = (float)e->pedal_rpm;
	*soll = (float)e->soll_A;
	*torq = (float)e->torque_pct;
	*flags = e->flags;
	*kmh = (float)e->kmh_2 / 2.0;
	*tfet = (float)e->temp_fet;
	return true;
}

int app_adc_log_count(void) {
	return luna_log_count;
}

// Liefert Ereignis i (0 = aeltestes im Puffer). Gibt false, wenn leer.
bool app_adc_log_get(int i, float *t, float *dur, float *scal,
		float *kmh0, float *kmh1, float *iin, float *imot, float *wh) {
	int n = luna_log_count < LUNA_LOG_EVENTS ? luna_log_count : LUNA_LOG_EVENTS;
	if (i < 0 || i >= n) {
		return false;
	}
	int base = luna_log_count <= LUNA_LOG_EVENTS ? 0 : luna_log_count % LUNA_LOG_EVENTS;
	luna_brake_event_t *e = &luna_log_buf[(base + i) % LUNA_LOG_EVENTS];
	*t = e->t_start; *dur = e->dur; *scal = e->scaling;
	*kmh0 = e->kmh_start; *kmh1 = e->kmh_end;
	*iin = e->i_in_peak; *imot = e->i_mot_peak;
	*wh = e->wh_end - e->wh_start;
	return true;
}

void app_adc_log_clear(void) {
	luna_log_count = 0;
	luna_log_open = false;
}

float app_adc_get_ms_without_power(void) {
	return ms_without_power;
}

float app_adc_get_regen_ramp(void) {
	return luna_regen_ramp;
}

float app_adc_get_regen_amps(void) {
	return luna_regen_amps;
}
// ----------------------------------------------------------------------

void app_adc_set_regen_amps(float amps) {
	utils_truncate_number(&amps, 0.0, 200.0);
	luna_regen_amps = amps;
}

void app_adc_set_walk_amps(float amps) {
	utils_truncate_number(&amps, 0.0, 100.0);
	luna_walk_amps = amps;
}

float app_adc_get_walk_amps(void) {
	return luna_walk_amps;
}

void app_adc_set_walk_assist(bool on) {
	luna_walk_assist = on;
}

bool app_adc_get_walk_assist(void) {
	return luna_walk_assist;
}

float app_adc_get_decoded_level2(void) {
	return decoded_level2;
}

float app_adc_get_voltage2(void) {
	return read_voltage2;
}

void app_adc_detach_adc(int detach) {
	adc_detached = detach;
	timeout_reset();
}

void app_adc_adc1_override(float val) {
	utils_truncate_number(&val, 0, 3.3);
	adc1_override = val;
	timeout_reset();
}

void app_adc_adc2_override(float val) {
	utils_truncate_number(&val, 0, 3.3);
	adc2_override = val;
	timeout_reset();
}

void app_adc_detach_buttons(bool state) {
	buttons_detached = state;
	timeout_reset();
}

void app_adc_rev_override(bool state) {
	rev_override = state;
	timeout_reset();
}

void app_adc_cc_override(bool state) {
	cc_override = state;
	timeout_reset();
}

bool app_adc_range_ok(void) {
	return range_ok;
}

static THD_FUNCTION(adc_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_ADC");
	is_running = true;

	// (volker, 10.10.2026) Bezug der Wicklungstemperatur je Rad aus dem
	// Profil. Der Raddurchmesser kommt aus der Motorkonfiguration.
	if (luna_profil()->r_ref > 0.0) {
		luna_rmess_r_ref = luna_profil()->r_ref;
		luna_rmess_t_ref = luna_profil()->t_ref;
		luna_rmess_ref_ok = true;
	}

	for(;;) {
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		// --- Bremseingang entprellen ------------------------------------
#ifdef HW_HAS_BRAKE_SENSOR
		{
			static int cnt = 0;
			bool roh;
			if (luna_profil()->bremse_adc1) {
				// Bremse am Gaseingang (Profil je Controller, hw_75_100_V2.c).
				// Schwelle mit Hysterese, danach dieselbe Entprellung.
				// Unterbrochene Leitung -> Pull-up -> "los": dann bremst nur
				// noch die Mechanik am selben Hebel, keine ploetzliche Reku.
				static bool adc_zu = false;
				float v_brems = ADC_VOLTS(ADC_IND_EXT);
				if (v_brems < LUNA_BREMSE_ADC1_ZU_V) {
					adc_zu = true;
				} else if (v_brems > LUNA_BREMSE_ADC1_AUF_V) {
					adc_zu = false;
				}
				roh = adc_zu;
			} else {
				roh = !palReadPad(HW_BRAKE_SENSOR_PORT, HW_BRAKE_SENSOR_PIN);
			}

			// Uhr fuer die Loeseverzoegerung
			static uint32_t rel_samples = 0;   // 0 = keine Messung offen
			static int rel_gegen = 0;
			if (luna_brake && !roh) {
				rel_gegen = 0;
				if (rel_samples == 0) {
					rel_samples = 1;
				} else {
					rel_samples++;
				}
			} else if (luna_brake && roh && rel_samples > 0) {
				if (++rel_gegen >= 3) {
					rel_samples = 0;       // er bremst weiter
				} else {
					rel_samples++;
				}
			}

			if (roh == luna_brake) {
				if (cnt > 0) {
					luna_brake_glitches++;   // begonnener Wechsel, der verfiel
					luna_brake_glitch_seit_abtastung = true;
				}
				cnt = 0;
			} else if (++cnt >= LUNA_BRAKE_DEBOUNCE) {
				luna_brake = roh;
				luna_brake_edges++;
				cnt = 0;
				if (!luna_brake && rel_samples > 0) {
					// Loesen erkannt. 500 Hz Threadtakt, also 2 ms je Abtastung.
					uint32_t ms = rel_samples * 2;
					luna_rel_ms_last = ms;
					if (ms > luna_rel_ms_max) {
						luna_rel_ms_max = ms;
					}
					luna_rel_ms_summe += ms;
					luna_rel_n++;
				}
				rel_samples = 0;
				rel_gegen = 0;
			}
		}
#endif

		// --- Spannungseckwerte -----------------------------------------
		{
			float v = mc_interface_get_input_voltage_filtered();
			float i = mc_interface_get_tot_current_in_filtered();
			if (i > LUNA_V_LAST_A && (luna_v_min == 0.0 || v < luna_v_min)) {
				luna_v_min = v;
				luna_v_min_i = i;
			}
			if (i < -LUNA_V_LAST_A && v > luna_v_max) {
				luna_v_max = v;
				luna_v_max_i = i;
			}
		}

		// --- Akkutemperatur fuer die Reku bei Kaelte, 10 Hz -------------
		{
			static int t_div = 0;
			if (++t_div >= 50) {
				t_div = 0;
				luna_akku_temp_update();
			}
		}

		// --- Verlaufsaufzeichnung ---------------------------------------
		// Hier oben, damit sie in jedem Zweig laeuft - der Bremszweig
		// weiter unten springt per continue und wuerde sonst fehlen.
		// luna_soll_A stammt aus dem vorigen Durchlauf; bei hoechstens 1 Hz
		// Abtastung und 500 Hz Threadtakt ist der Versatz belanglos.
		{
			// Sekundenuhr aus der Systemzeit. Die Differenz ist ueberlauffest,
			// die Uhr laeuft also auch nach Tagen am Stueck richtig.
			static systime_t sek_last = 0;
			static uint32_t sek_rest = 0;
			systime_t sek_now = chVTGetSystemTimeX();
			sek_rest += (uint32_t)(systime_t)(sek_now - sek_last);
			sek_last = sek_now;
			while (sek_rest >= CH_CFG_ST_FREQUENCY) {
				sek_rest -= CH_CFG_ST_FREQUENCY;
				luna_sek++;
			}

#ifdef HW_HAS_LUNA_SERIAL_DISPLAY
			bool display_da = luna_display_serial_connected();
#else
			bool display_da = true;
#endif
			// (volker, 13.10.2026) Aufgezeichnet wird nur bei Display an, alle
			// 10 s ein Punkt aus MITTELWERTEN ueber alle Durchlaeufe; der
			// Motorstrom als Effektivwert (fuer die Waerme zaehlt I^2), dazu
			// der Spitzenstrom. Kein 1-s-Takt und kein Takt bei Display aus mehr.
			static float a_i = 0, a_i2 = 0, a_iin = 0, a_v = 0, a_kmh = 0,
					a_prpm = 0, a_tq = 0, a_peak = 0;
			static uint32_t a_n = 0;
			static uint8_t a_f = 0;
			static bool display_alt = false;
			static uint32_t sek_letzte = 0;
			if (!display_da) {
				display_alt = false;
			} else {
				if (!display_alt || luna_seg_total == 0) {
					luna_seg_open(LUNA_MODUS_DISPLAY);   // neue Sequenz, sofort ein Punkt
					display_alt = true;
					a_i = a_i2 = a_iin = a_v = a_kmh = a_prpm = a_tq = a_peak = 0;
					a_n = 0;
				}
				{
					float i = mc_interface_get_tot_current_filtered();
					a_i += i; a_i2 += i * i;
					if (fabsf(i) > a_peak) { a_peak = fabsf(i); }
					a_iin += mc_interface_get_tot_current_in_filtered();
					a_v += mc_interface_get_input_voltage_filtered();
					a_kmh += mc_interface_get_speed() * 3.6;
					a_prpm += app_pas_get_pedal_rpm();
					a_tq += app_adc_get_decoded_level2() * 100.0;
					a_n++;
#ifdef HW_HAS_BRAKE_SENSOR
					if (luna_brake) { a_f |= 1; }
#endif
					if (luna_walk_assist) { a_f |= 2; }
					if (luna_regen_ramp > 0.1) { a_f |= 4; }
				}
			}
			if (display_da && (luna_trace_mark || (luna_sek - sek_letzte) >= LUNA_TRACE_T_DISPL)) {
				sek_letzte = luna_sek;
				{
					float n = a_n > 0 ? (float)a_n : 1.0;
					luna_trace_t *e = &luna_trace_buf[luna_trace_total % LUNA_TRACE_N];
					e->t_s = luna_sek;
					float ieff = sqrtf(a_i2 / n);
					if (a_i < 0.0) { ieff = -ieff; }
					e->i_mot_dA = (int16_t)(ieff * 10.0);
					e->i_in_dA = (int16_t)(a_iin / n * 10.0);
					e->v_in_cV = (uint16_t)((a_n > 0 ? a_v / n : mc_interface_get_input_voltage_filtered()) * 100.0);
					float kmh2 = a_kmh / n * 2.0;
					e->kmh_2 = (uint8_t)(kmh2 < 0.0 ? 0.0 : (kmh2 > 255.0 ? 255.0 : kmh2));
					float tf = mc_interface_temp_fet_filtered();
					e->temp_fet = (uint8_t)(tf < 0.0 ? 0.0 : (tf > 255.0 ? 255.0 : tf));
					float prpm = a_prpm / n;
					e->pedal_rpm = (uint8_t)(prpm < 0.0 ? 0.0 : (prpm > 255.0 ? 255.0 : prpm));
					e->soll_A = (uint8_t)(a_peak > 255.0 ? 255.0 : a_peak);
					float tq = a_tq / n;
					e->torque_pct = (uint8_t)(tq < 0.0 ? 0.0 : (tq > 100.0 ? 100.0 : tq));
					uint8_t f = a_f;
					a_i = a_i2 = a_iin = a_v = a_kmh = a_prpm = a_tq = a_peak = 0;
					a_n = 0;
					a_f = 0;
					if (luna_brake_glitch_seit_abtastung) {
						f |= 8;
						luna_brake_glitch_seit_abtastung = false;
					}
					if (luna_trace_mark) {
						f |= 16;
						luna_trace_mark = false;
					}
					e->flags = f;
					luna_trace_total++;
					// Die laufende Sequenz bis hierher fortschreiben.
					luna_trace_seg[(luna_seg_total - 1) % LUNA_TRACE_SEG_N].idx_end =
							luna_trace_total;
				}
			}
		}

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE && config.safe_start != SAFE_START_NO_FAULT) {
			ms_without_power = 0;
		}

		// Read the external ADC pin voltage
		float pwr = ADC_VOLTS(ADC_IND_EXT);
		// (volker, 07.10.2026) Gas bei allen Controllern aus: ADC1 ist beim
		// weissen Rad und beim Testcontroller die Bremse, und kein Rad hat
		// einen Gasgriff. Immer "Gasgriff in Ruhe". Fuer die Drehgriff-
		// Versuche gezielt je Profil wieder freigeben.
		pwr = config.voltage_start;

		// Override pwr value, when used from LISP
		if (adc_detached == 1 || adc_detached == 2) {
			pwr = adc1_override;
		}

		// Read voltage and range check
		static float read_filter = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(read_filter, pwr, FILTER_SAMPLES);

		if (config.use_filter) {
			read_voltage = read_filter;
		} else {
			read_voltage = pwr;
		}

		range_ok = read_voltage >= config.voltage_min && read_voltage <= config.voltage_max;

		// Map the read voltage
		switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		case ADC_CTRL_TYPE_PID_REV_CENTER:
			// Mapping with respect to center voltage
			if (pwr < config.voltage_center) {
				pwr = utils_map(pwr, config.voltage_start,
						config.voltage_center, 0.0, 0.5);
			} else {
				pwr = utils_map(pwr, config.voltage_center,
						config.voltage_end, 0.5, 1.0);
			}
			break;

		default:
			// Linear mapping between the start and end voltage
			pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0, 1.0);
			break;
		}

		// Optionally apply a filter
		static float pwr_filter = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(pwr_filter, pwr, FILTER_SAMPLES);

		if (config.use_filter) {
			pwr = pwr_filter;
		}

		// Truncate the read voltage
		utils_truncate_number(&pwr, 0.0, 1.0);

		// Optionally invert the read voltage
		if (config.voltage_inverted) {
			pwr = 1.0 - pwr;
		}

		decoded_level = pwr;

		// Read the external ADC pin and convert the value to a voltage.
#ifdef ADC_IND_EXT2
		float brake = ADC_VOLTS(ADC_IND_EXT2);
#else
		float brake = 0.0;
#endif

#ifdef HW_HAS_BRAKE_OVERRIDE
		hw_brake_override(&brake);
#endif

		// Override brake value, when used from LISP
		if (adc_detached == 1 || adc_detached == 3) {
			brake = adc2_override;
		}

		read_voltage2 = brake;

		// Optionally apply a filter
		static float filter_val_2 = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(filter_val_2, brake, FILTER_SAMPLES);

		if (config.use_filter) {
			brake = filter_val_2;
		}

		// Map and truncate the read voltage
		brake = utils_map(brake, config.voltage2_start, config.voltage2_end, 0.0, 1.0);
		utils_truncate_number(&brake, 0.0, 1.0);

		// Optionally invert the read voltage
		if (config.voltage2_inverted) {
			brake = 1.0 - brake;
		}

		decoded_level2 = brake;

		// Read the button pins
		bool cc_button = false;
		bool rev_button = false;
		if (use_rx_tx_as_buttons) {
			cc_button = !palReadPad(HW_UART_TX_PORT, HW_UART_TX_PIN);
			if ((config.buttons >> 1) & 1) {
				cc_button = !cc_button;
			}
			rev_button = !palReadPad(HW_UART_RX_PORT, HW_UART_RX_PIN);
			if ((config.buttons >> 2) & 1) {
				rev_button = !rev_button;
			}
		} else {
			// When only one button input is available, use it differently depending on the control mode
			if (config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON ||
                    config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER ||
					config.ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON ||
					config.ctrl_type == ADC_CTRL_TYPE_DUTY_REV_BUTTON ||
					config.ctrl_type == ADC_CTRL_TYPE_PID_REV_BUTTON) {
				rev_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
				if ((config.buttons >> 2) & 1) {
					rev_button = !rev_button;
				}
			} else {
				cc_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
				if ((config.buttons >> 1) & 1) {
					cc_button = !cc_button;
				}
			}
		}

		// Override button values, when used from LISP
		if (buttons_detached) {
			cc_button = cc_override;
			rev_button = rev_override;
			if ((config.buttons >> 1) & 1) {
				cc_button = !cc_button;
			}
			if ((config.buttons >> 2) & 1) {
				rev_button = !rev_button;
			}
		}

		if (!((config.buttons >> 0) & 1)) {
			cc_button = false;
		}

		// All pins and buttons are still decoded for debugging, even
		// when output is disabled.
		if (app_is_output_disabled()) {
			continue;
		}

		if (adc_detached && timeout_has_timeout()) {
			continue;
		}

		switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		case ADC_CTRL_TYPE_PID_REV_CENTER:
			// Scale the voltage and set 0 at the center
			pwr *= 2.0;
			pwr -= 1.0;
			break;

		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
			pwr -= brake;
			break;

		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
		case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
		case ADC_CTRL_TYPE_PID_REV_BUTTON:
			// Invert the voltage if the button is pressed
			if (rev_button) {
				pwr = -pwr;
			}
			break;

		default:
			break;
		}

		// Apply deadband
		utils_deadband(&pwr, config.hyst, 1.0);

		// Apply throttle curve
		pwr = utils_throttle_curve(pwr, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		// Apply ramping
		static systime_t last_time = 0;
		static float pwr_ramp = 0.0;
		float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&pwr_ramp, pwr, ramp_step);
			last_time = chVTGetSystemTimeX();
			pwr = pwr_ramp;
		}

		// Bei gezogener Bremse den Antriebspfad auf null zwingen - auch den
		// Rampenspeicher. Sonst baut sich beim Treten waehrend des Bremsens
		// im Hintergrund Unterstuetzung auf, die im Moment des Loslassens
		// schlagartig anliegt. So beginnt sie nach dem Loslassen wieder bei
		// null und laeuft ueber ramp_time_pos hoch.
		// (volker, 11.10.2026) KONFIGURATIONSSPERRE (alle ausser s_pedelec):
		// Ohne im Flash gespeicherte Motorkonfiguration (Motor-Wizard in VESC
		// Tool) keine Unterstuetzung und keine Reku. Display zeigt Err 09.
		if (!LUNA_SPED && !conf_general_mcconf_gespeichert) {
			luna_soll_A = 0.0;
			luna_freigeben();
			timeout_reset();
			continue;
		}
		// Pedelec 25: Akkustrom auf 10 A begrenzen, auch wenn in VESC Tool
		// mehr eingetragen wurde (nur Arbeitsspeicher, etwa einmal je Sekunde).
		{
			static int p25_div = 0;
			if (!LUNA_SPED && ++p25_div >= (int)config.update_rate_hz) {
				p25_div = 0;
				if (mc_interface_get_configuration()->l_in_current_max > LUNA_P25_AKKU_A) {
					mc_configuration *mcc = mempools_alloc_mcconf();
					*mcc = *mc_interface_get_configuration();
					mcc->l_in_current_max = LUNA_P25_AKKU_A;
					mc_interface_set_configuration(mcc);
					mempools_free_mcconf(mcc);
				}
			}
		}
#ifdef HW_HAS_BRAKE_SENSOR
		if (luna_brake) {
			pwr_ramp = 0.0;
			pwr = 0.0;
		}
#endif

		float current_rel = 0.0;
		bool current_mode = false;
		bool current_mode_brake = false;
		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		const float rpm_now = mc_interface_get_rpm();
		bool send_duty = false;

		// Use the filtered and mapped voltage for control according to the configuration.
		switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT:
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
			current_mode = true;
			current_rel = pwr;

			if (fabsf(pwr) < 0.001) {
				ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
			}
			break;

        case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
//			current_mode = true;
//			if (pwr >= 0.0) {
//				// if pedal assist (PAS) thread is running, use the highest current command
//				if (app_pas_is_running()) {
//					pwr = utils_max_abs(pwr, app_pas_get_current_target_rel());
//				}
//				current_rel = pwr;
//			} else {
//				current_rel = fabsf(pwr);
//				current_mode_brake = true;
//			}

			if (pwr < 0.001) {
				ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
			}

			if ((config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC ||
			    config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER) && rev_button) {
				current_rel = -current_rel;
			}
			break;

		case ADC_CTRL_TYPE_DUTY:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
			if (fabsf(pwr) < 0.001) {
				ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
			}

			if (!(ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start)) {
				mc_interface_set_duty(utils_map(pwr, -1.0, 1.0, -mcconf->l_max_duty, mcconf->l_max_duty));
				send_duty = true;
			}
			break;

		case ADC_CTRL_TYPE_PID:
		case ADC_CTRL_TYPE_PID_REV_CENTER:
		case ADC_CTRL_TYPE_PID_REV_BUTTON:
			if ((pwr >= 0.0 && rpm_now > 0.0) || (pwr < 0.0 && rpm_now < 0.0)) {
				current_rel = pwr;
			} else {
				current_rel = pwr;
			}

			if (!(ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start)) {
				float speed = 0.0;
				if (pwr >= 0.0) {
					speed = pwr * mcconf->l_max_erpm;
				} else {
					speed = pwr * fabsf(mcconf->l_min_erpm);
				}

				mc_interface_set_pid_speed(speed);
				send_duty = true;
			}

			if (fabsf(pwr) < 0.001) {
				ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
			}
			break;

		default:
			continue;
		}

		// (volker) Bremshebel VOR dem safe-start-Tor behandeln.
		//
		// Das Tor unten prueft die Plausibilitaet des GASGRIFFS: solange
		// ms_without_power < 500 ist oder ADC1 ausserhalb des Fensters liegt,
		// springt es per continue ueber den gesamten Steuerblock - und damit
		// auch ueber die Rekuperation. Gemessen am 19.09.: Mit
		// Werkskonfiguration (ADC1 Start 0,9 V) liegt der ruhende Gasgriff
		// bei 1,455 V, das sind dauerhaft 26 % Anforderung, der Zaehler kommt
		// nie ueber null und die Bremsfunktion war tot.
		//
		// Ein Bremsbefehl darf nicht davon abhaengen, ob der Gasgriff einen
		// sinnvollen Wert liefert. Deshalb steht die Reku jetzt davor.
#ifdef HW_HAS_BRAKE_SENSOR
		if (luna_brake) {
			// (volker, 01.10.2026) STILLSTAND MIT GEZOGENER BREMSE: Motor frei.
			// Unter LUNA_STANDSTILL_ERPM (~1,2 km/h) haelt die mechanische
			// Bremse am selben Hebel. Ein Bremsstrom im Stand heizt nur die
			// Wicklung, zieht Strom und brummt - und wuerde die
			// Abkuehlmessung mit festgebundenem Hebel verfaelschen. Bei
			// echter Drehzahl ist im Stand ohnehin kaum Strom geflossen.
			if (luna_rmess_nachlauf_ms > 0) {
				luna_rmess_nachlauf_ms -= 1000 / config.update_rate_hz;
			}
			if (fabsf(mc_interface_get_rpm()) < LUNA_STANDSTILL_ERPM ||
					luna_rmess_nachlauf_ms > 0) {
				luna_regen_ramp = 0.0;
				luna_soll_A = 0.0;
				mc_interface_release_motor();   // Hebel gezogen: immer frei
				luna_app_treibt = false;
				timeout_reset();
				pwr_ramp = 0.0;
				// Bremse >= 2 s im Stand + Schiebehilfe: Automatik umschalten.
				// Gemessen wird erst nach dem Loslassen der Taste.
				if (luna_walk_assist && !luna_rmess_walk_alt &&
						luna_rmess_halt_ms >= LUNA_RMESS_HALTEN_MS) {
					luna_rmess_auto = !luna_rmess_auto;
					if (luna_rmess_auto) {
						luna_rmess_anfrage = true;
					}
				}
				luna_rmess_walk_alt = luna_walk_assist;
				if (!luna_walk_assist) {
					luna_rmess_im_halt(1000 / config.update_rate_hz);
				}
				continue;
			}
			// Rollt das Rad mit gezogenem Hebel: Haltezeit neu zaehlen, aber
			// nicht vergessen, dass in diesem Halt schon gemessen wurde.
			luna_rmess_halt_ms = 0;

			// Rampe jetzt in Ampere je Sekunde: der volle Stufenwert soll
			// unabhaengig von seiner Hoehe in LUNA_REGEN_RAMP_UP_S stehen.
			// (volker, 26.09.) Bezug ist das Groessere von Soll und Ist.
			// Vorher stand hier nur der Sollwert: faellt der waehrend des
			// Bremsens (Stufenwechsel, Display geht an), wurde der Schritt
			// winzig und das Bremsmoment schlich ueber viele Sekunden weg,
			// statt in 0,3 s sauber umzusetzen.
			float ref = (luna_regen_amps > luna_regen_ramp)
					? luna_regen_amps : luna_regen_ramp;
			if (ref < 1.0) { ref = 1.0; }
			const float step = ref / LUNA_REGEN_RAMP_UP_S / (float)config.update_rate_hz;
			// Kaeltefaktor: bei kaltem Akku weniger Reku (siehe LUNA_AKKU_T_*).
			utils_step_towards(&luna_regen_ramp, luna_regen_amps * luna_kaelte, step);
			mc_interface_set_brake_current(luna_regen_ramp);
			luna_app_treibt = true;
			luna_soll_A = -luna_regen_ramp;
			timeout_reset();
			pwr_ramp = 0.0;     // Antrieb beginnt nach dem Loesen wieder bei null

			// --- Mitschnitt ---
			// (volker, 26.09.) Laeuft nur noch bei scharfgeschalteter
			// Aufnahme, also wenn der Lichttaster an ist. Der Fehler, fuer
			// den dieses Protokoll gebaut wurde, ist gefunden; im Alltag
			// soll der Bremspfad nichts weiter tun als bremsen. Zum
			// Nachmessen Licht ein, Sequenz fahren, "luna_log" lesen.
			if (luna_trace_on) {
				float now = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
				float kmh = mc_interface_get_speed() * 3.6;
				float i_in = mc_interface_get_tot_current_in_filtered();
				float i_mot = mc_interface_get_tot_current_filtered();
				luna_brake_event_t *e = &luna_log_buf[luna_log_count % LUNA_LOG_EVENTS];

				if (!luna_log_open) {
					e->t_start = now;
					e->scaling = luna_regen_amps;
					e->kmh_start = kmh;
					e->i_in_peak = 0.0;
					e->i_mot_peak = 0.0;
					e->wh_start = mc_interface_get_watt_hours_charged(false);
					luna_log_open = true;
				}
				e->dur = now - e->t_start;
				e->kmh_end = kmh;
				e->wh_end = mc_interface_get_watt_hours_charged(false);
				if (i_in < e->i_in_peak) {
					e->i_in_peak = i_in;
				}
				if (i_mot < e->i_mot_peak) {
					e->i_mot_peak = i_mot;
				}
			}

			continue;           // Antriebspfad ueberspringen, Bremse hat Vorrang
		} else {
			luna_rmess_reset_halt();
			// Vorgang abschliessen. Sehr kurze Zupfer verwerfen, die
			// verstopfen sonst den Puffer.
			if (luna_log_open) {
				luna_log_open = false;
				// Abschliessen auch dann, wenn die Aufnahme mitten im
				// Bremsvorgang abgeschaltet wurde - sonst bliebe der
				// Eintrag offen und der naechste ueberschreibt ihn.
				if (luna_log_buf[luna_log_count % LUNA_LOG_EVENTS].dur >= 0.3) {
					luna_log_count++;
				}
			}
			// (volker, 20.09.) Abfallrampe statt hartem Sprung.
			//
			// Vorher fiel die Rampe beim Loesen sofort auf null und der
			// Motor wurde im naechsten Durchlauf freigegeben - ein
			// Stromsprung von bis zu 60 A in einem Regeltakt. Der Strom in
			// der Motorinduktivitaet kommutiert dann ueber die Body-Dioden
			// ab, und das hoert man als kurzen Schlag.
			//
			// 0,08 s sind fuer den Fahrer weiterhin "sofort" - das Anlegen
			// laeuft mit 0,3 s fast viermal langsamer -, nehmen dem
			// Uebergang aber die Stufe.
			if (luna_regen_ramp > 0.1) {
				float ref_ab = (luna_regen_amps > luna_regen_ramp)
						? luna_regen_amps : luna_regen_ramp;
				if (ref_ab < 1.0) { ref_ab = 1.0; }
				const float step_ab = ref_ab / LUNA_REGEN_RAMP_DOWN_S / (float)config.update_rate_hz;
				utils_step_towards(&luna_regen_ramp, 0.0, step_ab);
				mc_interface_set_brake_current(luna_regen_ramp);
				luna_app_treibt = true;
				luna_soll_A = -luna_regen_ramp;
				timeout_reset();
				pwr_ramp = 0.0;
				continue;
			}
			luna_regen_ramp = 0.0;
		}
#endif

		// If safe start is enabled and the output has not been zero for long enough
		if ((ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start) || !range_ok) {
			static int pulses_without_power_before = 0;
			if (ms_without_power == pulses_without_power_before) {
				ms_without_power = 0;
			}
			pulses_without_power_before = ms_without_power;
			mc_interface_set_brake_current(timeout_get_brake_current());

			if (config.multi_esc) {
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current_brake(msg->id, timeout_get_brake_current());
					}
				}
			}

			continue;
		}

		// Reset timeout only when the ADC-app is not detached
		if (!adc_detached) {
			timeout_reset();
		}

		// If c is pressed and no throttle is used, maintain the current speed with PID control
		static bool was_pid = false;

		// Filter RPM to avoid glitches
		static float rpm_filtered = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(rpm_filtered, mc_interface_get_rpm(), RPM_FILTER_SAMPLES);

		if (current_mode && cc_button && fabsf(pwr) < 0.001) {
			static float pid_rpm = 0.0;

			if (!was_pid) {
				was_pid = true;
				pid_rpm = rpm_filtered;
			}

			mc_interface_set_pid_speed(pid_rpm);

			// Send the same duty cycle to the other controllers
			if (config.multi_esc) {
				float current = mc_interface_get_tot_current_directional_filtered();

				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current(msg->id, current);
					}
				}
			}

			continue;
		}

		was_pid = false;

		// Find lowest RPM (for traction control)
		float rpm_local = mc_interface_get_rpm();
		float rpm_lowest = rpm_local;
		if (config.multi_esc) {
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					float rpm_tmp = msg->rpm;

					if (fabsf(rpm_tmp) < fabsf(rpm_lowest)) {
						rpm_lowest = rpm_tmp;
					}
				}
			}
		}

		// Optionally send the duty cycles to the other ESCs seen on the CAN-bus
		if (send_duty && config.multi_esc) {
			float duty = mc_interface_get_duty_cycle_now();

			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					comm_can_set_duty(msg->id, duty);
				}
			}
		}
		current_mode = true;
		if (pwr >= 0.0) {
			// if pedal assist (PAS) thread is running, use the highest current command
			if (app_pas_is_running()) {
				pwr = utils_max_abs(pwr, app_pas_get_current_target_rel());
			}
			current_rel = pwr;
		}
		if (current_mode) {
			if (current_mode_brake) {
				mc_interface_set_brake_current_rel(current_rel);

				// Send brake command to all ESCs seen recently on the CAN bus
				if (config.multi_esc) {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							comm_can_set_current_brake_rel(msg->id, current_rel);
						}
					}
				}
			} else {
				float current_out = current_rel;
				bool is_reverse = false;
				if (current_out < 0.0) {
					is_reverse = true;
					current_out = -current_out;
					current_rel = -current_rel;
					rpm_local = -rpm_local;
					rpm_lowest = -rpm_lowest;
				}

				// Traction control
				if (config.multi_esc) {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							if (config.tc && config.tc_max_diff > 1.0) {
								float rpm_tmp = msg->rpm;
								if (is_reverse) {
									rpm_tmp = -rpm_tmp;
								}

								float diff = rpm_tmp - rpm_lowest;
                                if (diff < TC_DIFF_MAX_PASS) diff = 0;
                                if (diff > config.tc_max_diff) diff = config.tc_max_diff;
								current_out = utils_map(diff, 0.0, config.tc_max_diff, current_rel, 0.0);
							}

							if (is_reverse) {
								comm_can_set_current_rel(msg->id, -current_out);
							} else {
								comm_can_set_current_rel(msg->id, current_out);
							}
						}
					}

					if (config.tc) {
						float diff = rpm_local - rpm_lowest;
                        if (diff < TC_DIFF_MAX_PASS) diff = 0;
                        if (diff > config.tc_max_diff) diff = config.tc_max_diff;
						current_out = utils_map(diff, 0.0, config.tc_max_diff, current_rel, 0.0);
					}
				}
				{
					// Die Bremse ist hier nicht mehr zu pruefen - sie wird
					// oben vor dem safe-start-Tor behandelt und springt per
					// continue. Hier kommt nur an, wer nicht bremst.
					// Schiebehilfe. Das Flag faellt von selbst, sobald das
					// Display wieder eine normale Stufe schickt - die Taste
					// muss also gehalten werden.
					//
					// Volle Kraft bis LUNA_WALK_ERPM_FULL, dann linear auf
					// null bei LUNA_WALK_ERPM_ZERO.
					float walk_amps = 0.0;
					if (luna_walk_assist) {
						float erpm = fabsf(rpm_local);
						walk_amps = luna_walk_amps;
						if (erpm > LUNA_WALK_ERPM_FULL) {
							walk_amps = utils_map(erpm, LUNA_WALK_ERPM_FULL,
									LUNA_WALK_ERPM_ZERO, luna_walk_amps, 0.0);
							utils_truncate_number(&walk_amps, 0.0, luna_walk_amps);
						}
					}

					// (volker, 20.09.) Schiebehilfe und Tretunterstuetzung
					// ADDIEREN sich, statt dass nur der groessere Wert gilt.
					// Beim Anfahren im grossen Gang ist die Trittfrequenz
					// klein und die Tretunterstuetzung daher gering - dann
					// hilft der feste Anteil der Schiebehilfe spuerbar.
					// Die Summe wird von l_current_max gedeckelt.
					const volatile mc_configuration *mcc =
							mc_interface_get_configuration();
					float walk_rel = (mcc->l_current_max > 1.0)
							? (walk_amps / mcc->l_current_max) : 0.0;
					// Tempogrenze der Unterstuetzung (siehe RECHTLICHER RAHMEN).
					// Gilt auch fuer den Boost, der in current_out steckt.
					{
						float erpm = fabsf(rpm_local);
						float e_voll = LUNA_KMH2ERPM(LUNA_UNTERST_VOLL_KMH);
						float e_null = LUNA_KMH2ERPM(LUNA_UNTERST_NULL_KMH);
						if (erpm >= e_null) {
							current_out = 0.0;
						} else if (erpm > e_voll) {
							current_out *= utils_map(erpm, e_voll, e_null, 1.0, 0.0);
						}
					}
					float out = current_out + walk_rel;
					utils_truncate_number(&out, 0.0, 1.0);
					luna_soll_A = out * mcc->l_current_max;

					// (volker, 20.09.) Sperre gegen Anfahren im Stand.
					//
					// Am 20.09. zuckte das aufgebockte Rad und drehte sich
					// leicht vorwaerts; im Mitschnitt steht ein Stoss von
					// 6,4 A Motorstrom, obwohl weder getreten noch geschoben
					// wurde. Die Ursache ist nicht geklaert - dieses Rad hat
					// keinen Gasgriff, es DARF im Stand ohne Tritt gar kein
					// Antriebsstrom angefordert werden.
					//
					// Deshalb hier eine harte Bedingung statt einer Suche
					// nach der Quelle: Antrieb nur, wenn getreten wird, die
					// Schiebehilfe gedrueckt ist oder das Rad ohnehin schon
					// rollt. Was auch immer den Stoss ausgeloest hat, kommt
					// damit nicht mehr durch.
					if (!luna_walk_assist
							&& app_pas_get_pedal_rpm() < 1.0
							&& fabsf(rpm_local) < LUNA_STANDSTILL_ERPM) {
						luna_block_count++;
						luna_soll_A = 0.0;
						luna_freigeben();
					} else if (out < 0.001) {
						// Kein Anforderung: Motor FREIGEBEN, nicht auf null
						// regeln.
						//
						// mcpwm_foc_set_current(0) setzt zwar iq_set auf null,
						// kehrt aber vor dem Start zurueck - und stoppt einen
						// bereits laufenden Regler NICHT. Der Umrichter
						// moduliert dann durchgehend und haelt den Strom auf
						// null. Das ist hoerbar (Brummen im Stand, 20.09.
						// gemessen: Akkustrom 0,00 A, Motorstrom nur Rauschen)
						// und war zugleich die vermeintliche "Rekuperation" am
						// oberen Ende der Schiebehilfe.
						//
						// Aufgefallen ist es erst, seit die Konfiguration fest
						// einkompiliert ist: vorher stand ADC1 Start auf 0,9 V,
						// ms_without_power kam nie ueber null, und der
						// Steuerblock wurde uebersprungen.
						luna_freigeben();
					} else if (is_reverse) {
						mc_interface_set_current_rel(-out);
						luna_app_treibt = true;
					} else {
						mc_interface_set_current_rel(out);
						luna_app_treibt = true;
					}
				}
			}
		}
	}
}
