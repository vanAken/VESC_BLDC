/*
	Copyright 2021 Marcos Chaparro	mchaparro@powerdesigns.ca
	Copyright 2021 Maximiliano Cordoba	mcordoba@powerdesigns.ca

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "conf_general.h"
#ifdef HW_HAS_LUNA_SERIAL_DISPLAY
#include "hw.h"
#include "luna_display_serial.h"
#include "app.h"
#include "ch.h"
#include "hal.h"
#include "packet.h"
#include "commands.h"
#include "mc_interface.h"
#include "utils.h"
#include <math.h>
#include <string.h>
#include "comm_can.h"
#include "datatypes.h"
#include "stdio.h"



#define CMD_READ			0x11
#define CMD_WRITE			0x16

#define CMD_CONNECT			0x51
#define CMD_READ_BASIC		0x52
#define CMD_WRITE_BASIC		0x52
#define CMD_VERSION			0x90
#define CMD_AMPS			0x0A
#define CMD_WHEEL_RPM		0x20
#define CMD_MOVING			0x31
#define CMD_LIGHT_ON		0xF1
#define CMD_LIGHT_OFF		0xF0
#define CMD_PEDAL_MOV		0x08
#define CMD_BATT_SOC		0x11
#define CMD_BATT_VOLTAGE	0x12
#define CMD_PAS_LEVEL		0x0B
#define CMD_LIGHT			0x1A

#define CMD_READ_BASIC_LENGTH	26
#define CMD_WRITE_BASIC_LENGTH	26

#define CMD_WRITE_BASIC_SUCCESS 0x18

#define LUNA_DISPLAY_BAUD	1200
#define LUNA_SERIAL_BUFFER_SIZE	32
#define LUNA_TX_SERIAL_BUFFER_SIZE	32

typedef enum {
	PAS_LEVEL_0 = 0x00,
	PAS_LEVEL_1 = 0x01,
	PAS_LEVEL_2 = 0x0B,
	PAS_LEVEL_3 = 0x0C,
	PAS_LEVEL_4 = 0x0D,
	PAS_LEVEL_5 = 0x02,
	PAS_LEVEL_6 = 0x15,
	PAS_LEVEL_7 = 0x16,
	PAS_LEVEL_8 = 0x17,
	PAS_LEVEL_9 = 0x03,
	PAS_LEVEL_WALK = 0x06,
} LUNA_PAS_LEVEL;

typedef enum {
	WRITE_LOW_BATTERY_ERROR = 0x00,
	WRITE_MAX_CURRENT_ERROR = 0x01,
	WRITE_ASSIST_LEVEL_ERROR = 0x02,
	WRITE_ASSIST_SPEED_ERROR = 0x0C,
	WRITE_SPEEDOMETER_ERROR = 0x16,
	WRITE_SPEEDOMETER_SIGNAL_ERROR = 0x17,
	WRITE_BASIC_SUCCESS = 0x18
} CMD_WRITE_BASIC_RESPONSE;

typedef enum {
	LUNA_ERROR_NONE = 0x00,
	LUNA_ERROR_BRAKES = 0x03,
	LUNA_ERROR_HIGH_VOLTAGE = 0x07,
	LUNA_ERROR_MOTOR_HALL_SENSOR = 0x08,
	LUNA_ERROR_OVER_TEMPERATURE = 0x10,
	LUNA_ERROR_CURRENT_SENSOR = 0x12,
	LUNA_ERROR_BATTERY_TEMPERATURE = 0x13,
	LUNA_ERROR_WHEEL_SPEED_DETECTION = 0x21,
	LUNA_ERROR_BMS_COMMUNICATION = 0x22,
	LUNA_ERROR_TORQUE_SENSOR = 0x25,
	LUNA_ERROR_SPEED_SENSOR = 0x26,
	LUNA_ERROR_COMMUNICATION = 0x30,
} LUNA_ERROR_CODES;

typedef struct {
	uint8_t header;
	uint8_t length;
	uint8_t manufacturer[4];
	uint8_t model[4];
	uint8_t hardware_version[2];
	uint8_t firmware_version[2];
	uint8_t voltage;
	uint8_t max_current;
	uint8_t checksum;
} controller_info_t;

typedef struct {
	unsigned int rd_ptr;
	unsigned int wr_ptr;
	unsigned char data[LUNA_SERIAL_BUFFER_SIZE];
	unsigned char tx[LUNA_TX_SERIAL_BUFFER_SIZE];
} luna_serial_buffer_t;

static volatile LUNA_PAS_LEVEL pas_level = PAS_LEVEL_1;
static volatile bool display_thread_is_running = false;
static volatile bool display_uart_is_running = false;

// (volker, 26.09.) EINSCHALTEN DES DISPLAYS.
// Das Display bestimmt die Stufe, immer und sofort - auch beim Einschalten,
// wo es 0x00 schickt. Man schaltet von null hoch wie an jedem Rad. Eine
// Zwischenfassung setzte ein neu dazukommendes Display auf Stufe 3 und
// verwarf sein erstes Paket; das war undurchsichtig und ist wieder raus.
// Gegen den toten Bremshebel bei Stufe 0 schuetzt allein der Boden der
// Reku-Tabelle (20 A).
//
// Uebrig bleibt nur Beobachtung ohne Wirkung: ob das Display da ist und wie
// oft es neu dazukam. Eine Luecke ueber LUNA_DISPLAY_GAP_MS gilt als aus.
#define LUNA_DISPLAY_GAP_MS	2000
#define LUNA_STUFEN_BEZUG_A	60.0f   // Stufentabellen gelten in A bei diesem Motorstrom
static volatile systime_t luna_display_last_rx = 0;
static volatile bool luna_display_seen = false;
static volatile uint32_t luna_display_starts = 0;

// (volker, 03.10.2026) LICHT, wie Volker es will:
//  - Controller startet und 1 min kommt kein Display: Licht AN (Notlicht,
//    Heimfahrt ohne Display).
//  - Display war da und geht aus (Pause): Licht AUS. Vorher blieb die
//    Lampe in der Pause an, bis man das Display wieder einschaltete.
//  - Sonst gilt die Lichttaste.
// Die Lichttaste merkt sich nur den Wunsch, den Pin setzt der Thread
// jede Sekunde in luna_licht_pin().
static volatile bool luna_licht_taste = false;
// Rueckmeldung einer Widerstandsmessung: Licht 0,1 s umgekehrt.
static volatile systime_t luna_blink_bis = 0;
static volatile bool luna_blink = false;

static volatile uint8_t luna_status_manuell = 0;   // 0 = automatisch

void luna_display_serial_status_setzen(uint8_t code) {
	luna_status_manuell = code;
}

uint8_t luna_display_status(void) {
	if (luna_status_manuell != 0) {
		return luna_status_manuell;
	}
	if (!luna_profil()->s_pedelec && !conf_general_mcconf_gespeichert) {
		return 0x09;   // Err 09 "Motor pruefen": Motor-Wizard fehlt
	}
	return 0x01;
}

void luna_display_serial_licht_blink(void) {
	luna_blink_bis = chVTGetSystemTimeX() + MS2ST(100);
	luna_blink = true;
}

// (volker, 04.10.2026) Rueckmeldung fuer den 1-s-Takt, 30 s lang:
//   Start: Licht an, jede Sekunde 0,1 s aus (0,9 an / 0,1 aus)
//   Ende : Licht bleibt trotz "aus" noch 30 s an, alle 10 s 0,1 s aus,
//          danach aus
#define LUNA_SIGNAL_MS	30000
static volatile systime_t luna_signal_t0 = 0;
static volatile int luna_signal = 0;    // 0 keins, 1 Start, 2 Ende

void luna_display_serial_licht_signal(bool start) {
	luna_signal_t0 = chVTGetSystemTimeX();
	luna_signal = start ? 1 : 2;
}

// (volker, 26.09.) Ladestand von Hand vorgeben, um herauszufinden, welchen
// Balken das Display bei welcher Prozentzahl zeichnet. -1 = echte Messung.
// Nur im RAM, ein Neustart stellt die Messung wieder her.
static volatile int luna_soc_force = -1;

// (volker, 12.10.2026) Tacho als Anzeige: zeigt luna_tacho_kmh bis luna_tacho_bis.
// Spaeter fuer die Wicklungstemperatur nach der Messung (85 C -> "85").
static volatile int luna_tacho_kmh = -1;
static volatile systime_t luna_tacho_bis = 0;
void luna_display_serial_tacho_zeigen(int kmh, int sekunden) {
	luna_tacho_bis = chVTGetSystemTimeX() + MS2ST(sekunden * 1000);
	luna_tacho_kmh = kmh;
}

// (volker, 20.09.) HEIMFAHR-MODUS: Ohne Display gilt die Vorgabestufe aus
// luna_display_serial_start(6) = Display 3. Eine Zwischenfassung hatte die
// Rekuperation bis zum ersten Displaybefehl gesperrt - das ist wieder
// entfallen, weil es den Heimfahr-Modus halbiert haette.
//
// Faellt das Display unterwegs aus, behaelt die Firmware ohnehin die
// zuletzt gesendete Stufe. Die Regel ist damit durchgaengig einfach:
// Es gilt immer die zuletzt vom Display gesendete Stufe. Nur ohne Display
// greift beim Hochfahren die Vorgabe Display 3; ist eins da, schickt es
// sofort seine 0 und man schaltet selbst hoch.
//
// Der Preis: Wer das Rad mit gezogenem Hebel und ausgeschaltetem Display
// schiebt, bremst gegen den Motor. Bewusst in Kauf genommen - sobald das
// BMS einen Schalter hat, ist der Controller dabei ohnehin stromlos.

/* UART driver configuration structure */
static SerialConfig uart_cfg = {
		LUNA_DISPLAY_BAUD,
		0,
		USART_CR2_LINEN,
		0
};

static luna_serial_buffer_t serial_buffer;
static controller_info_t controller_info = {0x51,0x10,"LUNA","BBSH","12","55",0x05,250,00};

// Threads
static THD_WORKING_AREA(display_process_thread_wa, 1024);
static THD_FUNCTION(display_process_thread, arg);

static bool check_assist_level(uint8_t assist_code);
static LUNA_PAS_LEVEL translate_assist_level(int8_t level);
static void set_assist_level(uint8_t assist_code);
static uint8_t checksum(uint8_t *buf, uint8_t len);
static void serial_send_packet(unsigned char *data, unsigned int len);
static void serial_display_byte_process(unsigned char byte);
static void serial_display_check_rx(void);
static void luna_display_note_rx(void);

// (volker, 07.10.2026) MESSUNG: In welchem Takt schickt das Display die
// Stufe, und wie lange nach dem letzten "Schieben"-Paket kommt die normale
// Stufe? Grundlage fuer eine Zeitsperre, damit die Schiebehilfe beim
// Loslassen schneller endet. Nur Beobachtung, keine Wirkung.
static systime_t lm_stufe_t = 0, lm_walk_t = 0;
static bool lm_walk_aktiv = false;
static uint32_t lm_stufe_n = 0, lm_stufe_min = 0xFFFF, lm_stufe_max = 0, lm_stufe_letzt = 0;
static uint32_t lm_walk_n = 0, lm_walk_min = 0xFFFF, lm_walk_max = 0;
static uint32_t lm_ende_n = 0, lm_ende_letzt = 0, lm_ende_max = 0;

// (volker, 07.10.2026) ZEITSPERRE SCHIEBEHILFE. Gemessen: Solange die Taste
// gehalten wird, kommt "Schieben" alle 586-619 ms; nach dem Loslassen kam die
// normale Stufe erst nach 620-1204 ms. Bleibt "Schieben" LUNA_SCHIEBEN_AUS_MS
// aus, gilt die Taste als losgelassen - zurueck auf die Stufe davor.
#define LUNA_SCHIEBEN_AUS_MS	750
static volatile uint8_t luna_stufe_vor_schieben = PAS_LEVEL_0;

static void luna_stufe_paket_messen(uint8_t code) {
	systime_t now = chVTGetSystemTimeX();
	if (code != PAS_LEVEL_WALK) {
		luna_stufe_vor_schieben = code;
	}
	if (lm_stufe_t != 0) {
		uint32_t d = ST2MS(now - lm_stufe_t);
		lm_stufe_letzt = d;
		if (d < lm_stufe_min) { lm_stufe_min = d; }
		if (d > lm_stufe_max && d < 5000) { lm_stufe_max = d; }
		lm_stufe_n++;
	}
	lm_stufe_t = now;
	if (code == PAS_LEVEL_WALK) {
		if (lm_walk_aktiv) {
			uint32_t d = ST2MS(now - lm_walk_t);
			if (d < lm_walk_min) { lm_walk_min = d; }
			if (d > lm_walk_max) { lm_walk_max = d; }
			lm_walk_n++;
		}
		lm_walk_aktiv = true;
		lm_walk_t = now;
	} else if (lm_walk_aktiv) {
		// erstes normales Paket nach dem Schieben
		lm_ende_letzt = ST2MS(now - lm_walk_t);
		if (lm_ende_letzt > lm_ende_max) { lm_ende_max = lm_ende_letzt; }
		lm_ende_n++;
		lm_walk_aktiv = false;
	}
}

// (volker, 07.10.2026) Display-Empfang je nach Controller an PB11 (Werk)
// oder PC11 (Testcontroller, PB11 defekt). PC11 ist ebenfalls USART3_RX
// (AF7), derselbe UART empfaengt also nur ueber einen anderen Pin.
static void luna_display_rx_pin_setzen(void) {
	// Senden: PB10 (Werk) oder PC10 (rotes Rad, Display ganz an UART2).
	// PC10 ist ebenfalls USART3_TX (AF7).
	if (luna_profil()->display_tx_pc10) {
		palSetPadMode(GPIOC, 10, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
				PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	}
	if (luna_profil()->display_rx_pc11) {
		palSetPadMode(GPIOC, 11, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
				PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	} else {
		palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
				PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	}
}

void luna_display_serial_soc_force(int prozent) {
	luna_soc_force = prozent;
}

int luna_display_serial_soc_forced(void) {
	return luna_soc_force;
}

bool luna_display_serial_connected(void) {
	return luna_display_seen &&
			ST2MS(chVTTimeElapsedSinceX(luna_display_last_rx)) <= LUNA_DISPLAY_GAP_MS;
}

void luna_display_serial_print_status(void) {
	bool da = luna_display_serial_connected();
	commands_printf("--- Display ---");
	commands_printf("Verbindung      : %s", da ? "aktiv" : "still");
	commands_printf("Einschaltvorgaenge: %lu", (unsigned long)luna_display_starts);
	commands_printf("Stufe           : 0x%02x", (unsigned int)pas_level);
	if (luna_soc_force >= 0) {
		commands_printf("Ladestand       : %d %% VON HAND  (luna_soc auto beendet das)",
				luna_soc_force);
	}
	commands_printf("Reku jetzt      : %.1f A   Schiebehilfe: %.1f A",
			(double)app_adc_get_regen_amps(), (double)app_adc_get_walk_amps());
	commands_printf("Stufenpakete    : %lu, Abstand letzt %lu / min %lu / max %lu ms",
			(unsigned long)lm_stufe_n, (unsigned long)lm_stufe_letzt,
			(unsigned long)(lm_stufe_min == 0xFFFF ? 0 : lm_stufe_min), (unsigned long)lm_stufe_max);
	commands_printf("Schieben-Pakete : %lu, Abstand min %lu / max %lu ms",
			(unsigned long)lm_walk_n,
			(unsigned long)(lm_walk_min == 0xFFFF ? 0 : lm_walk_min), (unsigned long)lm_walk_max);
	commands_printf("Ende Schieben   : %lu mal, letztes Paket -> normale Stufe: letzt %lu / max %lu ms",
			(unsigned long)lm_ende_n, (unsigned long)lm_ende_letzt, (unsigned long)lm_ende_max);
}

void luna_display_serial_start(int8_t initial_level) {
    pas_level = translate_assist_level(initial_level);
    // Dieselbe Stufe gilt ohne Display (Heimfahr-Modus) und bei jedem
    // Display, das neu dazukommt. Ein Wert, eine Regel.

	if (!display_thread_is_running) {
		chThdCreateStatic(display_process_thread_wa, sizeof(display_process_thread_wa),
				NORMALPRIO, display_process_thread, NULL);
		display_thread_is_running = true;
	}
	serial_buffer.rd_ptr = 0;
	serial_buffer.wr_ptr = 0;

	sdStart(&HW_UART_DEV, &uart_cfg);
	if (!luna_profil()->display_tx_pc10) {
		palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
				PAL_STM32_OSPEED_HIGHEST |
				PAL_STM32_PUDR_PULLUP);
	}
	luna_display_rx_pin_setzen();
    
	display_uart_is_running = true;
	unsigned char buffer[32];
	uint8_t lenght = sprintf(buffer, "Bafang Serial Display\r\n");
	sdWrite(&HW_UART_DEV, buffer, lenght);

}

static LUNA_PAS_LEVEL translate_assist_level(int8_t level) {
    switch (level) {
        case 0: return PAS_LEVEL_0;
        case 1: return PAS_LEVEL_1;
        case 2: return PAS_LEVEL_2;
        case 3: return PAS_LEVEL_3;
        case 4: return PAS_LEVEL_4;
        case 5: return PAS_LEVEL_5;
        case 6: return PAS_LEVEL_6;
        case 7: return PAS_LEVEL_7;
        case 8: return PAS_LEVEL_8;
        case 9: return PAS_LEVEL_9;
        default: return -1;
    }    
}

/*
 * checks if the assist code to a current level
 *
 * returns true if valid, false if not.
 */
static bool check_assist_level(uint8_t assist_code) {
	bool ret = false;

	switch (assist_code) {
		case PAS_LEVEL_0:
		case PAS_LEVEL_1:
		case PAS_LEVEL_2:
		case PAS_LEVEL_3:
		case PAS_LEVEL_4:
		case PAS_LEVEL_5:
		case PAS_LEVEL_6:
		case PAS_LEVEL_7:
		case PAS_LEVEL_8:
		case PAS_LEVEL_9:
		case PAS_LEVEL_WALK:
			ret = true;
			break;
		default:
			break;
	}
	return ret;
}

/*
 * Sets the PAS current according to the assist code
 *
 */
static void set_assist_level(uint8_t assist_code) {
	float current_scale;
    volatile mc_configuration *mcconf = (volatile mc_configuration*) mc_interface_get_configuration();

	// Schiebehilfe zuerst und dann zurueck: Das Display schickt 0x06,
	// solange die Taste gedrueckt ist, und danach wieder die normale
	// Stufe. Wuerden Reku und Antrieb hier mitgesetzt, fielen sie
	// waehrend des Schiebens auf null - einschliesslich der Reku, wenn
	// man dabei den Hebel zieht. So behalten beide ihre Werte, und die
	// Staerke der Schiebehilfe stammt aus der zuletzt gueltigen Stufe.
	app_adc_set_walk_assist(assist_code == PAS_LEVEL_WALK);
	if (assist_code == PAS_LEVEL_WALK) {
		app_pas_set_current_sub_scaling(0.0);
		return;
	}

	// --- Schiebehilfe ueber die Displaystufe ---------------------------
	// 16 A bei ausgeschaltetem Antrieb, dann in Vierer-Schritten. Schieben
	// soll auf Knopfdruck helfen, auch wenn die Unterstuetzung aus ist -
	// deshalb faengt die Reihe nicht bei null an.
	//
	//   Display aus = 16 A     Display 3 = 28 A
	//   Display 1   = 20 A     Display 4 = 32 A
	//   Display 2   = 24 A     Display 5 = 36 A
	// (volker, 10.10.2026) ALLE STUFENTABELLEN RELATIV. Die Zahlen unten sind
	// Ampere bei LUNA_STUFEN_BEZUG_A = 60 A (weisses Rad). Tatsaechlich gilt
	// Wert / 60 * Motorstrom aus VESC Tool: Antrieb und Schiebehilfe auf
	// "Motor Current Max", Reku auf den Betrag von "Motor Current Max Brake".
	// Bei 60 A aendert sich nichts; mit kleinerem Motor (z. B. Pedelec 25)
	// skalieren alle Stufen mit, ohne neu zu bauen.
	const float k_antrieb = mcconf->l_current_max / LUNA_STUFEN_BEZUG_A;
	const float k_reku = fabsf(mcconf->l_current_min) / LUNA_STUFEN_BEZUG_A;

	float walk_amps;
	switch (assist_code) {
		case PAS_LEVEL_0: walk_amps = 16.0; break;   // Display aus
		case PAS_LEVEL_1: walk_amps = 18.0; break;
		case PAS_LEVEL_2: walk_amps = 20.0; break;   // Display 1
		case PAS_LEVEL_3: walk_amps = 22.0; break;
		case PAS_LEVEL_4: walk_amps = 24.0; break;   // Display 2
		case PAS_LEVEL_5: walk_amps = 26.0; break;
		case PAS_LEVEL_6: walk_amps = 28.0; break;   // Display 3
		case PAS_LEVEL_7: walk_amps = 30.0; break;
		case PAS_LEVEL_8: walk_amps = 32.0; break;   // Display 4
		case PAS_LEVEL_9: walk_amps = 36.0; break;   // Display 5
		default:          walk_amps = 16.0; break;
	}
	app_adc_set_walk_amps(walk_amps * k_antrieb);

	// --- Rekuperation ueber die Displaystufe ---------------------------
	// Ampere bei 60 A Bezug (seit 10.10.2026 relativ, siehe oben).
	// Nach der Probefahrt vom 20.09. um zwei Stufen nach unten verschoben:
	// was vorher Stufe 5 war, ist jetzt Stufe 3. Stufe 4 und 5 setzen die
	// Reihe linear fort.
	//
	// (volker, 26.09.) Stufe "Display aus" bekommt 20 A statt null, die
	// Reihe laeuft von dort linear in Achter-Schritten bis 60 A. Keine
	// Rekuperation zu haben macht wenig Sinn - man laesst den Hebel los,
	// wenn man nicht bremsen will. Damit gilt durchgaengig: Der Hebel
	// bremst immer, unabhaengig von Stufe und Display.
	//
	//   Display aus = 20 A   Display 3 = 44 A
	//   Display 1   = 28 A   Display 4 = 52 A
	//   Display 2   = 36 A   Display 5 = 60 A
	//
	// Der Akkustrom (l-in-current-min) begrenzt die Leistung bei hohem
	// Tempo - das ist Aufgabe des BMS-Schutzes, nicht dieser Tabelle.
	// Damit 50 und 60 A ueberhaupt ankommen, muss l-current-min auf -60
	// stehen; sonst kappt die Firmware vorher.
	float regen_amps;
	switch (assist_code) {
		case PAS_LEVEL_0: regen_amps = 20.0; break;   // Display aus
		case PAS_LEVEL_1: regen_amps = 24.0; break;
		case PAS_LEVEL_2: regen_amps = 28.0; break;   // Display 1
		case PAS_LEVEL_3: regen_amps = 32.0; break;
		case PAS_LEVEL_4: regen_amps = 36.0; break;   // Display 2
		case PAS_LEVEL_5: regen_amps = 40.0; break;
		case PAS_LEVEL_6: regen_amps = 44.0; break;   // Display 3
		case PAS_LEVEL_7: regen_amps = 48.0; break;
		case PAS_LEVEL_8: regen_amps = 52.0; break;   // Display 4
		case PAS_LEVEL_9: regen_amps = 60.0; break;   // Display 5
		default:          regen_amps = 20.0; break;   // nie null
	}
	app_adc_set_regen_amps(regen_amps * k_reku);

	// --- Antrieb ueber die Displaystufe --------------------------------
	// Ebenfalls Ampere bei 60 A Bezug. Nach der Probefahrt vom 20.09.:
	// Stufe 1 war einen Tick zu kraeftig (-20 %), oben fehlte Reserve am
	// Berg. Die Reihe laeuft jetzt linear von 7 bis 60 A.
	//
	//   Display 1 =  7 A   (vorher 8,9 A)
	//   Display 2 = 20 A
	//   Display 3 = 33 A
	//   Display 4 = 46 A
	//   Display 5 = 60 A   (vorher 40 A)
	//
	// app_pas_set_current_sub_scaling() erwartet einen Anteil, deshalb
	// wird hier auf l_current_max bezogen. Steht dort ein kleinerer Wert,
	// wirkt er als Deckel - gewollt, das ist die harte Grenze.
	float drive_amps;
	switch (assist_code) {
		case PAS_LEVEL_0: drive_amps =  0.0; break;   // Display aus
		case PAS_LEVEL_1: drive_amps =  3.5; break;
		case PAS_LEVEL_2: drive_amps =  7.0; break;   // Display 1
		case PAS_LEVEL_3: drive_amps = 13.5; break;
		case PAS_LEVEL_4: drive_amps = 20.0; break;   // Display 2
		case PAS_LEVEL_5: drive_amps = 26.5; break;
		case PAS_LEVEL_6: drive_amps = 33.0; break;   // Display 3
		case PAS_LEVEL_7: drive_amps = 39.5; break;
		case PAS_LEVEL_8: drive_amps = 46.0; break;   // Display 4
		case PAS_LEVEL_9: drive_amps = 60.0; break;   // Display 5
		default: return;
	}

	// Boost beim Hochschalten (siehe app_pas.c). set_assist_level wird
	// auch periodisch mit gleicher Stufe aufgerufen, daher nur bei
	// Wechsel. ~13 A je Displaystufe. Nicht aus Stufe 0 heraus: nach dem
	// Einschalten schaltet man im Stand hoch, das soll keinen Satz geben.
	{
		static float amps_alt = -1.0;
		if (amps_alt > 0.0 && drive_amps > amps_alt) {
			float stufen = (drive_amps - amps_alt) / 13.25;
			if (stufen >= 1.9) {
				app_pas_set_boost(1.0);
			} else if (stufen >= 0.9) {
				app_pas_set_boost(0.5);
			}
		} else if (drive_amps < amps_alt) {
			app_pas_set_boost(0.0);    // runterschalten beendet den Boost
		}
		amps_alt = drive_amps;
	}

	// relativ: drive_amps ist Ampere bei 60 A Bezug -> Anteil von l_current_max
	current_scale = drive_amps / LUNA_STUFEN_BEZUG_A;
	utils_truncate_number(&current_scale, 0.0, 1.0);
	app_pas_set_current_sub_scaling(current_scale);

}

static uint8_t checksum(uint8_t *buf, uint8_t len) {
	uint8_t sum = 0;
	for(int i = 0; i<len; i++) {
		sum += buf[i];
	}
	return sum;
}

static void serial_send_packet(unsigned char *data, unsigned int len) {

	if (display_uart_is_running) {
		sdWrite(&HW_UART_DEV, data, len);
	}
}

static void serial_display_byte_process(unsigned char byte) {
	//append new byte to the buffer.
	serial_buffer.data[serial_buffer.wr_ptr] = byte;
	serial_buffer.wr_ptr++;

	uint8_t rd_ptr = serial_buffer.rd_ptr;	//read pointer to try at different start addresses
	
	// process with at least 2 bytes available to read
	while( (serial_buffer.wr_ptr - rd_ptr ) > 1) {

		if(serial_buffer.data[rd_ptr] == CMD_WRITE) {
			// start byte found
			uint8_t command = serial_buffer.data[rd_ptr + 1];
			uint8_t checksum_addr;

			switch (command) {
				case CMD_LIGHT:
					// 0x16 0x1A <wert>, 0xF1 an, 0xF0 aus. Das Display
					// erwartet KEINE Antwort - am Emulator nachgewiesen.
					if ( (serial_buffer.wr_ptr - rd_ptr) < 3 ) {
						break;
					}
					{
						bool an = (serial_buffer.data[rd_ptr + 2] == CMD_LIGHT_ON);
						// (04.10.2026) Den Pin setzt nur noch der Thread alle 100 ms
						// aus luna_licht_taste - sonst ueberschreibt jedes vom
						// Display wiederholte Paket die 30-s-Rueckmeldung.
						// Das Display wiederholt seinen Lichtzustand im Abfragezyklus.
						// Als Tastendruck zaehlt nur ein WECHSEL.
						bool wechsel = (an != luna_licht_taste);
						luna_licht_taste = an;
						// (volker, 20.09.) Die Taste schaltet zugleich die
						// Verlaufsaufzeichnung. Solange der Lichtausgang nicht
						// belegt ist, ist sie frei - und am Berg ist ein Knopf
						// am Lenker das einzige brauchbare Bedienelement.
						// Einschalten beginnt eine neue Aufnahme, Ausschalten
						// haelt sie an. Auslesen mit "luna_trace".
						// (volker, 04.10.2026) Bremse + Lichttaste steuert 1-s-Takt
						// und Widerstandsmessung, siehe app_adc_licht_taste().
						if (wechsel) {
							app_adc_licht_taste(an);
						}
					}
					rd_ptr += 3;
					serial_buffer.rd_ptr = rd_ptr;
					continue;
				case CMD_PAS_LEVEL:
					if ( (serial_buffer.wr_ptr - rd_ptr) < 4 ) {	//this packet needs 4 bytes
						break;
					}
					checksum_addr = rd_ptr + 3;
					if(checksum_addr <= serial_buffer.wr_ptr) {	//check the checksum has been received
						// check sum
						if( serial_buffer.data[checksum_addr] ==
							checksum(serial_buffer.data + rd_ptr, 3) ) {

							if(check_assist_level(serial_buffer.data[rd_ptr + 2])){
								luna_stufe_paket_messen(serial_buffer.data[rd_ptr + 2]);
								pas_level = serial_buffer.data[rd_ptr + 2];
								set_assist_level(pas_level);
							}
							serial_buffer.rd_ptr = rd_ptr + 4;	//mark bytes as read
						}
					}
					break;
				// (volker, 26.09.) CMD_WRITE_BASIC ist bewusst NICHT behandelt.
				// Die Vorlage stammt vom Lishui-Display, das Einstellungen in den
				// Controller schreiben kann; stancecokes Fassung legte damit
				// Stromgrenze und Raddurchmesser in die Motorkonfiguration und
				// speicherte sie im Flash. Das Bafang-Display tut das nicht, und
				// die Konfiguration gehoert nicht dem Display. Ein solches Paket
				// faellt in den default-Zweig und wird verworfen.
				default:
					++rd_ptr;
					serial_buffer.rd_ptr = rd_ptr;	//discard the CMD_WRITE byte as it wasn't followed by a supported command
					continue;
			}
		}

		if(serial_buffer.data[rd_ptr] == CMD_READ) {
			// start byte found
			uint8_t command = serial_buffer.data[rd_ptr + 1];

			switch (command) {
				case CMD_VERSION:
					serial_send_packet((uint8_t*)&controller_info, sizeof(controller_info_t));
					rd_ptr +=2;
					serial_buffer.rd_ptr = rd_ptr;	//mark bytes as read
					continue;
				case CMD_READ_BASIC:
					{
					volatile mc_configuration *mcconf = (volatile mc_configuration*) mc_interface_get_configuration();

					serial_buffer.tx[0] = 0x52;
					serial_buffer.tx[1] = CMD_READ_BASIC_LENGTH - 2;
					serial_buffer.tx[2] = (uint8_t) mcconf->l_min_vin;        //low battery protection [V]
					serial_buffer.tx[3] = (uint8_t) mcconf->l_current_max;    //current limit [A]

					for(int i=0; i<10; i++) {
						serial_buffer.tx[i+4] = (uint8_t) 20;    //assist level [%]
					}

					serial_buffer.tx[4] = 0;
					serial_buffer.tx[5] = 28;
					serial_buffer.tx[6] = 37;
					serial_buffer.tx[7] = 46;
					serial_buffer.tx[8] = 55;
					serial_buffer.tx[9] = 64;
					serial_buffer.tx[10] = 73;
					serial_buffer.tx[11] = 82;
					serial_buffer.tx[12] = 91;
					serial_buffer.tx[13] = 100;

					for(int i=0; i<10; i++) {
						serial_buffer.tx[i+14] = (uint8_t) 100;    //speed level [%]
					}

					serial_buffer.tx[24] = (uint8_t) (mcconf->si_wheel_diameter * 39.4 * 2.0); //wheel diameter [inch*2]
					serial_buffer.tx[25] = 0x01;	//speedometer [01 = internal]
					serial_buffer.tx[26] = checksum(serial_buffer.tx + 2, CMD_READ_BASIC_LENGTH - 3);

					serial_send_packet(serial_buffer.tx, CMD_READ_BASIC_LENGTH);
					rd_ptr +=2;
					serial_buffer.rd_ptr = rd_ptr;
					continue;
					}
				case CMD_AMPS:
					{
					float current = mc_interface_get_tot_current_in_filtered() * 2.0;
					utils_truncate_number(&current, 0, 256);
					serial_buffer.tx[0] = (uint8_t) (current);
					serial_buffer.tx[1] = serial_buffer.tx[0];
					serial_send_packet(serial_buffer.tx, 2);
					rd_ptr +=2;
					serial_buffer.rd_ptr = rd_ptr;	//mark bytes as read
					continue;
					}
				case CMD_WHEEL_RPM:
					{
					// (volker) Der #ifdef HW_HAS_WHEEL_SPEED_SENSOR ist hier
					// entfernt. Er musste weg, sobald der Radsensor abgeschaltet
					// ist: sonst wird rd_ptr nicht weitergezaehlt, das continue
					// laeuft trotzdem, und der Parser dreht sich endlos.
					// mc_interface_get_speed() liefert das Tempo jetzt aus der
					// Motordrehzahl.
					// rpm = spd [m/s] / perim [m] * 60[s/min]
					const volatile mc_configuration *conf = mc_interface_get_configuration();
					uint16_t wheel_rpm = (uint16_t)(mc_interface_get_speed() / (M_PI * conf->si_wheel_diameter) * 60.0);
					// (volker, 12.10.2026) Tacho als Anzeige: feste km/h fuer eine Zeit
					if (luna_tacho_kmh >= 0 && chVTGetSystemTimeX() < luna_tacho_bis) {
						// SW102 gemessen 12.10.2026: zeigt 1 weniger (85 -> 84, 99 -> 98,
						// 86 -> 85). Deshalb +1. 100 zeigt es zweistellig als "00".
						wheel_rpm = (uint16_t)ceilf((float)(luna_tacho_kmh + 1) / 3.6 / (M_PI * conf->si_wheel_diameter) * 60.0);
					}
					serial_buffer.tx[0] = (uint8_t)(wheel_rpm / 256);
					serial_buffer.tx[1] = (uint8_t)(wheel_rpm & 0xFFFF);
					serial_buffer.tx[2] = serial_buffer.tx[0] + serial_buffer.tx[1] + 32;
					serial_send_packet(serial_buffer.tx, 3);
					rd_ptr +=2;
					serial_buffer.rd_ptr = rd_ptr;
					continue;
					}
				case CMD_MOVING:
					if( mc_interface_get_speed() > 0.0) {
						serial_buffer.tx[0] = 0x31;	//0x31 0x31 indicates bike is in movement.
						serial_buffer.tx[1] = 0x31;
					}
					else {
						serial_buffer.tx[0] = 0x30;
						serial_buffer.tx[1] = 0x30;
					}
					serial_send_packet(serial_buffer.tx, 2);
					rd_ptr +=2;
					serial_buffer.rd_ptr = rd_ptr;
					continue;
				case CMD_PEDAL_MOV:
					// (volker, 11.10.2026) Statusabfrage des Displays, GETESTET: 0x09 ->
					// "error 9" im SW102 (Bafang-Fehlercodes, Spez. Kap. 9).
					// 0x01 = in Ordnung. Von Hand setzen: luna_err.
					serial_buffer.tx[0] = luna_display_status();
					serial_send_packet(serial_buffer.tx, 1);
					rd_ptr +=2;
					serial_buffer.rd_ptr = rd_ptr;
					continue;
				case CMD_BATT_SOC:
					// Battery state of charge 0-100%.
					{
					float wh_left = 0.0;
					float battery_level = mc_interface_get_battery_level(&wh_left) * 100.0;
					utils_truncate_number((float*)&battery_level, 0.0, 100.0);
					if (luna_soc_force >= 0) {
						battery_level = (float)luna_soc_force;
					}

					serial_buffer.tx[0] = (uint8_t) battery_level;
					serial_buffer.tx[1] = (uint8_t) battery_level;
					serial_send_packet(serial_buffer.tx, 2);
					rd_ptr +=2;
					serial_buffer.rd_ptr = rd_ptr;
					continue;
					}
				case CMD_BATT_VOLTAGE:
					{
					// This command was custom made for the eggrider display running at 72V
					uint16_t batt_voltage = (uint16_t)(GET_INPUT_VOLTAGE() ) * 100.0;
					serial_buffer.tx[0] = (uint8_t)(batt_voltage / 256);
					serial_buffer.tx[1] = (uint8_t)(batt_voltage & 0xFFFF);
					serial_buffer.tx[2] = serial_buffer.tx[0] + serial_buffer.tx[1] + 0x12;
					serial_send_packet(serial_buffer.tx, 3);
					rd_ptr +=2;
					serial_buffer.rd_ptr = rd_ptr;              
					continue;
					}
				default:
					++rd_ptr;
					serial_buffer.rd_ptr = rd_ptr;	//discard the CMD_READ byte as it wasn't followed by a supported command
					continue;
			}
		}
		rd_ptr++;
	}

	if(serial_buffer.rd_ptr > 0) {
		memmove(serial_buffer.data, serial_buffer.data + serial_buffer.rd_ptr, LUNA_SERIAL_BUFFER_SIZE - serial_buffer.rd_ptr);
		serial_buffer.wr_ptr -= serial_buffer.rd_ptr;
		serial_buffer.rd_ptr = 0;
	}
	if(serial_buffer.wr_ptr == (LUNA_SERIAL_BUFFER_SIZE - 1) ) {
		//shift buffer to the left discarding the oldest byte
		memmove(serial_buffer.data,serial_buffer.data + 1, LUNA_SERIAL_BUFFER_SIZE - 1);
		serial_buffer.wr_ptr -= 1;
	}
}

// Jedes empfangene Byte ist ein Lebenszeichen. Nur gezaehlt, die Stufe
// bleibt unberuehrt.
static void luna_display_note_rx(void) {
	if (!luna_display_seen ||
			ST2MS(chVTTimeElapsedSinceX(luna_display_last_rx)) > LUNA_DISPLAY_GAP_MS) {
		luna_display_seen = true;
		luna_display_starts++;
	}
	luna_display_last_rx = chVTGetSystemTimeX();
}

static void serial_display_check_rx(void){
	bool rx = true;
	while (rx) {
		rx = false;

		if (display_uart_is_running) {
			msg_t res = sdGetTimeout(&HW_UART_DEV, TIME_IMMEDIATE);
			if (res != MSG_TIMEOUT) {
				luna_display_note_rx();
				serial_display_byte_process(res);
				rx = true;
//				unsigned char buffer[32];
//				buffer[0]=(unsigned char)res;
//				sdWrite(&HW_UART_DEV, buffer, 1);


			}

		}
	}
}

static THD_FUNCTION(display_process_thread, arg) {
	(void)arg;
	chRegSetThreadName("Luna serial display");

	event_listener_t el;
	chEvtRegisterMaskWithFlags(&HW_UART_DEV.event, &el, EVENT_MASK(0), CHN_INPUT_AVAILABLE);

    // Wait for motor config initialization
	chThdSleepMilliseconds(500);


    // Set default power level
	set_assist_level(pas_level);

	for(;;) {
		chEvtWaitAnyTimeout(ALL_EVENTS, ST2MS(100));

		// (volker, 20.09.2026) Pinmodus des Bremseingangs bei jedem
		// Durchlauf neu setzen, nicht einmalig.
		//
		// main.c ruft app_uartcomm_start(UART_PORT_BUILTIN) und danach
		// app_set_configuration(). Letzteres setzt PC10/PC11 bei
		// abgeschaltetem permanentem UART auf PAL_MODE_INPUT - also OHNE
		// Pull-up. Ein schwebender Eingang liest 0, und der Bremsschalter
		// ist active low: die Firmware hielt den Hebel dauerhaft fuer
		// gezogen und legte im Stand Bremsstrom an.
		//
		// Ein einmaliges Nachsetzen nach 500 ms reichte NICHT: Der Thread
		// zaehlt ab seiner Erzeugung in hw_init_gpio(), waehrend zwischen
		// dort und app_set_configuration() noch conf_general_init() mit
		// Flash-Zugriff und die Motorinitialisierung liegen. Das Nachsetzen
		// kam zu frueh und wurde ueberschrieben. Hier alle 100 ms, damit es
		// unabhaengig von der Reihenfolge gilt - auch wenn spaeter jemand
		// ueber VESC Tool eine App-Konfiguration schreibt.
		// (volker, 20.09.) Nur noch jede Sekunde statt alle 100 ms.
		// palSetPadMode() schreibt mehrere Register nacheinander; ist der
		// Pull-up dabei kurz inaktiv, schwebt der Eingang und liest 0 -
		// bei active low also "gebremst". Im 100-ms-Takt ergab das genau
		// die Flatterei, die am 20.09. die Rekuperation zerhackt hat.
		// Einmal je Sekunde genuegt vollauf, denn ueberschrieben wird der
		// Modus nur, wenn jemand die App-Konfiguration neu schreibt.
		static int pin_div = 0;
		if (++pin_div >= 10) {
			pin_div = 0;
#ifdef HW_HAS_BRAKE_SENSOR
			if (!luna_profil()->bremse_adc1) {
				palSetPadMode(HW_BRAKE_SENSOR_PORT, HW_BRAKE_SENSOR_PIN,
						PAL_MODE_INPUT_PULLUP);
			} else if (luna_pc10_unbenutzt()) {
				// weisses Rad: PC10 defekt, ohne Pull-up stilllegen
				palSetPadMode(HW_BRAKE_SENSOR_PORT, HW_BRAKE_SENSOR_PIN,
						PAL_MODE_INPUT_ANALOG);
			}
#endif
			// Der permanente UART kann PC11 beim Konfigurieren auf Eingang
			// zuruecksetzen - beim Testcontroller ist das der Display-Empfang.
			if (luna_profil()->display_rx_pc11 || luna_profil()->display_tx_pc10) {
				luna_display_rx_pin_setzen();
			}
#ifdef LUNA_LIGHT_GPIO
			palSetPadMode(LUNA_LIGHT_GPIO, LUNA_LIGHT_PIN,
					PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_LOWEST);
#endif
		}

#ifdef LUNA_LIGHT_GPIO
		{
		bool licht;
		if (!luna_display_seen) {
			// Nach dem Start 1 min auf ein Display warten, dann Notlicht.
			licht = ST2MS(chVTGetSystemTimeX()) > 60000;
		} else if (!luna_display_serial_connected()) {
			licht = false;              // Display aus -> Licht aus
			luna_licht_taste = false;   // und beim Wiedereinschalten aus
			app_adc_trace_set(false);   // 1-s-Takt endet mit dem Display
		} else {
			licht = luna_licht_taste;
		}
		if (luna_signal != 0 && luna_display_serial_connected()) {
			uint32_t ms = ST2MS(chVTTimeElapsedSinceX(luna_signal_t0));
			if (ms >= LUNA_SIGNAL_MS) {
				luna_signal = 0;
			} else if (luna_signal == 1) {
				licht = (ms % 1000) >= 100;
			} else {
				licht = (ms % 10000) >= 100 || ms < 1000;
			}
		}
		if (luna_blink && chVTGetSystemTimeX() < luna_blink_bis) {
			licht = !licht;
		} else {
			luna_blink = false;
		}
		if (licht) {
			palSetPad(LUNA_LIGHT_GPIO, LUNA_LIGHT_PIN);
		} else {
			palClearPad(LUNA_LIGHT_GPIO, LUNA_LIGHT_PIN);
		}
		}
#endif

		serial_display_check_rx();

		if (pas_level == PAS_LEVEL_WALK && lm_walk_aktiv &&
				ST2MS(chVTTimeElapsedSinceX(lm_walk_t)) > LUNA_SCHIEBEN_AUS_MS) {
			pas_level = luna_stufe_vor_schieben;
		}
		set_assist_level(pas_level); //assert periodically to make sure changes are commited when a new motor config is written
//		unsigned char buffer[32];
//		uint8_t lenght = sprintf(buffer, "%d, %d, %d, %d\r\n",
//				(int)app_pas_get_pedal_rpm(),
//				(int)(app_pas_get_current_target_rel()*1000),
//				(int)(app_adc_get_decoded_level()*1000),
//				(int)(app_adc_get_decoded_level2()*1000));
//		sdWrite(&HW_UART_DEV, buffer, lenght);

	}
}
#endif

// (volker, 12.10.2026) Leitungstest: Nullbytes auf der Sendeleitung (PB10,
// gruen). Bei 1200 Baud ist die Leitung dann ~90 % der Zeit auf 0 V - das
// Messgeraet zeigt ~0,3 V statt 3,3 V. So findet man die Ader am Stecker.
void luna_display_serial_null_senden(int sekunden) {
	static const uint8_t nullen[12] = {0};
	for (int i = 0; i < sekunden * 10; i++) {
		sdWrite(&HW_UART_DEV, nullen, sizeof(nullen));
	}
}
