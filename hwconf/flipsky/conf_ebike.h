/*
	Vorgabewerte fuer Volkers S-Pedelec, Controller XXXXXX.
	Stand 19.09.2026.

	Warum es diese Datei gibt
	-------------------------
	Ein Firmware-Update ueberschreibt die Sektoren 0 bis 7, und darin liegen
	auch Sektor 1 und 2 mit dem emulierten EEPROM. Nach jedem Aufspielen
	stand die Konfiguration deshalb auf Werkseinstellung, und es waren
	jedesmal faellig: Kette ab, foc_detect_apply_all, LispBM-Skript,
	App-Konfiguration schreiben.

	mcconf_default.h und appconf_default.h sind durchgaengig als
	#ifndef/#define gebaut. Wer frueher definiert, gewinnt. Diese Datei wird
	ganz oben aus hw_75_100_V2.h eingebunden und macht Volkers Werte zur
	Werkseinstellung. Damit ist der Controller nach einem Update sofort
	fahrbereit.

	Wichtig: Die Werte greifen nur, wenn der Konfigurationsbereich leer ist,
	also genau nach einem Update. Eine gespeicherte Konfiguration hat
	weiterhin Vorrang - spaetere Abstimmung per VESC Tool oder LispBM geht
	also nicht verloren.

	Luna macht dasselbe fuer die BBSHD, siehe hwconf/luna/bbshd/.
*/

#ifndef CONF_EBIKE_H_
#define CONF_EBIKE_H_

// =====================================================================
// Motorkonfiguration
// =====================================================================

// --- Motorkennwerte -------------------------------------------------
// Gemessen am 05.09.2026 mit KALTEM Motor. Am 19.09. ergab der warme
// Motor 88,52 mOhm; der Controller startet aber fast immer kalt, und aus
// R leitet er die Reglerparameter ab. Deshalb der kalte Wert.
// L und Flussverkettung aendern sich kaum, die Streuung liegt bei unter
// einem Prozent ueber alle Messungen.
#define MCCONF_FOC_MOTOR_R				0.07373      // 73,73 mOhm
#define MCCONF_FOC_MOTOR_L				0.00019237   // 192,37 uH
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE	0.028094     // 28,094 mWb
#define MCCONF_FOC_MOTOR_LD_LQ_DIFF		0.0000257435

// Abgeleitet: kp = L * 1000, ki = R * 1000, observer_gain = 1000 / Fluss^2.
// Die gleiche Rechnung, die foc_detect_apply_all anstellt - hier nur mit
// den kalten Werten statt den warmen.
#define MCCONF_FOC_CURRENT_KP			0.19237
#define MCCONF_FOC_CURRENT_KI			73.73
#define MCCONF_FOC_OBSERVER_GAIN		1266991.0

// --- Lagegeber ------------------------------------------------------
// 2 = FOC_SENSOR_MODE_HALL. Tabelle aus luna_dump_conf vom 19.09.,
// 255 an Position 0 und 7 sind die ungueltigen Zustaende.
#define MCCONF_FOC_SENSOR_MODE			FOC_SENSOR_MODE_HALL
#define MCCONF_FOC_HALL_TAB_0			255
#define MCCONF_FOC_HALL_TAB_1			172
#define MCCONF_FOC_HALL_TAB_2			103
#define MCCONF_FOC_HALL_TAB_3			139
#define MCCONF_FOC_HALL_TAB_4			38
#define MCCONF_FOC_HALL_TAB_5			4
#define MCCONF_FOC_HALL_TAB_6			71
#define MCCONF_FOC_HALL_TAB_7			255

// --- Drehrichtung ---------------------------------------------------
// Ohne das laeuft der Motor rueckwaerts. War bisher der haeufigste
// Stolperstein nach einem Update.
#define MCCONF_M_INVERT_DIRECTION		true

// --- Geometrie ------------------------------------------------------
// Bestimmt die Tachoanzeige und jede Geschwindigkeitsrechnung im
// Controller. foc_detect_apply_all setzt diese Werte zurueck, deshalb
// gehoeren sie unbedingt hierher.
#define MCCONF_SI_MOTOR_POLES			46
#define MCCONF_SI_GEAR_RATIO			1.0
// (volker, 04.10.2026) GPS-Vergleich, GPSLogger 1 s, je 20 s konstant:
// Controller 21,9 / 46,0 km/h, GPS 20,8 / 43,9 km/h -> Faktor 0,952.
// Wirksamer Abrolldurchmesser Big Apple 2" unter Last: 0,73 * 0,952.
// Vorher 0,73 (Massband) - das Rad war real 5 % langsamer als berechnet.
// (volker, 06.10.2026) Schwellen fuer die Bremse am Gaseingang ADC1 - welche
// Controller sie nutzen, steht seit 07.10. im Profil (hw_75_100_V2.c).
// Schwellen tief: ADC1 schwebt offen bei 1,1-1,5 V (gemessen 06.10., lila).
// Ein abgerissener Pull-up muss als "los" gelten, nicht als Reku-Befehl.
#define LUNA_BREMSE_ADC1_ZU_V		0.5f
#define LUNA_BREMSE_ADC1_AUF_V		0.8f

// (volker, 10.10.2026) Fuer BEIDE Raeder 0,700 m: leicht zu merken und auf der
// sicheren Seite (GPS weiss 0,695 m -> Abriegelung real ~46,9 statt 47,25 km/h;
// Big Apple rot nur ~4 mm groesser). Feinabgleich in VESC Tool, 2-5 mm ab.
#define MCCONF_SI_WHEEL_DIAMETER		0.700

// --- Akku -----------------------------------------------------------
#define MCCONF_SI_BATTERY_CELLS			13
#define MCCONF_SI_BATTERY_AH			25.0
#define MCCONF_L_BATTERY_CUT_START		44.2
#define MCCONF_L_BATTERY_CUT_END		39.0

// --- Stromgrenzen ---------------------------------------------------
// Antrieb 40 A Motorstrom, 25 A Akkustrom (30-A-Sicherung).
// Rekuperation: Motorstrom bestimmt das Bremsmoment bei niedrigem Tempo,
// Akkustrom deckelt die Leistung bei hohem.
//
// 19.09.2026, Versuchsstand: Akkustrom von -10 auf -20 A angehoben, das
// sind 0,8 C auf den 25-Ah-Speicher und bei 50 V rund 1000 W. Ueber dem,
// was als dauerhafter Ladestrom vorgesehen war (0,4 C), aber als Versuch
// gewollt - herunterschalten geht jederzeit ueber die Displaystufe.
// Mit dem neuen Akku und neuem BMS ist C = 1 das Ziel, also -25 A.
//
// Risiko, bewusst in Kauf genommen: Schaltet ein BMS waehrend der
// Rekuperation wegen Ueberstrom ab, verliert der Controller die Last,
// waehrend er Strom hineindrueckt. Der Spannungsabschneider in
// mc_interface.c (53,5/54,5 V) begrenzt den normalen Betrieb, gegen ein
// ploetzlich abschaltendes BMS hilft er nicht.
#define MCCONF_L_CURRENT_MAX			60.0
#define MCCONF_L_CURRENT_MIN			-60.0
#define MCCONF_L_IN_CURRENT_MAX			25.0   // Pedelec 25: Firmware begrenzt auf 10 A (app_adc.c)
#define MCCONF_L_IN_CURRENT_MIN			-10.0
#define MCCONF_L_MAX_ABS_CURRENT		80.0   // Absolutgrenze, Notabschaltung

// --- Akkuseitige Grenzen, 20.09.2026 ---------------------------------
// l_in_current_min -10 A (20.09. nach der Bergfahrt von -12 gesenkt).
//
// Die Zellen vertragen 12 A - das sind 0,49 C und genau Samsungs
// Standard-Ladestrom, bei dem auch die 500-Zyklen-Angabe gemessen wird.
// Der Engpass ist das BMS: Enerpower nennt 5 A zum Laden, die baugleiche
// Klasse von 13S-20-A-Platinen schaltet bei rund 10 A ab.
//
// Dazu kommt Ueberschwingen: Gemessen wurden -15,8 A Spitze bei -12 A
// Einstellung, also rund 30 % darueber. Die Rampe faehrt in 0,3 s auf
// 60 A Motorstrom hoch, der Akkustrombegrenzer braucht Zeit zum
// Nachregeln. Mit -10 A liegt die erwartete Spitze bei etwa -13 A.
//
// l_in_current_max 25 A: BMS gibt 20 A dauerhaft und 30 A kurzzeitig ab,
// ueber 5000 km am Lishui gefahren. 25 A liegt dazwischen.
//
// Der MOTORstrom (l_current_min/-max, +-60 A) sieht das BMS nicht. Er
// bestimmt das Moment bei niedrigem Tempo, der Akkustrom deckelt die
// Leistung bei hohem. Deshalb bleibt die kraeftige Verzoegerung aus
// langsamer Fahrt erhalten, auch wenn der Akkudeckel niedrig steht.
//
// FUER DEN STARKEN AKKU (Molicel P50B, JBD 40-A-BMS): Der Ladestrom
// bleibt vorerst ebenfalls bei -10 A. Erst messen, dann erhoehen - das
// Ueberschwingen von rund 30 % gilt dort genauso, und die Zahlen fuer
// eine begruendete Anhebung haben wir noch nicht.
//
// Was dort spaeter moeglich waere, wenn die Messung es traegt:
//   Molicel P50B: Standard-Ladestrom 5 A je Zelle, bei 5P also 25 A Pack.
//   JBD 40 A: Lade- und Entladestrom gleich, Abschaltung bei rund 50 A.
//   Ladetemperatur -20 bis 60 C statt 0 bis 45 C beim Samsung.
// Zu aendern waere dann nur:
//   #define MCCONF_L_IN_CURRENT_MIN  -20.0
//   #define MCCONF_L_IN_CURRENT_MAX   40.0

// 20.09.2026 nach der zweiten Probefahrt:
// l_current_max 40 -> 60 A. Am Berg fehlte Leistung, Displaystufe 5 gibt
//   jetzt 60 A statt 40. Die Stufentabelle in luna_display_serial.c setzt
//   absolute Ampere und bezieht sie hierauf.
// l_current_min -40 -> -60 A. Noetig, damit die neuen Rekustufen 4 und 5
//   (50 und 60 A) ueberhaupt ankommen - sonst kappt die Firmware vorher.
// l_abs_current_max 60 -> 80 A. Die Notabschaltung muss oberhalb des
//   groessten regulaeren Wertes liegen, sonst loest sie bei Stufe 5 aus.
//   Der Flipsky 75100 vertraegt das; die Grenze schuetzt weiterhin vor
//   echten Kurzschluessen.

// --- Motortemperatur ------------------------------------------------
// Kein Fuehler verbaut. Ohne diese Zeile rechnet die Firmware aus dem
// offenen Eingang eine sinnlose Temperatur. Begrenzt wird dadurch nichts
// (gedrosselt wird nur nach oben), es spart aber einen Messwert, ueber
// den man sich sonst wundert.
#define MCCONF_M_MOTOR_TEMP_SENS_TYPE	TEMP_SENSOR_DISABLED

// =====================================================================
// App-Konfiguration
// =====================================================================

// APP_ADC_PAS: Gasgriff-Kanal und Drehmomentsensor zusammen.
// NIEMALS UART / PPM+UART / ADC+UART waehlen - die kapern den
// Display-UART SD3. Nunchuk ebenfalls nicht, der startet I2C auf
// denselben Pins PB10/PB11.
#define APPCONF_APP_TO_USE				APP_ADC_PAS

// --- ADC ------------------------------------------------------------
// Die Startspannungen sind der wichtigste Wert dieser ganzen Datei.
// Gemessen: ADC1 ruht bei 1,456 V, ADC2 bei 1,544 V. Mit der Vorgabe von
// 0,9 V gilt beides dauerhaft als betaetigt - das ergibt 26 % Gas und
// 31 % Drehmoment im Stillstand. Folge: Unterstuetzung am Stufenanschlag,
// sobald sich die Pedale drehen, und ms_without_power in app_adc.c kommt
// nie ueber null, wodurch der Antrieb gesperrt bleibt.
#define APPCONF_ADC_CTRL_TYPE			ADC_CTRL_TYPE_CURRENT
#define APPCONF_ADC_VOLTAGE_START		1.6
#define APPCONF_ADC_VOLTAGE_END			3.0
#define APPCONF_ADC_VOLTAGE2_START		1.6
#define APPCONF_ADC_VOLTAGE2_END		3.0
#define APPCONF_ADC_SAFE_START			true
#define APPCONF_ADC_MULTI_ESC			false
#define APPCONF_ADC_UPDATE_RATE_HZ		500

// --- PAS ------------------------------------------------------------
// PEDAL_RPM_END ist im Drehmomentmodus die Bezugstrittfrequenz fuer die
// Kennlinie in app_pas.c: Bei diesem Wert und vollem Pedaldruck kommt die
// volle Stufenleistung heraus, darunter entsprechend weniger. Kleiner =
// bissiger, groesser = zahmer. 60 wurde am 19.09. erfahren und fuer gut
// befunden - vergleichbar mit dem Lishui.
//
// RAMP_TIME_POS 0,6 -> 0,3 s (20.09.): Die Unterstuetzung soll beim
// Antreten schneller am Stufenwert stehen. Das Anfahren bleibt weich,
// weil die Normierung auf die Trittfrequenz die Hoehe begrenzt - die
// Rampe bestimmt nur, wie schnell der erlaubte Wert erreicht wird.
#define APPCONF_PAS_CTRL_TYPE			PAS_CTRL_TYPE_TORQUE
#define APPCONF_PAS_SENSOR_TYPE			PAS_SENSOR_TYPE_QUADRATURE
#define APPCONF_PAS_PEDAL_RPM_START		10.0
#define APPCONF_PAS_PEDAL_RPM_END		60.0
#define APPCONF_PAS_MAGNETS				24
#define APPCONF_PAS_CURRENT_SCALING		1.0
#define APPCONF_PAS_RAMP_TIME_POS		0.3
#define APPCONF_PAS_RAMP_TIME_NEG		0.3
#define APPCONF_PAS_UPDATE_RATE_HZ		500

// Servo-Ausgang muss aus bleiben, sonst Kollision mit dem PAS-Pin PB6.
#define APPCONF_SERVO_OUT_ENABLE		false

// (volker, 20.09.2026) Permanenten UART abschalten.
// main.c:301 ruft app_uartcomm_start(UART_PORT_BUILTIN) bedingungslos und
// legt PC10 und PC11 auf UART4-Alternativfunktion - und zwar NACH
// hw_init_gpio(). Der Lichtausgang auf PC11 wurde dadurch wieder zum
// UART4-Eingang, palClearPad blieb wirkungslos und der Pin las dauerhaft 1.
// Beim Bremseingang auf PC10 fiel es nicht auf, weil ein Pin in
// Alternativfunktion ueber das Eingangsregister weiterhin lesbar ist.
// Mit false setzt app_uartcomm_configure() die Pins wieder frei.
// Gebraucht wird dieser UART hier nicht - der Stecker traegt Bremse und
// Licht.
#define APPCONF_PERMANENT_UART_ENABLED	false

#endif /* CONF_EBIKE_H_ */
