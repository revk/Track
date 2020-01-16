// GPS/G-Force high speed logger for go-carting
// Copyright (c) 2020 Adrian Kennard, Andrews & Arnold Limited, see LICENSE file (GPL)
static const char TAG[] = "Track";

#include "revk.h"
#include <driver/i2c.h>
#include <driver/uart.h>
#include <math.h>
#include "esp_sntp.h"
extern void hmac_sha256 (const uint8_t * key, size_t key_len, const uint8_t * data, size_t data_len, uint8_t * mac);

// Commands:-
// test         Toggle test mode which sends to mobile even if on MQTT
// udp          Send a raw UDP payload as if received via mobile (encrypted)
// contrast     Set OLED contrast now
// status       Send a fix status info message
// resend       Resend from date/time
// fix          Do a fix update now
// time         Set the time manually (for testing)
// lat, lon, alt, course, speed, hdop, pdop, vdop, hepe, vepe: Manually force a value, for testing
// gpstx        Send a message to the GPS
// attx         Send a message to the modem
// locus        Get status of LOCUS log
// dump         Dump LOCUS log
// erase        Erase LOCUS log
// hot          Do hot restart of GPS
// warm         Do warn restart of GPS
// cold         Do cold start of GPS
// reset        Do full reset cold start of GPS
// sleep        Put GPS to sleep
// version      Get GPS version

#define settings	\
	bl(gpsdebug,N,		GPS debug logging)	\
	s8(gpspps,-1,		GPS PPS GPIO)	\
	s8(gpsuart,1,		GPS UART ID)	\
	s8(gpsrx,-1,		GPS Rx GPIO)	\
	s8(gpstx,-1,		GPS Tx GPIO)	\
	s8(gpsfix,-1,		GPS Fix GPIO)	\
	s8(gpsen,-1,		GPS EN GPIO)	\
        u32(gpsbaud,115200,	GPS Baud)	\
	u32(fixms,100,		GPS fix rate (ms)) \
	bl(fixdebug,N,		GPS Fix debug log)	\
	u8(gport,0,		G-Force I2C port) \
	s8(gscl,-1,		G-Force SCL)	\
	s8(gsda,-1,		G-Force SDA)	\
	s8(gint,-1,		G-Force INT)	\
	u8(arate,0,		G-Force accel rate 0-3) \
	u8(ascale,0,		G-Force accel scale 0-3) \
	u8(gscale,0,		G-Force dyro scale 0-3) \
	u8(gaddress,0x68,	G-Force I2C address) \
	b(usefifo,N,		G-Force use FIFO) \
	bl(gdebug,N,		G-Force debug logging)	\
	b(navstar,Y,		GPS track NAVSTAR GPS)	\
	b(glonass,Y,		GPS track GLONASS GPS)	\
	b(galileo,Y,		GPS track GALILEO GPS)	\
	b(waas,Y,		GPS enable WAAS)	\
	b(sbas,Y,		GPS enable SBAS)	\
	b(qzss,N,		GPS enable QZSS)	\
	b(aic,Y,		GPS enable AIC)	\
	b(easy,Y,		GPS enable Easy)	\
	b(ecef,N,		GPS ECEF tracking)	\
	b(walking,N,		GPS Walking mode)	\
	b(flight,N,		GPS Flight mode)	\
	b(balloon,N,		GPS Balloon mode)	\
	u16(mtu,1488,		UDP MTU)	\

#define u32(n,d,t)	uint32_t n;
#define u16(n,d,t)	uint16_t n;
#define s8(n,d,t)	int8_t n;
#define u8(n,d,t)	uint8_t n;
#define b(n,d,t) uint8_t n;
#define bl(n,d,t) uint8_t n;
#define h(n,t) uint8_t *n;
#define s(n,d,t) char * n;
settings
#undef u16
#undef u32
#undef s8
#undef u8
#undef bl
#undef b
#undef h
#undef s
// Current info values as at end of GGA processing
float speed = 0;
float bearing = 0;
float lat = 0;
float lon = 0;
float alt = 0;
int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;
int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;
int64_t ecefx = 0;
int64_t ecefy = 0;
int64_t ecefz = 0;
float gsep = 0;
float pdop = 0;
float hdop = 0;
float hepe = 0;
float vepe = 0;
float vdop = 0;
float course = 0;
float hepea = 0;                // Slower average
uint8_t sats = 0;
uint8_t gxgsv[3] = { };
uint8_t gngsa[3] = { };

uint8_t fixtype = 0;
uint8_t fixmode = 0;

int8_t mobile = 0;              // Mobile data on line
int8_t gotfix = 0;
int8_t lonforce = 0;
int8_t latforce = 0;
int8_t altforce = 0;
int8_t timeforce = 0;
int8_t speedforce = 0;
int8_t courseforce = 0;
int8_t hepeforce = 0;
int8_t vepeforce = 0;
int8_t hdopforce = 0;
int8_t pdopforce = 0;
int8_t vdopforce = 0;
int8_t sendinfo = 0;
uint8_t online = 0;
int gpserrors = 0;
int gpserrorcount = 0;
volatile int8_t gpsstarted = 0;
time_t moving = 0;
char iccid[22] = { };
char imei[22] = { };

uint8_t gok = 0;

float tempc = -999;

#define MINL	0.1
time_t gpszda = 0;              // Last ZDA

SemaphoreHandle_t cmd_mutex = NULL;
SemaphoreHandle_t ack_semaphore = NULL;
int pmtk = 0;                   // Waiting ack
SemaphoreHandle_t at_mutex = NULL;

void g_poll (void);

typedef struct datalog_s datalog_t;
struct datalog_s
{
   time_t base;
   // Data for each 1/10th of second
   float lat[10];
   float lon[10];
   float speed[10];
   int16_t ax[10];              // Accel
   int16_t ay[10];
   int16_t gx[10];              // Gyro
   int16_t gy[10];
   int16_t gz[10];
};
datalog_t *datalog = NULL;
int datalogi = 0,
   datalogo = 0;
#define DATALOGMAX 36000        // One hour recording

void
fixstatus (void)
{
   revk_info (TAG, "Sats %d (NAVSTAR %d/%d, GLONASS %d/%d, GALILEO %d/%d) %s %s hepe=%.1f vepe=%.1f", sats, gngsa[0], gxgsv[0],
              gngsa[1], gxgsv[1], gngsa[2], gxgsv[2], fixtype == 0 ? "Invalid" : fixtype ==
              1 ? "GNSS" : fixtype == 2 ? "DGPS" : fixtype == 6 ? "Estimated" : "?",
              fixmode == 1 ? "No fix" : fixmode == 2 ? "2D" : fixmode == 3 ? "3D" : "?", hepe, vepe);
}

uint32_t gpsbaudnow = 0;
      // Init UART for GPS
void
gps_connect (unsigned int baud)
{
   esp_err_t err;
   if (gpsbaudnow)
      uart_driver_delete (gpsuart);
   gpsbaudnow = baud;
   uart_config_t uart_config = {
      .baud_rate = baud,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
   };
   if ((err = uart_param_config (gpsuart, &uart_config)))
      revk_error (TAG, "UART param fail %s", esp_err_to_name (err));
   else if ((err = uart_set_pin (gpsuart, gpstx, gpsrx, -1, -1)))
      revk_error (TAG, "UART pin fail %s", esp_err_to_name (err));
   else if ((err = uart_driver_install (gpsuart, 1024, 0, 0, NULL, 0)))
      revk_error (TAG, "UART install fail %s", esp_err_to_name (err));
   uart_write_bytes (gpsuart, "\r\n\r\n", 4);
}

void
gps_cmd (const char *fmt, ...)
{                               // Send command to UART
   if (pmtk)
      xSemaphoreTake (ack_semaphore, 1000 * portTICK_PERIOD_MS);        // Wait for ACK from last command
   char s[100];
   va_list ap;
   va_start (ap, fmt);
   vsnprintf (s, sizeof (s) - 5, fmt, ap);
   va_end (ap);
   uint8_t c = 0;
   char *p;
   for (p = s + 1; *p; p++)
      c ^= *p;
   if (gpsdebug)
      revk_info ("gpstx", "%s", s);
   if (*s == '$')
      p += sprintf (p, "*%02X\r\n", c); // We allowed space
   xSemaphoreTake (cmd_mutex, portMAX_DELAY);
   uart_write_bytes (gpsuart, s, p - s);
   xSemaphoreGive (cmd_mutex);
   if (!strncmp (s, "$PMTK", 5))
   {
      xSemaphoreTake (ack_semaphore, 0);
      pmtk = atoi (s + 5);
   } else
      pmtk = 0;
}

const char *
app_command (const char *tag, unsigned int len, const unsigned char *value)
{
   if (!strcmp (tag, "wifi"))
   {                            // WiFi connected, but not need for SNTP as we have GPS
      if (gpszda)
         sntp_stop ();
      return "";
   }
   if (!strcmp (tag, "disconnect"))
   {
      return "";
   }
   if (!strcmp (tag, "connect"))
   {
      return "";
   }
   if (!strcmp (tag, "resend"))
   {                            // Resend log data
      return "";
   }
   if (!strcmp (tag, "time"))
   {
      if (!len)
         timeforce = 0;
      else
      {
         timeforce = 1;
         struct tm t = { };
         char *z = strptime ((char *) value, "%Y-%m-%d %H:%M:%S", &t);
         if (!z || !*z)
            t.tm_isdst = -1;
         struct timeval v = { };
         v.tv_sec = mktime (&t);
         settimeofday (&v, NULL);
      }
      return "";
   }
#define force(x) if (!strcmp (tag, #x)) { if (!len) x##force = 0; else { x##force = 1; x = strtof ((char *) value, NULL); } return ""; }
   force (lat);
   force (lon);
   force (alt);
   force (course);
   force (speed);
   force (hdop);
   force (pdop);
   force (vdop);
   force (hepe);
   force (vepe);
#undef force
   if (!strcmp (tag, "gpstx") && len)
   {                            // Send arbitrary GPS command (do not include *XX or CR/LF)
      gps_cmd ("%s", value);
      return "";
   }
   if (!strcmp (tag, "dump"))
   {
      gps_cmd ("$PMTK622,1");   // Dump log
      return "";
   }
   if (!strcmp (tag, "locus"))
   {
      gps_cmd ("$PMTK183");     // Log status
      return "";
   }
   if (!strcmp (tag, "erase"))
   {
      gps_cmd ("$PMTK184,1");   // Erase
      return "";
   }
   if (!strcmp (tag, "hot"))
   {
      gps_cmd ("$PMTK101");     // Hot start
      gpsstarted = 0;
      return "";
   }
   if (!strcmp (tag, "warm"))
   {
      gps_cmd ("$PMTK102");     // Warm start
      gpsstarted = 0;
      return "";
   }
   if (!strcmp (tag, "cold"))
   {
      gps_cmd ("$PMTK103");     // Cold start
      gpsstarted = 0;
      return "";
   }
   if (!strcmp (tag, "reset"))
   {
      gps_cmd ("$PMTK104");     // Full cold start (resets to default settings including Baud rate)
      revk_restart ("GPS has been reset", 1);
      return "";
   }
   if (!strcmp (tag, "sleep"))
   {
      gps_cmd ("$PMTK291,7,0,10000,1"); // Low power (maybe we need to drive EN pin?)
      return "";
   }
   if (!strcmp (tag, "version"))
   {
      gps_cmd ("$PMTK605");     // Version
      return "";
   }
   return NULL;
}

static void
gps_init (void)
{                               // Set up GPS
   if (gpsbaudnow != gpsbaud)
   {
      gps_cmd ("$PQBAUD,W,%d", gpsbaud);        // 
      gps_cmd ("$PMTK251,%d", gpsbaud); // Required Baud rate set
      sleep (1);
      gps_connect (gpsbaud);
   }
   gps_cmd ("$PMTK286,%d", aic ? 1 : 0);        // AIC
   gps_cmd ("$PMTK353,%d,%d,%d,0,0", navstar, glonass, galileo);
   gps_cmd ("$PMTK352,%d", qzss ? 0 : 1);       // QZSS (yes, 1 is disable)
   gps_cmd ("$PQTXT,W,0,1");    // Disable TXT
   gps_cmd ("$PQECEF,W,%d,1", ecef);    // Enable/Disable ECEF
   gps_cmd ("$PQEPE,W,1,1");    // Enable EPE
   gps_cmd ("$PMTK886,%d", balloon ? 3 : flight ? 2 : walking ? 1 : 0); // FR mode
   // Queries - responses prompt settings changes if needed
   gps_cmd ("$PMTK414");        // Q_NMEA_OUTPUT
   gps_cmd ("$PMTK400");        // Q_FIX
   gps_cmd ("$PMTK401");        // Q_DGPS
   gps_cmd ("$PMTK413");        // Q_SBAS
   gps_cmd ("$PMTK869,0");      // Query EASY
   if (fixdebug)
      gps_cmd ("$PMTK605");     // Q_RELEASE
   gpsstarted = 1;
}

static void
nmea (char *s)
{
   if (gpsdebug)
      revk_info ("gpsrx", "%s", s);
   if (!s || *s != '$' || !s[1] || !s[2] || !s[3])
      return;
   char *f[50];
   int n = 0;
   s++;
   while (n < sizeof (f) / sizeof (*f))
   {
      f[n++] = s;
      while (*s && *s != ',')
         s++;
      if (!*s || *s != ',')
         break;
      *s++ = 0;
   }
   if (!n)
      return;
   if (!gpsstarted && *f[0] == 'G' && !strcmp (f[0] + 2, "GGA") && (esp_timer_get_time () > 10000000 || !revk_offline ()))
      gpsstarted = -1;          // Time to send init
   if (!strcmp (f[0], "PMTK001") && n >= 3)
   {                            // ACK
      int tag = atoi (f[1]);
      if (pmtk && pmtk == tag)
      {                         // ACK received
         xSemaphoreGive (ack_semaphore);
         pmtk = 0;
      }
      int ok = atoi (f[2]);
      if (ok == 1)
         revk_error (TAG, "PMTK%d unsupported", tag);
      else if (ok == 2)
         revk_error (TAG, "PMTK%d failed", tag);
      return;
   }
   if (!strcmp (f[0], "PQTXT"))
      return;                   // ignore
   if (!strcmp (f[0], "PQECEF"))
      return;                   // ignore
   if (*f[0] == 'G' && !strcmp (f[0] + 2, "GLL"))
      return;                   // ignore
   if (*f[0] == 'G' && !strcmp (f[0] + 2, "RMC"))
      return;                   // ignore
   if (!strcmp (f[0], "PMTK010"))
      return;                   // Started, happens at end of init anyway
   if (!strcmp (f[0], "PMTK011"))
      return;                   // Message! Ignore
   if (!strcmp (f[0], "PQEPE") && n >= 3)
   {                            // Estimated position error
      if (!hepeforce)
      {
         hepe = strtof (f[1], NULL);
         if (hepe)
         {
            if (hepe > hepea)
               hepea = hepe;
            else
               hepea = (hepe + hepea) / 2;
         }
      }
      if (!vepeforce)
         vepe = strtof (f[2], NULL);
      return;
   }
   if (!strcmp (f[0], "ECEFPOSVEL") && n >= 7)
   {
      if (strlen (f[2]) > 6 && strlen (f[3]) > 6 && strlen (f[4]) > 6)
      {
         char *s,
          *p;
         if (*(s = p = f[2]) == '-')
            p++;
         ecefx = strtoll (p, &p, 0) * 1000000LL;
         if (*p++ == '.')
            ecefx += strtoll (p, NULL, 0);
         if (*s == '-')
            ecefx *= -1;
         if (*(s = p = f[3]) == '-')
            p++;
         ecefy = strtoll (p, &p, 0) * 1000000LL;
         if (*p++ == '.')
            ecefy += strtoll (p, NULL, 0);
         if (*s == '-')
            ecefy *= -1;
         if (*(s = p = f[4]) == '-')
            p++;
         ecefz = strtoll (p, &p, 0) * 1000000LL;
         if (*p++ == '.')
            ecefz += strtoll (p, NULL, 0);
         if (*s == '-')
            ecefz *= -1;
         if (ecef && (ecefx || ecefy || ecefz))
            gotfix = 1;
         //revk_info (TAG, "%lld %lld %lld %s %s %s", ecefx, ecefy, ecefz, f[2], f[3], f[4]);
      }
      return;
   }
   if (!strcmp (f[0], "PMTK869") && n >= 4)
   {                            // Set EASY
      if (atoi (f[1]) == 2 && atoi (f[2]) != easy)
      {
         if (fixdebug)
            revk_info (TAG, "Setting EASY %s  (%s days)", easy ? "on" : "off", f[3]);
         gps_cmd ("$PMTK869,1,%d", easy ? 1 : 0);
      }
      return;
   }
   if (!strcmp (f[0], "PMTK513") && n >= 2)
   {                            // Set SBAS
      if (atoi (f[1]) != sbas)
      {
         if (fixdebug)
            revk_info (TAG, "Setting SBAS %s", sbas ? "on" : "off");
         gps_cmd ("$PMTK313,%d", sbas ? 1 : 0);
      }
      return;
   }
   if (!strcmp (f[0], "PMTK501") && n >= 2)
   {                            // Set DGPS
      if (atoi (f[1]) != ((sbas || waas) ? 2 : 0))
      {
         if (fixdebug)
            revk_info (TAG, "Setting DGPS %s", (sbas || waas) ? "on" : "off");
         gps_cmd ("$PMTK301,%d", (sbas || waas) ? 2 : 0);
      }
      return;
   }
   if (!strcmp (f[0], "PMTK500") && n >= 2)
   {                            // Fix rate
      if (atoi (f[1]) != fixms)
      {
         if (fixdebug)
            revk_info (TAG, "Setting fix rate %dms", fixms);
         gps_cmd ("$PMTK220,%d", fixms);
      }
      return;
   }
   if (!strcmp (f[0], "PMTK705") && n >= 2)
   {
      revk_info (TAG, "GPS version %s", f[1]);
      return;
   }
   if (!strcmp (f[0], "PMTK514") && n >= 2)
   {
      unsigned int rates[19] = { };
      rates[2] = (1000 / fixms ? : 1);  // VTG
      rates[3] = 1;             // GGA
      rates[4] = (10000 / fixms ? : 1); // GSA
      rates[5] = (10000 / fixms ? : 1); // GSV
      rates[17] = (10000 / fixms ? : 1);        // ZDA
      int q;
      for (q = 0; q < sizeof (rates) / sizeof (rates) && rates[q] == (1 + q < n ? atoi (f[1 + q]) : 0); q++);
      if (q < sizeof (rates) / sizeof (rates))
      {                         // Set message rates
         if (fixdebug)
            revk_info (TAG, "Setting message rates");
         gps_cmd ("$PMTK314,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                  rates[0], rates[1], rates[2], rates[3], rates[4], rates[5], rates[6], rates[7], rates[8], rates[9], rates[10],
                  rates[11], rates[12], rates[13], rates[14], rates[15], rates[16], rates[17], rates[18], rates[19]);
      }
      return;
   }
   if (*f[0] == 'G' && !strcmp (f[0] + 2, "GGA") && n >= 14)
   {                            // Fix: $GPGGA,093644.000,5125.1569,N,00046.9708,W,1,09,1.06,100.3,M,47.2,M,,
      if (strlen (f[1]) >= 10)
      {
         fixtype = atoi (f[6]);
         int s = atoi (f[7]);
         if (s != sats)
         {
            sats = s;
            if (fixdebug)
               fixstatus ();
         }
         if (!altforce)
            alt = strtof (f[9], NULL);
         gsep = strtof (f[10], NULL);
         if (strlen (f[2]) >= 9 && strlen (f[4]) >= 10)
         {
            if (!latforce)
               lat = ((f[2][0] - '0') * 10 + f[2][1] - '0' + strtof (f[2] + 2, NULL) / 60) * (f[3][0] == 'N' ? 1 : -1);
            if (!lonforce)
               lon =
                  ((f[4][0] - '0') * 100 + (f[4][1] - '0') * 10 + f[4][2] - '0' + strtof (f[4] + 3, NULL) / 60) * (f[5][0] ==
                                                                                                                   'E' ? 1 : -1);
            if (!hdopforce)
               hdop = strtof (f[8], NULL);
            if (!ecef && (lat || lon))
               gotfix = 1;
         }
      }
      g_poll ();
      // Create log entry
      // TODO
      return;
   }
   if (*f[0] == 'G' && !strcmp (f[0] + 2, "ZDA") && n >= 5)
   {                            // Time: $GPZDA,093624.000,02,11,2019,,
      gpserrors = gpserrorcount;
      gpserrorcount = 0;
      if (strlen (f[1]) == 10 && !timeforce && atoi (f[4]) > 2000)
      {
         struct tm t = { };
         struct timeval v = { };
         t.tm_year = atoi (f[4]) - 1900;
         t.tm_mon = atoi (f[3]) - 1;
         t.tm_mday = atoi (f[2]);
         t.tm_hour = (f[1][0] - '0') * 10 + f[1][1] - '0';
         t.tm_min = (f[1][2] - '0') * 10 + f[1][3] - '0';
         t.tm_sec = (f[1][4] - '0') * 10 + f[1][5] - '0';
         v.tv_usec = atoi (f[1] + 7) * 1000;
         v.tv_sec = mktime (&t);
         if (!gpszda)
         {
            sntp_stop ();
            if (fixdebug)
               revk_info (TAG, "Set clock %s-%s-%s %s", f[4], f[3], f[2], f[1]);
         }
         gpszda = v.tv_sec;
         settimeofday (&v, NULL);
      }
      return;
   }
   if (*f[0] == 'G' && !strcmp (f[0] + 2, "VTG") && n >= 10)
   {
      if (!courseforce)
         course = strtof (f[1], NULL);
      if (!speedforce)
         speed = strtof (f[7], NULL);
      return;
   }
   if (*f[0] == 'G' && !strcmp (f[0] + 2, "GSA") && n >= 18)
   {
      fixmode = atoi (f[2]);
      if (!pdopforce)
         pdop = strtof (f[15], NULL);
      if (!vdopforce)
         vdop = strtof (f[17], NULL);
      if (n >= 19)
      {
         int s = atoi (f[18]);
         if (s && s <= sizeof (gngsa) / sizeof (*gngsa))
         {                      // Count active satellites
            int q = 0;
            for (int p = 0; p < 12; p++)
               if (*f[3 + p])
                  q++;
            gngsa[s - 1] = q;
         }
      }
      return;
   }
   if (*f[0] == 'G' && !strcmp (f[0] + 2, "GSV") && n >= 4)
   {
      int n = atoi (f[3]);
      if (f[0][1] == 'P')
         gxgsv[0] = n;
      else if (f[0][1] == 'L')
         gxgsv[1] = n;
      else if (f[0][1] == 'A')
         gxgsv[2] = n;
      return;
   }
#if 0
   if (!gpsdebug)
   {                            // Report unknown
      for (int q = 1; q < n; q++)
         f[q][-1] = ',';
      revk_error ("gpsrx", "$%s", f[0]);
   }
#endif
}

void
datalog_task (void *z)
{
	while(1)
	{
		sleep(1);
		// Send log data
		// TODO
	}
}

void
nmea_task (void *z)
{
   uint8_t buf[1000],
    *p = buf;
   uint64_t timeout = esp_timer_get_time () + 10000000;
   while (1)
   {
      // Get line(s), the timeout should mean we see one or more whole lines typically
      int l = uart_read_bytes (gpsuart, p, buf + sizeof (buf) - p, 10 / portTICK_PERIOD_MS);
      if (l <= 0)
      {
         if (timeout && timeout < esp_timer_get_time ())
         {
            gpsstarted = 0;
            static int rate = 0;
            const uint32_t rates[] = { 4800, 9600, 14400, 19200, 38400, 57600, 115200 };
            rate++;
            if (rate == sizeof (rates) / sizeof (*rates))
               rate = 0;
            if (!rate && gpsen >= 0)
            {                   // Reset GPS
               gpio_set_level (gpsen, 0);
               sleep (1);
               gpio_set_level (gpsen, 1);
            }
            gps_connect (rates[rate]);
            revk_info (TAG, "GPS silent, trying %d", gpsbaudnow);
            timeout = esp_timer_get_time () + 2000000 + fixms;
         }
         continue;
      }
      uint8_t *e = p + l;
      p = buf;
      while (p < e)
      {
         uint8_t *l = p;
         while (l < e && *l >= ' ')
            l++;
         if (l == e)
            break;
         if (*p == '$' && (l - p) >= 4 && l[-3] == '*' && isxdigit (l[-2]) && isxdigit (l[-1]))
         {
            // Checksum
            uint8_t c = 0,
               *x;
            for (x = p + 1; x < l - 3; x++)
               c ^= *x;
            if (((c >> 4) > 9 ? 7 : 0) + (c >> 4) + '0' != l[-2] || ((c & 0xF) > 9 ? 7 : 0) + (c & 0xF) + '0' != l[-1])
            {
               revk_error (TAG, "[%.*s] (%02X)", l - p, p, c);
               gpserrorcount++;
            } else
            {                   // Process line
               timeout = esp_timer_get_time () + 60000000 + fixms;
               l[-3] = 0;
               nmea ((char *) p);
            }
         } else if (l > p)
            revk_error (TAG, "[%.*s]", l - p, p);
         while (l < e && *l < ' ')
            l++;
         p = l;
      }
      if (p < e && (e - p) < sizeof (buf))
      {                         // Partial line
         memmove (buf, p, e - p);
         p = buf + (e - p);
         continue;
      }
      p = buf;                  // Start from scratch
   }
}

void
g_write (uint8_t address, uint8_t value)
{
   i2c_cmd_handle_t t = i2c_cmd_link_create ();
   i2c_master_start (t);
   i2c_master_write_byte (t, (gaddress << 1) | I2C_MASTER_WRITE, true);
   i2c_master_write_byte (t, address, true);
   i2c_master_write_byte (t, value, true);
   i2c_master_stop (t);
   esp_err_t e = i2c_master_cmd_begin (gport, t, 10 / portTICK_PERIOD_MS);
   i2c_cmd_link_delete (t);
   if (e)
      revk_error (TAG, "I2C write error: %s", esp_err_to_name (e));
}

int
g_read (uint8_t address)
{
   uint8_t v = 0;
   i2c_cmd_handle_t t = i2c_cmd_link_create ();
   i2c_master_start (t);
   i2c_master_write_byte (t, (gaddress << 1) | I2C_MASTER_WRITE, true);
   i2c_master_write_byte (t, address, true);
   i2c_master_start (t);
   i2c_master_write_byte (t, (gaddress << 1) | I2C_MASTER_READ, true);
   i2c_master_read_byte (t, &v, I2C_MASTER_LAST_NACK);
   i2c_master_stop (t);
   esp_err_t e = i2c_master_cmd_begin (gport, t, 10 / portTICK_PERIOD_MS);
   i2c_cmd_link_delete (t);
   if (e)
   {
      revk_error (TAG, "I2C read error: %s", esp_err_to_name (e));
      return -1;
   }
   return v;
}

int
g_readn (uint8_t address, void *addr, uint16_t n)
{
   i2c_cmd_handle_t t = i2c_cmd_link_create ();
   i2c_master_start (t);
   i2c_master_write_byte (t, (gaddress << 1) | I2C_MASTER_WRITE, true);
   i2c_master_write_byte (t, address, true);
   i2c_master_start (t);
   i2c_master_write_byte (t, (gaddress << 1) | I2C_MASTER_READ, true);
   if (n > 1)
      i2c_master_read (t, addr, n - 1, I2C_MASTER_ACK);
   i2c_master_read_byte (t, addr + n - 1, I2C_MASTER_LAST_NACK);
   i2c_master_stop (t);
   esp_err_t e = i2c_master_cmd_begin (gport, t, 10 / portTICK_PERIOD_MS);
   i2c_cmd_link_delete (t);
   if (e)
   {
      revk_error (TAG, "I2C read error: %s", esp_err_to_name (e));
      return -1;
   }
   return n;
}

void
g_init (void)
{                               // Init G sensor
   if (i2c_driver_install (gport, I2C_MODE_MASTER, 0, 0, 0))
   {
      ESP_LOGE (TAG, "I2C install fail");
      return;
   }
   i2c_config_t config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = gsda,
      .scl_io_num = gscl,
      .sda_pullup_en = true,
      .scl_pullup_en = true,
      .master.clk_speed = 400000,
   };
   if (i2c_param_config (gport, &config))
   {
      i2c_driver_delete (gport);
      revk_error (TAG, "I2C config fail");
      gscl = gsda = -1;
      return;
   }
   i2c_set_timeout (gport, 400000);
   int v = g_read (117),
      try = 20;
   while (v < 0 && try--)
   {
      sleep (1);
      v = g_read (117);
   }
   if (v < 0)
   {
      revk_error (TAG, "GY-521 No reply");
      return;
   }
   if ((v & 0x7E) != (gaddress & 0x7E))
   {
      revk_error (TAG, "GY-521 ID Wrong (%02X)", v);
      return;
   }
   g_write (107, 0);            // Wake up, 8MHz clock
   g_write (25, 7);             // 1MHz ?
   g_write (27, (gscale & 3) << 3);
   g_write (28, (ascale & 3) << 3);
   if (usefifo)
   {
      g_write (35, 0x78);       // FIFO_EN Gyro and accel, all axis
      g_write (106, 0x40);      // USER_CTTL FIFO_EN
#if 0                           // Slow
      g_write (108, (arate & 3) << 6);  // PWR_MGMT_2
      g_write (107, 0x28);      // Wake up, cycle and no temp
#endif
   }
   gok = 1;
}

void
g_poll (void)
{                               // Poll data from FiFO
   uint8_t data[12];
   if (!gok)
      return;                   // Non G-Force sensor
   void debug (void)
   {
      if (!gdebug)
         return;
      int16_t ax = (data[0] << 8) | data[1];
      int16_t ay = (data[2] << 8) | data[3];
      int16_t az = (data[4] << 8) | data[5];
      int16_t gx = (data[6] << 8) | data[7];
      int16_t gy = (data[8] << 8) | data[9];
      int16_t gz = (data[10] << 8) | data[11];
      revk_info (TAG, "Data %d/%d/%d %d/%d/%d", ax, ay, az, gx, gy, gz);
   }
   if (usefifo)
   {
      char b[2];
      if (g_readn (114, b, sizeof (b)) < 0)
         return;
      int fifo = (b[0] << 8) + b[1];
      revk_info (TAG, "FIFO %d", fifo);
      fifo = (fifo / 12) * 12;  // Blocks of 12 bytes
      int v = 0;
      while (fifo)
      {
         for (int p = 0; p < 12; p++)
         {
            v = g_read (116);
            if (v < 0)
               break;
            data[p] = v;
         }
         if (v < 0)
            break;
         debug ();
         fifo -= 12;
      }
      if (fifo)
      {                         // FIFO reset
         revk_error (TAG, "FIFO reset");
         g_write (106, 0x00);
         g_write (106, 0x04);
         g_write (106, 0x40);
      }
      return;
   }
   if (g_readn (59, data, 6) < 0 || g_readn (67, data + 6, 6) < 0)
      return;
   debug ();
}

void
app_main ()
{
   cmd_mutex = xSemaphoreCreateMutex ();        // Shared command access
   vSemaphoreCreateBinary (ack_semaphore);      // GPS ACK mutex
   at_mutex = xSemaphoreCreateMutex (); // Shared command access
   revk_init (&app_command);
#define b(n,d,t) revk_register(#n,0,sizeof(n),&n,#d,SETTING_BOOLEAN);
#define bl(n,d,t) revk_register(#n,0,sizeof(n),&n,#d,SETTING_BOOLEAN|SETTING_LIVE);
#define h(n,t) revk_register(#n,0,0,&n,NULL,SETTING_BINARY|SETTING_HEX);
#define u32(n,d,t) revk_register(#n,0,sizeof(n),&n,#d,0);
#define u16(n,d,t) revk_register(#n,0,sizeof(n),&n,#d,0);
#define s8(n,d,t) revk_register(#n,0,sizeof(n),&n,#d,SETTING_SIGNED);
#define u8(n,d,t) revk_register(#n,0,sizeof(n),&n,#d,0);
#define s(n,d,t) revk_register(#n,0,0,&n,d,0);
   settings;
#undef u16
#undef u32
#undef s8
#undef u8
#undef bl
#undef b
#undef h
#undef s
   datalog = malloc (sizeof (datalog_t) * DATALOGMAX);
   if (!datalog)
      revk_error (TAG, "Failed to allocate log space");
   if (mtu > 1488)
      mtu = 1488;
   if (gpspps >= 0)
      gpio_set_direction (gpspps, GPIO_MODE_INPUT);
   if (gpsfix >= 0)
      gpio_set_direction (gpsfix, GPIO_MODE_INPUT);
   if (gpsen >= 0)
   {                            // Enable
      gpio_set_level (gpsen, 1);
      gpio_set_direction (gpsen, GPIO_MODE_OUTPUT);
   }
   gps_connect (gpsbaud);
   if (gpsrx >= 0)
      revk_task ("NMEA", nmea_task, NULL);
   if(datalog)
	   revk_task("Datalog",datalog_task,NULL);
   if (gscl >= 0 && gsda >= 0)
      g_init ();
   while (1)
   {
      sleep (1);
      if (gpstx >= 0 && gpsrx >= 0 && gpsstarted < 0)
         gps_init ();
      if (gpstx < 0 || gpsrx < 0)
      {                         // Logging just G forces
         g_poll ();
      }
   }
}
