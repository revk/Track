// Track logged
// Copyright (c) 2020 Adrian Kennard, Andrews & Arnold Limited, see LICENSE file (GPL)

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <popt.h>
#include <err.h>
#include <malloc.h>
#include <time.h>
#include <sqllib.h>
#include <mosquitto.h>
#include <math.h>

#define MQTTCONF "/etc/.mqtt.conf"      // Default MQTT config file if exists and readable
// Config file format is list of hosts (if not host specified in command then uses first one in config file, else localhost)
// First line is [hostname]
// Then keywords like username: and the username. You can set username, password, port, cafile

int debug = 0;
const char *sqlhostname = NULL;
const char *sqldatabase = "gps";
const char *sqlusername = NULL;
const char *sqlpassword = NULL;
const char *sqlconffile = NULL;
const char *sqltrack = "track";

int
main (int argc, const char *argv[])
{
   const char *mqtthostname = NULL;
   const char *mqttusername = NULL;
   const char *mqttpassword = NULL;
   const char *mqttappname = "Track";
   const char *mqttid = NULL;
   const char *mqttconf = NULL;
   const char *mqttcafile = NULL;
   int mqttport = 0;
   {                            // POPT
      poptContext optCon;       // context for parsing command-line options
      const struct poptOption optionsTable[] = {
	      /* *INDENT-OFF* */
         {"sql-conffile", 'c', POPT_ARG_STRING, &sqlconffile, 0, "SQL conf file", "filename"},
         {"sql-hostname", 'H', POPT_ARG_STRING, &sqlhostname, 0, "SQL hostname", "hostname"},
         {"sql-database", 'd', POPT_ARG_STRING | POPT_ARGFLAG_SHOW_DEFAULT, &sqldatabase, 0, "SQL database", "db"},
         {"sql-username", 'U', POPT_ARG_STRING, &sqlusername, 0, "SQL username", "name"},
         {"sql-password", 'P', POPT_ARG_STRING, &sqlpassword, 0, "SQL password", "pass"},
         {"sql-table", 't', POPT_ARG_STRING | POPT_ARGFLAG_SHOW_DEFAULT, &sqltrack, 0, "SQL log table", "table"},
         {"sql-debug", 'v', POPT_ARG_NONE, &sqldebug, 0, "SQL Debug"},
         {"mqtt-config", 'c', POPT_ARG_STRING, &mqttconf, 0, "MQTT config", "filename"},
         {"mqtt-hostname", 'h', POPT_ARG_STRING, &mqtthostname, 0, "MQTT hostname", "hostname"},
         {"mqtt-username", 'u', POPT_ARG_STRING, &mqttusername, 0, "MQTT username", "username"},
         {"mqtt-password", 'p', POPT_ARG_STRING, &mqttpassword, 0, "MQTT password", "password"},
         {"mqtt-ca", 'C', POPT_ARG_STRING, &mqttcafile, 0, "MQTT CA", "filename"},
         {"mqtt-port", 0, POPT_ARG_INT, &mqttport, 0, "MQTT port", "port"},
         {"mqtt-id", 0, POPT_ARG_STRING, &mqttid, 0, "MQTT id", "id"},
         {"mqtt-appname", 'a', POPT_ARG_STRING | POPT_ARGFLAG_SHOW_DEFAULT, &mqttappname, 0, "MQTT appname", "appname"},
         {"debug", 'V', POPT_ARG_NONE, &debug, 0, "Debug"},
         POPT_AUTOHELP {}
	    /* *INDENT-ON* */
      };
      optCon = poptGetContext (NULL, argc, argv, optionsTable, 0);
      int c;
      if ((c = poptGetNextOpt (optCon)) < -1)
         errx (1, "%s: %s\n", poptBadOption (optCon, POPT_BADOPTION_NOALIAS), poptStrerror (c));
      if (poptPeekArg (optCon))
      {
         poptPrintUsage (optCon, stderr, 0);
         return -1;
      }
      poptFreeContext (optCon);
   }
   if (!mqttconf && !access (MQTTCONF, R_OK))
      mqttconf = MQTTCONF;
   if (mqttconf)
   {
      FILE *f = fopen (mqttconf, "r");
      if (!f)
         err (1, "Cannot open %s", mqttconf);
      char line[1000];
      int skip = 1;
      while (fgets (line, sizeof (line), f))
      {
         char *v = line + strlen (line);
         while (v > line && v[-1] < ' ')
            v--;
         *v = 0;
         if (*line == '[' && (v = strchr (line, ']')))
         {                      // Host name (pick first if no host specified)
            if (!skip)
               break;           // Done
            *v = 0;
            skip = 0;
            if (!mqtthostname)
               mqtthostname = strdup (line + 1);
            else
               skip = strcasecmp (line + 1, mqtthostname);
         }
         if (skip)
            continue;
         v = strchr (line, ':');
         if (!v)
            continue;
         *v++ = 0;
         if (!mqttusername && !strcasecmp (line, "username"))
            mqttusername = strdup (v);
         else if (!mqttpassword && !strcasecmp (line, "password"))
            mqttpassword = strdup (v);
         else if (!mqttcafile && !strcasecmp (line, "cafile"))
            mqttcafile = strdup (v);
         else if (!mqttport && !strcasecmp (line, "port"))
            mqttport = atoi (v);
      }
      fclose (f);
   }
   SQL sql;
   if (debug)
      fprintf (stderr, "Connecting to %s port %d\n", mqtthostname ? : "localhost", mqttport ? : mqttcafile ? 8883 : 1883);
   int e = mosquitto_lib_init ();
   if (e)
      errx (1, "MQTT init failed %s", mosquitto_strerror (e));
   struct mosquitto *mqtt = mosquitto_new (mqttid, 1, NULL);
   if (mqttusername)
   {
      e = mosquitto_username_pw_set (mqtt, mqttusername, mqttpassword);
      if (e)
         errx (1, "MQTT auth failed %s", mosquitto_strerror (e));
   }
   void connect (struct mosquitto *mqtt, void *obj, int rc)
   {
      obj = obj;
      rc = rc;
      char *sub = NULL;
      asprintf (&sub, "state/%s/#", mqttappname);       // Note, picks up state 0 and any keep alive messages
      int e = mosquitto_subscribe (mqtt, NULL, sub, 0);
      if (e)
         errx (1, "MQTT subscribe failed %s (%s)", mosquitto_strerror (e), sub);
      if (debug)
         warnx ("MQTT Sub %s", sub);
      free (sub);
      asprintf (&sub, "info/%s/#", mqttappname);
      e = mosquitto_subscribe (mqtt, NULL, sub, 0);
      if (e)
         errx (1, "MQTT subscribe failed %s (%s)", mosquitto_strerror (e), sub);
      if (debug)
         warnx ("MQTT Sub %s", sub);
      free (sub);
   }
   void disconnect (struct mosquitto *mqtt, void *obj, int rc)
   {
      obj = obj;
      rc = rc;
   }
   time_t txn = 0;
   void message (struct mosquitto *mqtt, void *obj, const struct mosquitto_message *msg)
   {
      obj = obj;
      char *topic = strdupa (msg->topic);
      if (!msg->payloadlen)
      {
         warnx ("No payload %s", topic);
         return;
      }
      char *type = NULL;
      char *tag = NULL;
      char *app = strchr (topic, '/');
      if (app)
      {
         *app++ = 0;
         tag = strchr (app, '/');
         if (tag)
         {
            *tag++ = 0;
            type = strchr (tag, '/');
            if (type)
               *type++ = 0;
         }
      }
      if (!tag)
      {
         warnx ("No tag %s", topic);
         return;
      }
      char *val = malloc (msg->payloadlen + 1);
      memcpy (val, msg->payload, msg->payloadlen);
      val[msg->payloadlen] = 0;
      if (type && !strcmp (type, "Data"))
      {
         char when[30] = "";
         float ax,
           ay,
           gz,
           lat,
           lon,
           speed;
         if (sscanf (val, "%22s %f/%f/%f %f/%f/%f", when, &ax, &ay, &gz, &lat, &lon, &speed) != 7)
            fprintf (stderr, "Unexpected %s\n", val);
         else
         {
            if (isnan (ax))
               ax = 0;          // Sanity
            if (isnan (ay))
               ay = 0;
            if (isnan (gz))
               gz = 0;
            if (isnan (lat))
               lat = 0;
            if (isnan (lon))
               lon = 0;
            if (isnan (speed))
               speed = 0;
            if (!txn)
            {
               txn = time (0) + 10;
               sql_transaction (&sql);
            }
            sql_safe_query_free (&sql,
                                 sql_printf ("REPLACE INTO `%S` SET device=%#s,utc=%#.21s,ax=%f,ay=%f,gz=%f,lat=%f,lon=%f,speed=%f",
                                             sqltrack, tag, when, ax, ay, gz, lat, lon, speed));
            if (txn && txn < time (0))
            {
               txn = 0;
               sql_safe_commit (&sql);
            }
         }
      }
      free (val);
   }
   if (mqttcafile && (e = mosquitto_tls_set (mqtt, mqttcafile, NULL, NULL, NULL, NULL)))
      warnx ("MQTT cert failed (%s) %s", mqttcafile, mosquitto_strerror (e));
   mosquitto_connect_callback_set (mqtt, connect);
   mosquitto_disconnect_callback_set (mqtt, disconnect);
   mosquitto_message_callback_set (mqtt, message);
   e = mosquitto_connect (mqtt, mqtthostname ? : "localhost", mqttport ? : mqttcafile ? 8883 : 1883, 60);
   if (e)
      errx (1, "MQTT connect failed (%s) %s", mqtthostname, mosquitto_strerror (e));
   sql_real_connect (&sql, sqlhostname, sqlusername, sqlpassword, sqldatabase, 0, NULL, 0, 1, sqlconffile);
   e = mosquitto_loop_forever (mqtt, -1, 1);
   if (e)
      errx (1, "MQTT loop failed %s", mosquitto_strerror (e));
   mosquitto_destroy (mqtt);
   mosquitto_lib_cleanup ();
   sql_close (&sql);
   return 0;
}
