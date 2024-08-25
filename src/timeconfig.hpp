#include "RTClib.h"
#include <WebSerialLite.h>
#include <time.h>

extern RTC_PCF8563 rtc;

int64_t getTimestamp(int year, int mon, int mday, int hour, int min, int sec)
{
  const uint16_t ytd[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};                /* Anzahl der Tage seit Jahresanfang ohne Tage des aktuellen Monats und ohne Schalttag */
  int leapyears = ((year - 1) - 1968) / 4 - ((year - 1) - 1900) / 100 + ((year - 1) - 1600) / 400; /* Anzahl der Schaltjahre seit 1970 (ohne das evtl. laufende Schaltjahr) */
  int64_t days_since_1970 = (year - 1970) * 365 + leapyears + ytd[mon - 1] + mday - 1;
  if ((mon > 2) && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)))
    days_since_1970 += 1; /* +Schalttag, wenn Jahr Schaltjahr ist */
  return sec + 60 * (min + 60 * (hour + 24 * days_since_1970));
}

void setInternalTime(uint64_t epoch = 0, uint32_t us = 0)
{
  struct timeval tv;
  tv.tv_sec = epoch;
  tv.tv_usec = us;
  settimeofday(&tv, NULL);
}
 
void printRTC()
{
  DateTime dtrtc = rtc.now(); // get date time from RTC i
  if (!dtrtc.isValid())
  {
    WebSerial.println(F("E103: RTC not valid"));
  }
  else
  {
    time_t newTime = getTimestamp(dtrtc.year(), dtrtc.month(), dtrtc.day(), dtrtc.hour(), dtrtc.minute(), dtrtc.second());
    WebSerial.printf("RTC:%lld %02d-%02d-%04d %02d:%02d:%02d UTC\n", newTime, dtrtc.month(), dtrtc.day(), dtrtc.year(), dtrtc.hour(), dtrtc.minute(), dtrtc.second());
  }
}

/*
   set date/time of external RTC
*/
void setRTC()
{
  WebSerial.printf("setRTC --> from internal time\n");
  time_t now;          // this are the seconds since Epoch (1970) (UTC)
  tm tm;               // the structure tm holds time information in a convenient way
  time(&now);          // read the current time and store to now
  gmtime_r(&now, &tm); // update the structure tm with the current GMT
  rtc.adjust(DateTime(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec));
}

void showTime()
{
  time_t now;             // this are the seconds since Epoch (1970) GMT
  tm tm;                  // a readable structure
  time(&now);             // read the current time and store to now
  localtime_r(&now, &tm); // update the structure tm with the current time
  char buf[50];
  strftime(buf, sizeof(buf), "%D %T %Z", &tm); // https://www.cplusplus.com/reference/ctime/strftime/
  // WebSerial.printf("now:%llu", now);
  WebSerial.print(buf);
  WebSerial.printf("\n");
}

void time_is_set(bool from_sntp)
{
  if (from_sntp) // needs Core 3.0.0 or higher!
  {
    WebSerial.printf("The internal time is set from SNTP.\n");
    setRTC();
    printRTC();
  }
  else
  {
    Serial.println(F("The internal time is set."));
  }
}

void recvMsg(uint8_t *data, size_t len)
{
  WebSerial.println("Received Data...");
  String d = "";
  for (size_t i = 0; i < len; i++)
  {
    d += char(data[i]);
  }
  WebSerial.println(d);
}



int snprintfTimestamp(char *dest, size_t size)
{
  // tm tm;
  // DateTime dtrtc = rtc.now();          // get date time from RTC i
  // time_t newTime = getTimestamp(dtrtc.year(), dtrtc.month(), dtrtc.day(), dtrtc.hour(), dtrtc.minute(), dtrtc.second());
  // localtime_r(&newTime, &tm);
  // WebSerial.printf("%02d-%02d-%04d %02d:%02d:%02d UTC\n", dtrtc.month(), dtrtc.day(), dtrtc.year(), dtrtc.hour(), dtrtc.minute(), dtrtc.second());

  // DateTime newdt(tm.tm_year, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  // return snprintf(dest, size, "%02d-%02d-%04d %02d:%02d:%02d UTC\n", tm.tm_mon, tm.tm_mday, tm.tm_year, tm.tm_hour, tm.tm_min, tm.tm_sec);

  time_t now;             // this are the seconds since Epoch (1970) GMT
  tm tm;                  // a readable structure
  time(&now);             // read the current time and store to now
  localtime_r(&now, &tm); // update the structure tm with the current time
  char buf[50];
  return strftime(dest, size, "%D %T %Z,", &tm); // https://www.cplusplus.com/reference/ctime/strftime/
}

void getRTC()
{
  Serial.println(F("getRTC --> update internal clock"));
  DateTime dtrtc = rtc.now(); // get date time from RTC i
  if (!dtrtc.isValid())
  {
    Serial.print(F("E127: RTC not valid"));
  }
  else
  {
    time_t newTime = getTimestamp(dtrtc.year(), dtrtc.month(), dtrtc.day(), dtrtc.hour(), dtrtc.minute(), dtrtc.second());
    setInternalTime(newTime);
    // Serial.print(F("UTC:")); Serial.println(newTime);
    // printRTC();
  }
}