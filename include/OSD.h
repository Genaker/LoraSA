#ifndef _OSD_H_
#define _OSD_H_
extern void osd_spectrum();
extern void osdPrintSignalLevelChart(int col, int signal_value);
extern unsigned short selectFreqChar(int bin, int start_level = 0);
#endif
