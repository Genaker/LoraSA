

#ifdef OSD_ENABLED
void osd_spectrum()
{ // OSD enabled

    for (int i = 0; i < OSD_WIDTH; i++)
    {
        max_bins_array[i] = 33;
        max_bins_array_value[i] = 0;
    }
    // memset(max_bins_array, 33, 30);
    max_bin = 0;

    osd.displayString(12, 1, String(FREQ_BEGIN));
    osd.displayString(12, 30 - 8, String(FREQ_END));
    // Finding biggest in result
    //  Skiping 0 to avoid overflow
    for (int i = 1; i < 32; i++)
    {
        // filter
        if (result[i] > 0 && ((result[i - 1] != 0)))
        {
            max_bin = i;
#ifdef PRINT_DEBUG
            Serial.print("MAX in bin:" + String(max_bin));
            Serial.println();
#endif
            break;
        }
    }
    if (max_bins_array[col] > max_bin)
    {
        max_bins_array[col] = max_bin;
// Store RSSI value for RSSI Method
#ifdef METHOD_RSSI
        max_bins_array_value[col] = result[max_bin];
#endif
    }
    // Going to the next OSD step
    if (x % osd_steps == 0 && col < 30)
    {
        // OSD SIDE BAR with frequency log
#ifdef OSD_SIDE_BAR
        {
            osd.displayString(col, 30 - 7,
                              String(FREQ_BEGIN + (col * osd_mhz_in_bin)) + ":" +
                                  String(max_bins_array[col]));
        }
#endif
        // Test with Random Result...
        // max_bins_array[s] = rand() % 32;
#ifdef METHOD_RSSI
        // With THe RSSI method we can get real RSSI value not just a bin
#endif
        // PRINT SIGNAL CHAR ROW, COL, VALUE
        osdPrintSignalLevelChart(col, max_bins_array[col]);

#ifdef PRINT_DEBUG
        Serial.println("MAX:" + String(max_bins_array[col]));
#endif
        col++;
    }
}

#ifdef OSD_ENABLED
unsigned short selectFreqChar(int bin, int start_level = 0)
{
    if (bin > start_level)
    {
        // level when we are starting show levels symbols
        // you can override with your own character for example 0x100 = " " empty char
        return power_level[33];
    }
    else if (bin >= 0 && bin < MAX_POWER_LEVELS)
        return power_level[bin];
    // when wrong bin number or noc har assigned we are showing "!" char
    return 0x121;
}
#endif
#ifdef OSD_ENABLED
void osdPrintSignalLevelChart(int col, int signal_value)
{
    // Third line
    if (signal_value <= 9)
    {
        osd.displayChar(13, col + 2, 0x100);
        osd.displayChar(14, col + 2, 0x100);
        osd.displayChar(12, col + 2, selectFreqChar(signal_value, drone_detection_level));
    }
    // Second line
    else if (max_bins_array[col] < 19)
    {
        osd.displayChar(12, col + 2, 0x100);
        osd.displayChar(14, col + 2, 0x100);
        osd.displayChar(13, col + 2, selectFreqChar(signal_value, drone_detection_level));
    }
    // First line
    else
    {
        // Clean Up symbol
        osd.displayChar(12, col + 2, 0x100);
        osd.displayChar(13, col + 2, 0x100);
        osd.displayChar(14, col + 2, selectFreqChar(signal_value, drone_detection_level));
    }
}
#endif
#endif // END OSD ENABLED
