#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

typedef struct {
    uint16_t checksum;
    uint16_t so_far;
    size_t len;
} wrap_t;

#define POLY 0x1021
uint16_t crc16(char *p, char *end, uint16_t c) {
    c ^= 0xffff;
    if (end == NULL) {
        end = strchr(p, 0);
    }

    for (; p != end; p++) {
        uint16_t ch = *p;
        c = c ^ (ch << 8);
        for (int j = 0; j < 8; j++) {
            if (c & 0x8000) {
                c = (c << 1) ^ POLY;
            } else {
                c <<= 1;
            }
        }
    }

    return c ^ 0xffff;
}

#define BUFSIZE 102400
#define LOOPS 200
int main(int argc, char** argv)
{
    char *port = "/dev/ttyACM0";

    int device = open(port, O_RDWR | O_NOCTTY | O_SYNC);

    char *scan_ln = "SCAN -1 -1\n";
    char *buffer = malloc(BUFSIZE + 1);

    write(device, scan_ln, strlen(scan_ln));

    int lines = 0;
    int errors = 0;

    int pos = 0;
    int progress = 0;
    int progress_ln = 0;

    wrap_t wrapper;
    wrapper.len = 0;

    while(lines - errors < LOOPS)
    {
        if (pos == BUFSIZE)
        {
            buffer[pos] = 0;
            printf("Something went completely wrong: %s\n", buffer);
            break;
        }

        int n = read(device, buffer + pos, BUFSIZE - pos);

        if (n > 0)
        {
            n += pos;
            buffer[n] = 0;

            char *nl = strchr(buffer + pos, '\n');
            while (nl != NULL)
            {
                lines++;

                // assert: n points at the first unused byte in buffer
                pos = (nl - buffer) + 1;

                char *wrap = "WRAP ";
                char is_wrap = strncmp(buffer, wrap, strlen(wrap)) == 0;
                if (is_wrap) {
                    char *end;
                    char *p = buffer + strlen(wrap);
                    wrapper.checksum = strtol(p, &end, 16);
                    wrapper.len = 0;
                    if (end == p) {
                        is_wrap = 0;
                    } else {
                        p = end;
                        wrapper.len = strtol(p, &end, 10);
                        if (end == p) {
                            is_wrap = 0;
                        } else {
                            wrapper.so_far = crc16(p+1, end+1, 0);
                        }
                    }
                }

                char *scan_result = "SCAN_RESULT ";
                char is_scan_result = strncmp(buffer, scan_result, strlen(scan_result)) == 0;

                if (is_scan_result) {
                    uint16_t c16 = crc16(buffer, buffer + pos, wrapper.so_far);
                    if (wrapper.len == 0 || wrapper.checksum != c16)
                    {
                        errors++;
                        if (wrapper.len == 0) {
                            int n = snprintf(buffer, pos, "E: no wrap"); // string definitely fits
                            buffer[n] = ' ';
                        } else {
                            int n = snprintf(buffer, pos, "E: bad crc"); // string definitely fits
                            buffer[n] = ' ';
                            int n1 = snprintf(buffer + n + 1, pos - n - 1, "want: %x, got: %x", wrapper.checksum, c16);
                            n1 += n + 1;
                            if (n1 <= pos - n - 1) {
                                buffer[n1] = ' ';
                            }
                        }
                    }
                    wrapper.len = 0; // ok, used up the wrapper

                    // assert: pos points at the first byte of the next line
                    write(1, buffer, pos);
                    // printf("Last char is: %c\n", buffer[pos-3]); - wow, HWCDC.println ends up producing \r\n
                } else if (strncmp(buffer, "Wrote stuff in ", 15) == 0) {
                    lines--;
                    write(1, buffer, pos);
                } else if (strncmp(buffer, "Unable to make ", 15) == 0) {
                    lines--;
                    write(1, buffer, pos);
                } else if (!is_wrap) {
                    write(1, "> ", 2);
                    write(1, buffer, pos);
                    errors++;
                }

                if (lines > progress_ln)
                {
                    progress_ln = lines + 10;
                    if (lines - errors > progress)
                    {
                        progress = lines - errors;
                    }
                    else
                    {
                        printf("D'oh! %d lines read, %d errors - no progress\n", lines, errors);
                    }
                }

                n -= pos;
                memmove(buffer, nl + 1, n + 1);
                // assert: n points at the first unused byte after moving out the parsed line
                nl = strchr(buffer, '\n');
            }

            pos = n;
        }
    }

    char *stop_scan_ln = "SCAN 0 -1\n";
    write(device, stop_scan_ln, strlen(stop_scan_ln));
    close(device);

    free(buffer);

    printf("Read %d lines, got %d errors. Success rate: %.2f\n",
           lines, errors,
           lines == 0 ? 0.0: 100 - 100.0 * errors / lines);
}
