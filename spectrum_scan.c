#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

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

                char *scan_result = "SCAN_RESULT ";
                char is_scan_result = strncmp(buffer, scan_result, strlen(scan_result)) == 0;

                if (is_scan_result) {
                    if (
                    !(buffer[pos-2] == ']' || // that's where it should "normally" appear
                      buffer[pos-3] == ']')   // that's where it actually appears - because HWCDC.println produces \r\n
                    )
                    {
                        errors++;
                    }
                    // assert: pos points at the first byte of the next line
                    write(1, buffer, pos);
                    // printf("Last char is: %c\n", buffer[pos-3]); - wow, HWCDC.println ends up producing \r\n
                } else if (strncmp(buffer, "Wrote stuff in ", 15) == 0) {
                    lines--;
                    write(1, buffer, pos);
                } else if (strncmp(buffer, "Unable to make ", 15) == 0) {
                    lines--;
                    write(1, buffer, pos);
                } else {
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
