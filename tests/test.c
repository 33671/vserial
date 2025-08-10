/*
 * vserial_test.c - Test program for virtual serial port driver
 *
 * Compile with: gcc -o vserial_test vserial_test.c
 * Requires: /dev/vserialctl and /dev/tty_vserial* devices
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

// IOCTL definitions (must match kernel module)
#define VSERIAL_IOCTL_MAGIC 'V'
#define VSERIAL_SET_PAIRS _IOW(VSERIAL_IOCTL_MAGIC, 0, int)
#define VSERIAL_GET_PAIRS _IOR(VSERIAL_IOCTL_MAGIC, 1, struct vserial_pairs_info)

struct vserial_pair_info {
    int minorA;
    int minorB;
};

struct vserial_pairs_info {
    int num_pairs;
    struct vserial_pair_info *pairs;
};

// Test data pattern
#define TEST_DATA_SIZE 128
static const char test_data[TEST_DATA_SIZE] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "!@#$%^&*()_+-=[]{}|;:,.<>/?`~"
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "!@#$%^&*()_+-=[]{}|;:,.<>/?`~";

// Helper function to print termios settings
void print_termios_settings(const char *prefix, struct termios *tio) {
    printf("%s: ispeed=%d, ospeed=%d, cflag=0x%08x\n",
           prefix,
           cfgetispeed(tio),
           cfgetospeed(tio),
           tio->c_cflag);

    printf("  Data bits: %d, ", tio->c_cflag & CSIZE);
    switch (tio->c_cflag & CSIZE) {
        case CS5: printf("5"); break;
        case CS6: printf("6"); break;
        case CS7: printf("7"); break;
        case CS8: printf("8"); break;
        default: printf("?");
    }

    printf(", Parity: %s, Stop bits: %s\n",
           (tio->c_cflag & PARENB) ?
               ((tio->c_cflag & PARODD) ? "ODD" : "EVEN") : "NONE",
           (tio->c_cflag & CSTOPB) ? "2" : "1");
}

// Configure terminal with specific settings
int configure_serial(int fd, speed_t speed, int data_bits, int parity, int stop_bits) {
    struct termios tio;

    if (tcgetattr(fd, &tio) < 0) {
        perror("tcgetattr failed");
        return -1;
    }

    // Clear data bits, parity, stop bits
    tio.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD);

    // Set data bits
    switch (data_bits) {
        case 5: tio.c_cflag |= CS5; break;
        case 6: tio.c_cflag |= CS6; break;
        case 7: tio.c_cflag |= CS7; break;
        case 8: tio.c_cflag |= CS8; break;
        default:
            fprintf(stderr, "Invalid data bits: %d\n", data_bits);
            return -1;
    }

    // Set parity
    if (parity == 1) { // ODD
        tio.c_cflag |= (PARENB | PARODD);
    } else if (parity == 2) { // EVEN
        tio.c_cflag |= PARENB;
    }

    // Set stop bits
    if (stop_bits == 2) {
        tio.c_cflag |= CSTOPB;
    }

    // Set speed
    if (cfsetispeed(&tio, speed) < 0 || cfsetospeed(&tio, speed) < 0) {
        perror("cfsetispeed/cfsetospeed failed");
        return -1;
    }

    // Raw mode
    cfmakeraw(&tio);

    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        perror("tcsetattr failed");
        return -1;
    }

    return 0;
}

// Verify received data matches expected
int verify_data(const char *received, size_t len, const char *expected, size_t expected_len) {
    if (len != expected_len) {
        printf("  ERROR: Received length %zu != expected %zu\n", len, expected_len);
        return -1;
    }

    if (memcmp(received, expected, len) != 0) {
        printf("  ERROR: Data mismatch!\n");
        // Print first few mismatched bytes
        for (size_t i = 0; i < len && i < 16; i++) {
            if (received[i] != expected[i]) {
                printf("    Mismatch at byte %zu: 0x%02x != 0x%02x\n",
                       i, (unsigned char)received[i], (unsigned char)expected[i]);
            }
        }
        return -1;
    }

    return 0;
}

// Test data exchange with matching configurations
int test_matched_configuration(int fdA, int fdB) {
    char buffer[TEST_DATA_SIZE * 2];
    ssize_t written, read_bytes;

    printf("\n=== TEST 1: MATCHED CONFIGURATIONS ===\n");

    // Configure both ports with same settings
    if (configure_serial(fdA, B115200, 8, 0, 1) < 0 ||
        configure_serial(fdB, B115200, 8, 0, 1) < 0) {
        fprintf(stderr, "Failed to configure serial ports\n");
        return -1;
    }

    // Print settings for verification
    struct termios tioA, tioB;
    tcgetattr(fdA, &tioA);
    tcgetattr(fdB, &tioB);
    print_termios_settings("Port A", &tioA);
    print_termios_settings("Port B", &tioB);

    // Test 1: Write from A to B
    printf("\nTesting data flow A → B:\n");
    written = write(fdA, test_data, TEST_DATA_SIZE);
    if (written != TEST_DATA_SIZE) {
        perror("Write from A failed");
        return -1;
    }
    printf("  Wrote %zd bytes from A\n", written);

    // Give time for data to propagate
    usleep(100000);

    read_bytes = read(fdB, buffer, sizeof(buffer));
    if (read_bytes <= 0) {
        perror("Read at B failed");
        return -1;
    }
    printf("  Read %zd bytes at B\n", read_bytes);

    if (verify_data(buffer, read_bytes, test_data, TEST_DATA_SIZE) < 0) {
        return -1;
    }
    printf("  SUCCESS: Data matched for A → B\n");

    // Test 2: Write from B to A
    printf("\nTesting data flow B → A:\n");
    written = write(fdB, test_data, TEST_DATA_SIZE);
    if (written != TEST_DATA_SIZE) {
        perror("Write from B failed");
        return -1;
    }
    printf("  Wrote %zd bytes from B\n", written);

    // Give time for data to propagate
    usleep(100000);

    read_bytes = read(fdA, buffer, sizeof(buffer));
    if (read_bytes <= 0) {
        perror("Read at A failed");
        return -1;
    }
    printf("  Read %zd bytes at A\n", read_bytes);

    if (verify_data(buffer, read_bytes, test_data, TEST_DATA_SIZE) < 0) {
        return -1;
    }
    printf("  SUCCESS: Data matched for B → A\n");

    return 0;
}

// Test data exchange with non-matched configurations
int test_non_matched_configuration(int fdA, int fdB) {
    char buffer[TEST_DATA_SIZE * 2];
    ssize_t written, read_bytes;

    printf("\n=== TEST 2: NON-MATCHED CONFIGURATIONS ===\n");

    // Configure ports with different settings
    // Port A: 115200, 8N1
    if (configure_serial(fdA, B115200, 8, 0, 1) < 0) {
        fprintf(stderr, "Failed to configure Port A\n");
        return -1;
    }

    // Port B: 9600, 7E1 (incompatible)
    if (configure_serial(fdB, B9600, 7, 2, 1) < 0) {
        fprintf(stderr, "Failed to configure Port B\n");
        return -1;
    }

    // Print settings for verification
    struct termios tioA, tioB;
    tcgetattr(fdA, &tioA);
    tcgetattr(fdB, &tioB);
    print_termios_settings("Port A", &tioA);
    print_termios_settings("Port B", &tioB);

    // Test 1: Write from A to B (should fail due to mismatch)
    printf("\nTesting data flow A → B (mismatched):\n");
    written = write(fdA, test_data, TEST_DATA_SIZE);
    if (written != TEST_DATA_SIZE) {
        perror("Write from A failed");
        return -1;
    }
    printf("  Wrote %zd bytes from A\n", written);

    // Give time for data to propagate
    usleep(100000);

    // Should not receive data due to termios mismatch
    read_bytes = read(fdB, buffer, sizeof(buffer));
    if (read_bytes > 0) {
        printf("  ERROR: Received %zd bytes at B when expecting none!\n", read_bytes);
        return -1;
    } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
        // No data is expected, but we don't want other errors
        perror("Unexpected error reading from B");
        return -1;
    }

    printf("  SUCCESS: No data received at B (expected due to configuration mismatch)\n");

    // Test 2: Make them compatible and try again
    printf("\nMaking configurations compatible...\n");
    if (configure_serial(fdB, B115200, 8, 0, 1) < 0) {
        fprintf(stderr, "Failed to reconfigure Port B\n");
        return -1;
    }

    // Verify settings
    tcgetattr(fdA, &tioA);
    tcgetattr(fdB, &tioB);
    print_termios_settings("Port A", &tioA);
    print_termios_settings("Port B", &tioB);

    // Try again - should now work
    printf("\nTesting data flow A → B after making compatible:\n");
    written = write(fdA, test_data, TEST_DATA_SIZE);
    if (written != TEST_DATA_SIZE) {
        perror("Write from A failed");
        return -1;
    }
    printf("  Wrote %zd bytes from A\n", written);

    // Give time for data to propagate
    usleep(100000);

    read_bytes = read(fdB, buffer, sizeof(buffer));
    if (read_bytes <= 0) {
        perror("Read at B failed");
        return -1;
    }
    printf("  Read %zd bytes at B\n", read_bytes);

    if (verify_data(buffer, read_bytes, test_data, TEST_DATA_SIZE) < 0) {
        return -1;
    }
    printf("  SUCCESS: Data matched after fixing configuration\n");

    return 0;
}

int main() {
    int ctl_fd, fdA = -1, fdB = -1;
    struct vserial_pairs_info info;
    int pair_count = 1;
    char dev_path[64];

    printf("Virtual Serial Port Driver Test Program\n");
    printf("=======================================\n\n");

    // 1. Open control device
    ctl_fd = open("/dev/vserialctl", O_RDWR);
    if (ctl_fd < 0) {
        perror("Failed to open /dev/vserialctl");
        fprintf(stderr, "Make sure the vserial module is loaded\n");
        return EXIT_FAILURE;
    }
    printf("Opened control device /dev/vserialctl\n");

    // 2. Set up 1 pair of virtual serial ports
    if (ioctl(ctl_fd, VSERIAL_SET_PAIRS, &pair_count) < 0) {
        perror("VSERIAL_SET_PAIRS failed");
        close(ctl_fd);
        return EXIT_FAILURE;
    }
    printf("Created %d virtual serial pair\n", pair_count);
    system("ls -l /dev/tty_vserial0");
    sleep(1);
    system("ls -l /dev/tty_vserial0");
    // 3. Get pair information
    info.num_pairs = 0;
    info.pairs = NULL;

    if (ioctl(ctl_fd, VSERIAL_GET_PAIRS, &info) < 0) {
        perror("VSERIAL_GET_PAIRS failed");
        // Try to clean up before exiting
        pair_count = 0;
        //ioctl(ctl_fd, VSERIAL_SET_PAIRS, &pair_count);
        close(ctl_fd);
        return EXIT_FAILURE;
    }

    // Allocate memory for pairs info
    info.pairs = malloc(sizeof(struct vserial_pair_info) * info.num_pairs);
    if (!info.pairs) {
        perror("malloc failed");
        pair_count = 0;
        //ioctl(ctl_fd, VSERIAL_SET_PAIRS, &pair_count);
        close(ctl_fd);
        return EXIT_FAILURE;
    }

    // Get actual pair info
    if (ioctl(ctl_fd, VSERIAL_GET_PAIRS, &info) < 0) {
        perror("VSERIAL_GET_PAIRS (2nd call) failed");
        free(info.pairs);
        pair_count = 0;
        //ioctl(ctl_fd, VSERIAL_SET_PAIRS, &pair_count);
        close(ctl_fd);
        return EXIT_FAILURE;
    }

    printf("Got %d virtual serial pair(s)\n", info.num_pairs);
    if (info.num_pairs > 0) {
        printf("Pair 0: Minor A=%d, Minor B=%d\n",
               info.pairs[0].minorA, info.pairs[0].minorB);
    }

    // 4. Open both ends of the virtual serial pair
    snprintf(dev_path, sizeof(dev_path), "/dev/tty_vserial%d", info.pairs[0].minorA);
    fdA = open(dev_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fdA < 0) {
        perror("Failed to open port A");
        free(info.pairs);
        pair_count = 0;
        //ioctl(ctl_fd, VSERIAL_SET_PAIRS, &pair_count);
        close(ctl_fd);
        return EXIT_FAILURE;
    }
    printf("Opened %s\n", dev_path);

    snprintf(dev_path, sizeof(dev_path), "/dev/tty_vserial%d", info.pairs[0].minorB);
    fdB = open(dev_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fdB < 0) {
        perror("Failed to open port B");
        close(fdA);
        free(info.pairs);
        pair_count = 0;
        //ioctl(ctl_fd, VSERIAL_SET_PAIRS, &pair_count);
        close(ctl_fd);
        return EXIT_FAILURE;
    }
    printf("Opened %s\n", dev_path);

    // 5. Run test with matched configurations
    if (test_matched_configuration(fdA, fdB) < 0) {
        printf("\nMATCHED CONFIGURATION TEST FAILED\n");
    } else {
        printf("\nMATCHED CONFIGURATION TEST PASSED\n");
    }

    // 6. Run test with non-matched configurations
    if (test_non_matched_configuration(fdA, fdB) < 0) {
        printf("\nNON-MATCHED CONFIGURATION TEST FAILED\n");
    } else {
        printf("\nNON-MATCHED CONFIGURATION TEST PASSED\n");
    }

    // 7. Cleanup
    printf("\nCleaning up...\n");
    if (fdA >= 0) close(fdA);
    if (fdB >= 0) close(fdB);
    free(info.pairs);

    // Remove all pairs
    pair_count = 0;
    //if (ioctl(ctl_fd, VSERIAL_SET_PAIRS, &pair_count) < 0) {
    //    perror("Failed to clean up virtual pairs");
    //}

    close(ctl_fd);
    printf("Test completed successfully\n");

    return EXIT_SUCCESS;
}
