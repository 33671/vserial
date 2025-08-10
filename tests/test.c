/*
 * vserial_test.c - Test program for virtual serial port driver
 *
 * This extended version tests multiple virtual serial pairs (2 pairs) in both
 * matched and non-matched configuration scenarios. It verifies data transmission
 * correctness when port configurations match, and validates expected failure
 * when configurations mismatch (with subsequent recovery test).
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
    int minorA;  // First port in the pair
    int minorB;  // Second port in the pair (connected to minorA)
};

struct vserial_pairs_info {
    int num_pairs;             // Number of virtual pairs created
    struct vserial_pair_info *pairs;  // Array of pair information
};

// Test data pattern - contains all printable ASCII characters
#define TEST_DATA_SIZE 128
static const char test_data[TEST_DATA_SIZE] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "!@#$%^&*()_+-=[]{}|;:,.<>/?`~"
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "!@#$%^&*()_+-=[]{}|;:,.<>/?`~";

// Helper function to print termios settings for debugging
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

// Configure terminal with specific serial settings
int configure_serial(int fd, speed_t speed, int data_bits, int parity, int stop_bits) {
    struct termios tio;

    if (tcgetattr(fd, &tio) < 0) {
        perror("tcgetattr failed");
        return -1;
    }

    // Clear existing data bits, parity, and stop bits settings
    tio.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD);

    // Set data bits (5-8 bits supported)
    switch (data_bits) {
        case 5: tio.c_cflag |= CS5; break;
        case 6: tio.c_cflag |= CS6; break;
        case 7: tio.c_cflag |= CS7; break;
        case 8: tio.c_cflag |= CS8; break;
        default:
            fprintf(stderr, "Invalid data bits: %d\n", data_bits);
            return -1;
    }

    // Set parity (0=none, 1=odd, 2=even)
    if (parity == 1) { // ODD parity
        tio.c_cflag |= (PARENB | PARODD);
    } else if (parity == 2) { // EVEN parity
        tio.c_cflag |= PARENB;
    }

    // Set stop bits (1 or 2)
    if (stop_bits == 2) {
        tio.c_cflag |= CSTOPB;
    }

    // Configure input/output speed
    if (cfsetispeed(&tio, speed) < 0 || cfsetospeed(&tio, speed) < 0) {
        perror("cfsetispeed/cfsetospeed failed");
        return -1;
    }

    // Set raw mode (disable canonical mode and special character processing)
    cfmakeraw(&tio);

    // Apply settings immediately
    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        perror("tcsetattr failed");
        return -1;
    }

    return 0;
}

// Verify received data matches expected pattern
int verify_data(const char *received, size_t len, const char *expected, size_t expected_len) {
    if (len != expected_len) {
        printf("  ERROR: Received length %zu != expected %zu\n", len, expected_len);
        return -1;
    }

    if (memcmp(received, expected, len) != 0) {
        printf("  ERROR: Data mismatch!\n");
        // Print first few mismatched bytes for debugging
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

// Test data exchange with matching configurations (should succeed)
int test_matched_configuration(int fdA, int fdB) {
    char buffer[TEST_DATA_SIZE * 2];
    ssize_t written, read_bytes;

    printf("\n=== TEST 1: MATCHED CONFIGURATIONS (8N1 @ 115200) ===\n");

    // Configure both ports identically (115200 baud, 8N1)
    if (configure_serial(fdA, B115200, 8, 0, 1) < 0 ||
        configure_serial(fdB, B115200, 8, 0, 1) < 0) {
        fprintf(stderr, "Failed to configure serial ports\n");
        return -1;
    }

    // Verify settings were applied correctly
    struct termios tioA, tioB;
    tcgetattr(fdA, &tioA);
    tcgetattr(fdB, &tioB);
    print_termios_settings("Port A", &tioA);
    print_termios_settings("Port B", &tioB);

    // Test bidirectional data flow
    printf("\nTesting data flow A → B:\n");
    written = write(fdA, test_data, TEST_DATA_SIZE);
    if (written != TEST_DATA_SIZE) {
        perror("Write from A failed");
        return -1;
    }
    printf("  Wrote %zd bytes from A\n", written);

    // Allow time for data propagation through virtual driver
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

    // Test reverse direction
    printf("\nTesting data flow B → A:\n");
    written = write(fdB, test_data, TEST_DATA_SIZE);
    if (written != TEST_DATA_SIZE) {
        perror("Write from B failed");
        return -1;
    }
    printf("  Wrote %zd bytes from B\n", written);

    usleep(100000);  // Allow propagation time

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

// Test data exchange with non-matching configurations (should fail initially)
int test_non_matched_configuration(int fdA, int fdB) {
    char buffer[TEST_DATA_SIZE * 2];
    ssize_t written, read_bytes;

    printf("\n=== TEST 2: NON-MATCHED CONFIGURATIONS (A:115200/8N1 vs B:9600/7E1) ===\n");

    // Configure ports with incompatible settings
    // Port A: Standard 115200 baud, 8N1
    if (configure_serial(fdA, B115200, 8, 0, 1) < 0) {
        fprintf(stderr, "Failed to configure Port A\n");
        return -1;
    }

    // Port B: Deliberately mismatched (9600 baud, 7E1)
    if (configure_serial(fdB, B9600, 7, 2, 1) < 0) {
        fprintf(stderr, "Failed to configure Port B\n");
        return -1;
    }

    // Verify incompatible settings
    struct termios tioA, tioB;
    tcgetattr(fdA, &tioA);
    tcgetattr(fdB, &tioB);
    print_termios_settings("Port A", &tioA);
    print_termios_settings("Port B", &tioB);

    // Test 1: Write from A to B with mismatched settings (should fail)
    printf("\nTesting data flow A → B (mismatched):\n");
    written = write(fdA, test_data, TEST_DATA_SIZE);
    if (written != TEST_DATA_SIZE) {
        perror("Write from A failed");
        return -1;
    }
    printf("  Wrote %zd bytes from A\n", written);

    usleep(100000);  // Allow propagation time

    // Expect no data due to configuration mismatch
    read_bytes = read(fdB, buffer, sizeof(buffer));
    if (read_bytes > 0) {
        printf("  ERROR: Received %zd bytes at B when expecting none!\n", read_bytes);
        return -1;
    } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
        perror("Unexpected error reading from B");
        return -1;
    }

    printf("  SUCCESS: No data received at B (expected due to configuration mismatch)\n");

    // Test 2: Fix configuration and verify recovery
    printf("\nMaking configurations compatible...\n");
    if (configure_serial(fdB, B115200, 8, 0, 1) < 0) {
        fprintf(stderr, "Failed to reconfigure Port B\n");
        return -1;
    }

    // Verify settings are now identical
    tcgetattr(fdA, &tioA);
    tcgetattr(fdB, &tioB);
    print_termios_settings("Port A", &tioA);
    print_termios_settings("Port B", &tioB);

    // Test again with compatible settings
    printf("\nTesting data flow A → B after making compatible:\n");
    written = write(fdA, test_data, TEST_DATA_SIZE);
    if (written != TEST_DATA_SIZE) {
        perror("Write from A failed");
        return -1;
    }
    printf("  Wrote %zd bytes from A\n", written);

    usleep(100000);  // Allow propagation time

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
    int pair_count = 4;  // Create virtual serial pairs for comprehensive testing
    char dev_path[64];

    printf("Virtual Serial Port Driver Test Program\n");
    printf("=======================================\n");
    printf("Testing multiple pairs (2) in matched and non-matched configurations\n\n");

    // 1. Open control device for driver management
    ctl_fd = open("/dev/vserialctl", O_RDWR);
    if (ctl_fd < 0) {
        perror("Failed to open /dev/vserialctl");
        fprintf(stderr, "Make sure the vserial module is loaded\n");
        return EXIT_FAILURE;
    }
    printf("Opened control device /dev/vserialctl\n");

    // 2. Create 2 virtual serial pairs (instead of 1)
    if (ioctl(ctl_fd, VSERIAL_SET_PAIRS, &pair_count) < 0) {
        perror("VSERIAL_SET_PAIRS failed");
        close(ctl_fd);
        return EXIT_FAILURE;
    }
    printf("Created %d virtual serial pairs\n", pair_count);

    // Note: Removed ls/sleep calls as they're unnecessary for multiple pair testing
    // Original code had:
    //   system("ls -l /dev/tty_vserial0");
    //   sleep(1);
    //   system("ls -l /dev/tty_vserial0");
    // But these are device-specific and not needed for multi-pair testing

    // 3. Get information about created pairs
    info.num_pairs = 0;
    info.pairs = NULL;

    // First call to get number of pairs
    if (ioctl(ctl_fd, VSERIAL_GET_PAIRS, &info) < 0) {
        perror("VSERIAL_GET_PAIRS failed");
        pair_count = 0;
        // ioctl(ctl_fd, VSERIAL_SET_PAIRS, &pair_count);
        close(ctl_fd);
        return EXIT_FAILURE;
    }

    // Allocate memory for pair information
    info.pairs = malloc(sizeof(struct vserial_pair_info) * info.num_pairs);
    if (!info.pairs) {
        perror("malloc failed");
        pair_count = 0;
        // ioctl(ctl_fd, VSERIAL_SET_PAIRS, &pair_count);
        close(ctl_fd);
        return EXIT_FAILURE;
    }

    // Second call to populate pair information
    if (ioctl(ctl_fd, VSERIAL_GET_PAIRS, &info) < 0) {
        perror("VSERIAL_GET_PAIRS (2nd call) failed");
        free(info.pairs);
        pair_count = 0;
        // ioctl(ctl_fd, VSERIAL_SET_PAIRS, &pair_count);
        close(ctl_fd);
        return EXIT_FAILURE;
    }

    printf("Got %d virtual serial pair(s)\n", info.num_pairs);
    for (int i = 0; i < info.num_pairs; i++) {
        printf("  Pair %d: Minor A=%d, Minor B=%d\n",
               i, info.pairs[i].minorA, info.pairs[i].minorB);
    }

    // 4. Test each virtual pair independently
    int overall_result = EXIT_SUCCESS;
    for (int pair_idx = 0; pair_idx < info.num_pairs; pair_idx++) {
        printf("\n");
        printf("==================================================\n");
        printf("  TESTING VIRTUAL PAIR #%d (minorA=%d, minorB=%d)\n",
               pair_idx, info.pairs[pair_idx].minorA, info.pairs[pair_idx].minorB);
        printf("==================================================\n");
        sleep(1);
        // Open first port in the pair (A)
        snprintf(dev_path, sizeof(dev_path), "/dev/tty_vserial%d", info.pairs[pair_idx].minorA);
        fdA = open(dev_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fdA < 0) {
            perror("Failed to open port A");
            overall_result = EXIT_FAILURE;
            continue;  // Skip to next pair
        }
        printf("  Opened %s\n", dev_path);

        // Open second port in the pair (B)
        snprintf(dev_path, sizeof(dev_path), "/dev/tty_vserial%d", info.pairs[pair_idx].minorB);
        fdB = open(dev_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fdB < 0) {
            perror("Failed to open port B");
            close(fdA);
            overall_result = EXIT_FAILURE;
            continue;  // Skip to next pair
        }
        printf("  Opened %s\n", dev_path);

        // Test 1: Matched configuration test
        printf("\n--- Running matched configuration test ---\n");
        if (test_matched_configuration(fdA, fdB) < 0) {
            printf("\nPAIR %d: MATCHED CONFIGURATION TEST FAILED\n", pair_idx);
            overall_result = EXIT_FAILURE;
        } else {
            printf("\nPAIR %d: MATCHED CONFIGURATION TEST PASSED\n", pair_idx);
        }

        // Test 2: Non-matched configuration test
        printf("\n--- Running non-matched configuration test ---\n");
        if (test_non_matched_configuration(fdA, fdB) < 0) {
            printf("\nPAIR %d: NON-MATCHED CONFIGURATION TEST FAILED\n", pair_idx);
            overall_result = EXIT_FAILURE;
        } else {
            printf("\nPAIR %d: NON-MATCHED CONFIGURATION TEST PASSED\n", pair_idx);
        }

        // Close ports for this pair before moving to next
        close(fdA);
        close(fdB);
        fdA = fdB = -1;  // Reset descriptors
    }

    // 5. Cleanup resources
    printf("\n");
    printf("==================================================\n");
    printf("  CLEANUP AND FINAL VERIFICATION\n");
    printf("==================================================\n");
    free(info.pairs);

    // Remove all virtual pairs (critical for resource cleanup)
    // pair_count = 0;
    // if (ioctl(ctl_fd, VSERIAL_SET_PAIRS, &pair_count) < 0) {
    //     perror("Failed to clean up virtual pairs");
    //     close(ctl_fd);
    //     return EXIT_FAILURE;
    // }
    // printf("Removed all virtual serial pairs\n");

    close(ctl_fd);
    printf("Closed control device\n");

    if (overall_result == EXIT_SUCCESS) {
        printf("\nALL TESTS COMPLETED SUCCESSFULLY FOR ALL PAIRS\n");
    } else {
        printf("\nONE OR MORE TESTS FAILED\n");
    }

    return overall_result;
}
