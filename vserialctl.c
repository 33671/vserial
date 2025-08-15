#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// IOCTL definitions (must match kernel module)
#define VSERIAL_IOCTL_MAGIC 'V'
#define VSERIAL_SET_PAIRS _IOW(VSERIAL_IOCTL_MAGIC, 0, int)
#define VSERIAL_GET_PAIRS                                                      \
  _IOR(VSERIAL_IOCTL_MAGIC, 1, struct vserial_pairs_info)

struct vserial_pair_info {
  int minorA; // First port in the pair
  int minorB; // Second port in the pair (connected to minorA)
};

struct vserial_pairs_info {
  int num_pairs;                   // Number of virtual pairs created
  struct vserial_pair_info *pairs; // Array of pair information
};
void print_usage(char *program_name) {
  printf("Usage: %s [options]\n", program_name);
  printf("Options:\n");
  printf("  -c, --clear          clear all virtual serial pairs\n");
  printf("  -s N, --set-pairs N  set number of virtual serial pairs (0-64)\n");
  printf("  -h, --help           show this help\n");
}

int main(int argc, char **argv) {
  static struct option long_options[] = {
      {"clear", no_argument, NULL, 'c'},
      {"set-pairs", required_argument, NULL, 's'},
      {"help", no_argument, NULL, 'h'},
      {NULL, 0, NULL, 0}};

  int c;
  int action = 0; // 0: none, 1: clear, 2: set
  int num_pairs = 0;

  while ((c = getopt_long(argc, argv, "cs:h", long_options, NULL)) != -1) {
    switch (c) {
    case 'h':
      print_usage(argv[0]);
      return 0;
    case 'c':
      if (action != 0) {
        fprintf(stderr, "Multiple actions specified\n");
        return 1;
      }
      action = 1;
      break;
    case 's':
      if (action != 0) {
        fprintf(stderr, "Multiple actions specified\n");
        return 1;
      }
      action = 2;
      num_pairs = atoi(optarg);
      if (num_pairs < 0 || num_pairs > 64) {
        fprintf(stderr, "Invalid number of pairs: %d (must be 0-64)\n",
                num_pairs);
        return 1;
      }
      break;
    default:
      return 1;
    }
  }

  if (action == 0) {
    print_usage(argv[0]);
    return 1;
  }

  int fd = open("/dev/vserialctl", O_RDWR);
  if (fd < 0) {
    perror("Failed to open /dev/vserialctl");
    return 1;
  }

  int set_num = (action == 1) ? 0 : num_pairs;
  if (ioctl(fd, VSERIAL_SET_PAIRS, &set_num) < 0) {
    perror("VSERIAL_SET_PAIRS failed");
    close(fd);
    return 1;
  }
  sleep(1);

  if (set_num == 0) {
    printf("Cleared all virtual serial pairs.\n");
    close(fd);
    return 0;
  }

  // Get information about created pairs
  struct vserial_pairs_info info;
  info.num_pairs = 0;
  info.pairs = NULL;

  if (ioctl(fd, VSERIAL_GET_PAIRS, &info) < 0) {
    perror("VSERIAL_GET_PAIRS (first call) failed");
    close(fd);
    return 1;
  }

  info.pairs = malloc(sizeof(struct vserial_pair_info) * info.num_pairs);
  if (!info.pairs) {
    perror("malloc failed");
    close(fd);
    return 1;
  }

  if (ioctl(fd, VSERIAL_GET_PAIRS, &info) < 0) {
    perror("VSERIAL_GET_PAIRS (second call) failed");
    free(info.pairs);
    close(fd);
    return 1;
  }

  printf("Successfully created %d virtual serial pairs:\n", info.num_pairs);
  for (int i = 0; i < info.num_pairs; i++) {
    printf("Pair %d: /dev/ttyGSV%d <-> /dev/ttyGSV%d\n", i,
           info.pairs[i].minorA, info.pairs[i].minorB);
  }

  free(info.pairs);
  close(fd);
  return 0;
}
