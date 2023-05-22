#ifndef FILTER_DEF_H_
#define FILTER_DEF_H_

#define CLOCK_PERIOD 10

#define MAX_IMAGE_BUFFER_LENTH 1024

// sobel mask parameters
const int DMA_TRANS = 64;

// Sobel Filter inner transport addresses
// Used between blocking_transport() & do_filter()
const int SOBEL_FILTER_R_ADDR = 0x00000000;
const int SOBEL_FILTER_RESULT_ADDR = 0x00000004;

const int SOBEL_FILTER_RS_R_ADDR   = 0x00000000;
const int SOBEL_FILTER_RS_W_WIDTH  = 0x00000004;
const int SOBEL_FILTER_RS_W_HEIGHT = 0x00000008;
const int SOBEL_FILTER_RS_W_DATA   = 0x0000000C;
const int SOBEL_FILTER_RS_RESULT_ADDR = 0x00800000;


union word {
  int sint;
  unsigned int uint;
  unsigned char uc[4];
};

// Sobel mask
const int mask[3][3] = {{1, 1, 1}, {1, 2, 1}, {1, 1, 1}};
#endif
