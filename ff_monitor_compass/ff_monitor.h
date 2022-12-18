#include  "hls_stream.h"
#include "ap_axi_sdata.h"
//#include <ap_fixed.h>
#include "string.h"

#define WIDTH 32

// typedef ap_uint<8> pixel_type;

typedef hls::stream<ap_axiu<WIDTH,1,1,1> > axis;
typedef ap_axiu<WIDTH,1,1,1> video_stream;

void ff_monitor_gen(axis& op, int row, int column, int char_1, int char_2) ;


