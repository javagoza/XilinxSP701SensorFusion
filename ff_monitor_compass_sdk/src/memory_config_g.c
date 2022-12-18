/* This file is automatically generated based on your hardware design. */
#include "memory_config.h"

struct memory_range_s memory_ranges[] = {
	/* microblaze_0_local_memory_dlmb_bram_Mem memory will not be tested since application resides in the same memory */
	{
		"mig_7series_1_memaddr",
		"mig_7series_1",
		0x80000000,
		134217728,
	},
	{
		"bram_ctrl_0_Mem0",
		"bram_ctrl_0",
		0xC0000000,
		8192,
	},
};

int n_memory_ranges = 2;