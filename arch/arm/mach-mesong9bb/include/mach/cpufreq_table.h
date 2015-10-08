#include <linux/cpufreq.h>

static struct cpufreq_frequency_table meson_freq_table[]=
{
    //	0	, CPUFREQ_ENTRY_INVALID    ,
    //	1	, CPUFREQ_ENTRY_INVALID    ,
    {0	, 96000    },
    {1	, 192000   },
    {2	, 312000   },
    {3	, 408000   },
    {4	, 504000   },
    {5	, 600000   },
    {6	, 696000   },
    {7	, 816000   },
    {8	, 912000   },
    {9	, 1008000  },
    {10	, 1104000  },
    {11	, 1200000  },
    {12	, 1296000  },
    {13	, 1416000  },
    {14	, 1488000  },
    {15	, CPUFREQ_TABLE_END},
};

#ifdef CONFIG_FIX_SYSPLL
static struct cpufreq_frequency_table meson_freq_table_fix_syspll[]=
{
    {0  ,  1},      //  1 / 32
    {1  ,  2},      //  2 / 32
    {2  ,  4},      //  4 / 32
    {3  ,  8},      //  8 / 32
    {4  , 16},      // 16 / 32
    {5  , 20},      // 20 / 32
    {6  , 24},      // 24 / 32
    {7  , 28},      // 28 / 32
    {8  , 32},      // 32 / 32
    {9  , CPUFREQ_TABLE_END},
};

void adj_cpufreq_table(struct cpufreq_frequency_table *table, int target, int mpll);

int fixpll_freq_verify(unsigned long rate);
#endif
