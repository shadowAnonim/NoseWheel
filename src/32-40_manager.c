#include "32-40_defs.h"

void nws_manager_step(
    Input_t* in,
    Output_t* out
)
{
    static Bus_t bus;
    static CU_Channel_t ch1;
    static CU_Channel_t ch2;
    nws_cu_step(
        in,
        &bus,
        &ch1,
        &ch2
    );
    nws_phys_step(
        in,
        &bus,
        out
    );
}
