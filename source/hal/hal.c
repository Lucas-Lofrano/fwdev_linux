#include "hal.h"

void hal_deinit(void)
{

}

void hal_init(void)
{
    utl_dbg_init();
    utl_dbg_mod_enable(UTL_DBG_MOD_PORT);
}
