#include "hal.h"
#include "app.h"

static char *app_name = 0;

char *main_app_name_get(void)
{
    return app_name;
}

int main(int argc, char *argv[])
{
    app_name = malloc(strlen(argv[0]) + 1);
    strcpy(app_name, argv[0]);

    hal_init();
    app_init();

    app_loop();
    
    app_deinit();
    hal_deinit();

    free(app_name);
    
    return 0;
}
