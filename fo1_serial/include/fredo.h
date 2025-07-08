#define POT_RAW_TOPIC 60000
#define PULSE_RAW_TOPIC 60001
#define CTRL_MODE 0
#define TWIN_MODE 1
#define CALI_MODE 2


struct fredo_msg{
    double time;
    int pot_val_1;
    int pot_val_2;
    int pot_val_3;
};