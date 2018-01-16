#ifndef MYVISRL_SIMSOFT
#define MYVISRL_SIMSOFT
#include <stdint.h>


extern void simsoft_init(void);

extern uint16_t rl_maxsteps;

extern uint16_t failed_episodes_count;
extern uint8_t sequential_failed_episodes;

extern uint8_t ep_success, run_success;

// extern uint16_t rl_curmaxeps;
// extern uint16_t rl_initmaxeps;
// extern uint16_t rl_maxepsinc;
// extern uint16_t rl_maxeps;

// extern uint8_t rl_eps_increase;
extern uint16_t rl_cur_episodes_limit, rl_max_episodes_limit;
extern int16_t rl_cur_episodes_limit_change;
extern int8_t rl_cur_epsilon_change;

extern uint8_t rl_change_cur_episodes_limit(int16_t increase_by);
extern uint8_t rl_change_epsilon(int8_t change_by);
// extern uint8_t rl_dec_eps(void);
// extern uint8_t rl_inc_eps(void);
// extern uint8_t rl_inc_cur_episodes_limit(void);

extern uint8_t print_qdict(void);
extern uint8_t write_qdict(void);
extern uint8_t load_qdict(void);
extern uint8_t load_qdict_fromtxt(void);

extern uint8_t copy_qdict(void);
extern uint8_t copy_logs(void);

extern uint8_t rl_write_episode_log(void);
extern uint8_t rl_write_step_log(void);

uint8_t rl_resetrun(void);

uint8_t snprint_fail(int8_t errorcode);
extern uint8_t setup_run_fold(void);
extern uint8_t setup_sess_fold(void);
extern uint8_t simsoft_cleanup(void);
extern uint8_t save_run_metadata(void);

extern uint8_t runnum;
extern uint8_t endrun, endsess;

extern uint8_t rl_maxruns;
extern uint8_t rl_addruncounter(void);

#endif
