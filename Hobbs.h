#include <stdint.h>
#ifndef HOBBS_H
#define HOBBS_H

#include <time.h>

#define HOBBS_MAX_BUFFLEN  4096

extern time_t hobbs_counter;
extern int hobbs_state;

void StartHobbs(struct lg_master *pLgMaster);
void EndHobbs(struct lg_master *pLgMaster);
time_t ReadHobbs(char * HobbsName );
void WriteHobbs(time_t counter, char * HobbsName );
void DoHobbsSet(struct lg_master *pLgMaster, char * data, uint32_t respondToWhom);
void DoHobbsGet(struct lg_master *pLgMaster, char * data, uint32_t respondToWhom);
int HobbsCountersInit(struct lg_master *pLgMaster);
#endif
