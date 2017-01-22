#ifndef ATT_EST_H
#define ATT_EST_H
#include "../MessageTypes/type_methods.h"

void attitude_estimator_start(void);
void attitude_init(void);
void attAcquire(att_t *att);
void attBlockingAcquire(att_t *att);
#endif
