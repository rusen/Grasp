#ifndef INCLUDE_SAVELOG_H_
#define INCLUDE_SAVELOG_H_

#include "mujoco.h"
#include "stdio.h"

// The function that writes the header.
void writeHeader(const mjModel* m, mjData* d, FILE * fp);

// The function that saves simulation data.
void savelog(const mjModel *m, mjData *d, FILE *out);

#endif /* INCLUDE_SAVELOG_H_ */
