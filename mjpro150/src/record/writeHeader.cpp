#include "record/savelog.h"
#include "string.h"

void writeHeader(const mjModel* m, mjData* d, FILE * fp){
	int header[6];
    header[0] = m->nq;
	header[1] = m->nv;
	header[2] = m->nu;
	header[3] = m->nmocap;
	header[4] = m->nsensordata;
	header[5] = strlen(m->names);
	fwrite(header, sizeof(int), 6, fp);
}
