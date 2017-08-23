#include "record/savelog.h"

void savelog(const mjModel *m, mjData *d, FILE *out){
	// Print out relevant data to re-create the simulation later.
    int recsz = 1 + m->nq + m->nv + m->nu + 7*m->nmocap + m->nsensordata;

    // Create data.
    float data[recsz];
    data[0] = (float) d->time;
    mju_n2f(data+1, d->qpos, m->nq);
    mju_n2f(data+1+m->nq, d->qvel, m->nv);
    mju_n2f(data+1+m->nq+m->nv, d->ctrl, m->nu);
    mju_n2f(data+1+m->nq+m->nv+m->nu, d->mocap_pos, 3*m->nmocap);
    mju_n2f(data+1+m->nq+m->nv+m->nu+3*m->nmocap, d->mocap_quat, 4*m->nmocap);
    mju_n2f(data+1+m->nq+m->nv+m->nu+7*m->nmocap, d->sensordata, m->nsensordata);

	// read data, print size
	size_t nn = fwrite(data, recsz*sizeof(float), 1, out);
}
