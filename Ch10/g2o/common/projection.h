#ifndef PROJECTION_H
#define PROJECTION_H

#include "tools/rotation.h"

template<typename T>
inline bool CamProjectionWithDistortion(const T* camera, const T* point, T* predictions)
{
    T p[3];
    AngleAxisRotatePoint(camera, point, p);

    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    T xp = -p[0] / p[2];
    T yp = -p[1] / p[2];

    const T& l1 = camera[7];
    const T& l2 = camera[8];

    T r2 = xp * xp + yp * yp;
    T distortion = T(1.0) + r2 * (l1 + l2 * r2);

    const T& focal = camera[6];
    predictions[0] = focal * distortion * xp;
    predictions[1] = focal * distortion * yp;

    return true;
}

#endif
