#ifndef _RESAMPLE_H
#define _RESAMPLE_H

#include <vector>
#include <iosfwd>

#include "filter.h"

void Resample(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & outputs);


#endif /* end of include guard: _RESAMPLE_H */
