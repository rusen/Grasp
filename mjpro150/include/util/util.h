/*
 * util.h
 *
 *  Created on: 19 Sep 2017
 *      Author: rusi
 */

#ifndef INCLUDE_UTIL_UTIL_H_
#define INCLUDE_UTIL_UTIL_H_

#include <planner/GraspPlanner.h>
#include <string>

namespace Grasp{
std::string CreateXMLs(const char * base, GraspPlanner * planner, int objectId, int baseId);
void UploadFiles(const char * base, GraspPlanner * planner, int objectId, int baseId);
void UploadExtraFiles(const char * path);
}

#endif /* INCLUDE_UTIL_UTIL_H_ */
