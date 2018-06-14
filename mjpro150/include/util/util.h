/*
 * util.h
 *
 *  Created on: 19 Sep 2017
 *      Author: rusi
 */

#ifndef INCLUDE_UTIL_UTIL_H_
#define INCLUDE_UTIL_UTIL_H_

#include <planner/GraspResult.h>
#include <planner/GraspPlanner.h>
#include <string>

namespace Grasp{
std::string CreateXMLs(const char * base, GraspPlanner * planner, int objectId, int baseId, int runCount, bool reSimulateFlag);
void UploadFiles(const char * base, GraspPlanner * planner, int objectId, int baseId, bool keepCollidingGrasps, bool reSimulateFlag);
void RemoveOldTmpFolders(const char * modelFolder, bool reSimulateFlag);
GraspResult * readGraspData(const char * fileName);
}

#endif /* INCLUDE_UTIL_UTIL_H_ */
