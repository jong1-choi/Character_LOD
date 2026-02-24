#ifndef MotionFitting_h
#define MotionFitting_h

#include <vector>
#include "Motion.h"

void FitMotionDirect(unsigned int level, const Motion& origin, Motion& fitted, const std::vector<int>& targets);
void FitMotionIK(unsigned int level, Motion& origin, Motion& fitted, const std::vector<int>& targets);
void FitMotion(unsigned int level, Motion& origin, Motion& fitted);

#endif /* MotionFitting_h */
