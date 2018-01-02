#ifndef __HARD_DEBUG_H_
#define __HARD_DEBUG_H_
#include "../neolixMV.h"

namespace neolix
{
void hardebug();
void softdebug(int method);
void hardebug(depthData depth_, int rec[]);
void debug_rancas();
}

#endif // __HARD_DEBUG_H_
