#ifndef HSD_TO_TMS_GRAY
#define HSD_TO_TMS_GRAY

#include "png.h"
#include <algorithm>
#include <assert.h>
#include "HimawariStandardData.h"
#include "TileMapService.h"
#include "CloudTopAltitude.h"

namespace hsd2tms {

void createTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData,
                DataType aType,
                uint32_t aBand);
void createTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                const HimawariStandardData& aData,
                DataType aType,
                uint32_t aBand0, uint32_t aBand1, uint32_t aBand2);
void shrinkTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                DataType aType,
                uint32_t aBand);
void shrinkTile(uint32_t aZ, uint32_t aX, uint32_t aY,
                DataType aType,
                uint32_t aBand0, uint32_t aBand1, uint32_t aBand2);

void createAltitudeFile(uint32_t aZ, uint32_t aX, uint32_t aY,
                        const HimawariStandardData& aData,
                        const CloudTopAltitude& aTable);
} // hsd2tms
#endif
