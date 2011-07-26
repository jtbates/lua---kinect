
#include "TH.h"
#include "luaT.h"

#include <libfreenect.hpp>

#define torch_(NAME) TH_CONCAT_3(torch_, Real, NAME)
#define torch_string_(NAME) TH_CONCAT_STRING_3(torch., Real, NAME)
#define kinect_(NAME) TH_CONCAT_3(kinect_, Real, NAME)

static const void* torch_FloatTensor_id = NULL;
static const void* torch_DoubleTensor_id = NULL;

#include "generic/kinect.cpp"
#include "THGenerateFloatTypes.h"

extern "C" {
  DLL_EXPORT int luaopen_libkinect(lua_State *L)
  {
    torch_FloatTensor_id = luaT_checktypename2id(L, "torch.FloatTensor");
    torch_DoubleTensor_id = luaT_checktypename2id(L, "torch.DoubleTensor");

    kinect_FloatInit(L);
    kinect_DoubleInit(L);

    return 1;
  }
}
