#pragma once
#include "keys.h"

#ifdef __cplusplus
extern "C"
{
#endif
  typedef void (*esp32BleCallback)(AP10Keys_t key, AP10Action_t action);

  void SetEsp32BleCallback(esp32BleCallback callback);
  void BleInit(void);

#ifdef __cplusplus
}
#endif
