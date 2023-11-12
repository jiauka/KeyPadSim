#pragma once

typedef enum
{
  AP10ActionStart = 0,
  AP10ActionStop,
  AP10ActionContinue,
} AP10Action_t;

typedef enum
{
  AP10KeyPlusOne = 0,
  AP10KeyPlusTen,
  AP10KeyMinusOne,
  AP10KeyMinusTen,
  AP10KeyAuto,
  AP10KeyMode,
  AP10KeyStandby,
  AP10KeyNone // just when released
} AP10Keys_t;
