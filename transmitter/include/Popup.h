#pragma once

#include <Arduino.h>

int    openPopupMultiChoice(String header, String choices[], int numChoices, int hovered);
long   openPopupNumber(String header, long initialValue, long minValue, long maxValue);
String openPopupString(String header, String initialValue, int stringLength);
