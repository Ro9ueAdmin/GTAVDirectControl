#pragma once
#include <string>
#include <vector>
#include <inc/types.h>

#include "Color.h"

void showText(float x, float y, float scale, const std::string &text, int font = 0, const Color &rgba = solidWhite, bool outline = true);
void showDebugInfo3D(Vector3 location, const std::vector<std::string> &textLines, Color backgroundColor = transparentGray, Color fontColor = solidWhite);
void showDebugInfo3DColors(Vector3 location, const std::vector<std::pair<std::string, Color>> &textLines, Color backgroundColor = transparentGray);
void showNotification(const std::string &message, int *prevNotification = nullptr);
void showSubtitle(const std::string &message, int duration = 2500);

void drawLine(Vector3 a, Vector3 b, Color c);
void drawSphere(Vector3 p, float scale, Color c);
void drawChevron(Vector3 pos, Vector3 dir, Vector3 rot, float scale, float arrow, Color c);
std::string getGxtName(Hash hash);
