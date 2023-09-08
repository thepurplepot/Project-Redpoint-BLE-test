#pragma once
#ifndef _UTILS_H
#define _UTILS_H

#include <string>

std::string getBTAddress();
void euler2quat(const float euler[], float q[]);

#endif // _UTILS_H