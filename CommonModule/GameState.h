#pragma once
#include "Types.h"
struct GameState
{
	uint reserved = 2;
	uint stateSize;

	bool isPlaying;
};

