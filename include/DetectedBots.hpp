#pragma once
#include "DataDesc.hpp"

struct BotLocation final {
    unsigned char id;
    double x, y;
};

struct BotsLocation final {
    std::vector<BotLocation> data;
};

ACTOR_PROTOCOL_DEFINE(sync_position_atom, BotsLocation);
