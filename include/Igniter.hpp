#pragma once

#include <tuple>

class Igniter
{
    public:
        enum IgnitionSpecifier {
            LAUNCH,
            LAND
        };

        Igniter() = default;
        void Ignite(IgnitionSpecifier ignite);
};
