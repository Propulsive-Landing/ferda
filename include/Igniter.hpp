#pragma once

#include <tuple>

class Igniter
{
    public:
        enum IgnitionSpecifier {
            LAUNCH,
            ABORT
        };

        Igniter() = default;
        void Ignite(IgnitionSpecifier ignite);
};
