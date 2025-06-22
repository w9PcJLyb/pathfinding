#pragma once

#include "env.h"

class AbsPathFinder {
    public:
        AbsPathFinder() {};
        virtual ~AbsPathFinder() {};
        virtual Path find_path(int start, int end) = 0;
};
