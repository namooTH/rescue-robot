#pragma once

float curve(float x) {
    return 1.0f - 4.0f * (x - 0.5f) * (x - 0.5f);
}
