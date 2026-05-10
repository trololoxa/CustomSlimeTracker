#include <Arduino.h>

#include "app/tracker_app_context.hpp"

void setup() {
    trackerAppContextSetup();
}

void loop() {
    trackerAppContextLoop();
}
