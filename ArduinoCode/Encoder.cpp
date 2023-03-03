#include "Encoder.h"

Encoder::Encoder() {
  Speed = 0;
  Theta = 0;
}

Encoder::Encoder(int pinA, int pinB) {
  Speed = 0;
  Theta = 0;
  //Создали энкодер
  enc = {INPUT_PULLUP, pinA, pinB};
}

void Encoder::Tick() {
  //enc.tickISR();
  enc.tick();
}

void Encoder::Update() {
  Theta = enc.counter * ANGLE_PER_TICK;
}
