/*
  Pyro - All functions relating to pyrotechnics
*/


#ifndef PYRO_H
#define PYRO_H

#include <Arduino.h>

extern double pyro1_cont, pyro2_cont;

void pyro_init();
void pyro1_fire(double dt);
void pyro2_fire(double dt);

#endif