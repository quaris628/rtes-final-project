#include "game.h"
#include <stdbool.h>

void run() {
  STATE state = MAIN;
  while (true)
  {
    switch (state)
    {
    case PROBLEM:
      problemState();
      break;
    case END:
      problemEndState();
      break;
    default:
      mainMenuState();
      break;
    }
  } 
}

void mainMenuState() {

}

void problemState() {

}

void problemEndState() {

}