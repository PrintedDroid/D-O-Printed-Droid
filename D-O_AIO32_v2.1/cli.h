/**
 * Serial CLI for D-O AIO32 v2.1
 *
 * Single-line command interface over USB-CDC (Serial). Designed so all
 * runtime-configurable parameters of the droid can be inspected and
 * changed without reflashing. Modeled after the Mega v3.4 configuration
 * menu but flattened to command-lines instead of nested menus — better
 * for copy/paste, scripting and quick tweaks.
 *
 * Entry points:
 *   cliBegin()   — call once in setup() after Serial.begin(). Optional
 *                  banner + prompt.
 *   cliUpdate()  — call every iteration of loop(). Non-blocking: reads
 *                  Serial chars into an internal buffer, dispatches on
 *                  newline.
 *
 * Type `help` at the prompt for a category overview, `help <category>`
 * for the commands of that category.
 */

#ifndef CLI_H
#define CLI_H

#include <Arduino.h>

void cliBegin();
void cliUpdate();

#endif // CLI_H
