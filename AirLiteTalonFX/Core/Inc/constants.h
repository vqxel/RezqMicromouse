/*
 * constants.h
 *
 *  Created on: Feb 12, 2025
 *      Author: mine215
 */

#pragma once

static const int PWM_ARR = 1024; // Note just for reference, real value is set in init

static const float VOLT_NORM_SLOPE = 0.00305532;
static const float VOLT_NORM_INT = 0.658423;

static const int ADC_TIMEOUT_MS = 100;

static const float NOM_VOLTAGE = 7.4;

static const float LEFT_KS = 0.0413704;
static const float LEFT_KV = 0.000895934;

static const float RIGHT_KS = 0.0220421;
static const float RIGHT_KV = 0.000810323;

// Distance Sensors
// L
static const float L_FACTOR = 30089.7773;
static const float L_POW = -0.835766;

// R
static const float R_FACTOR = 587887.835;
static const float R_POW = -1.21735;

// FL
static const float FL_FACTOR = 309608344;
static const float FL_POW = -2.02718;

// FR
static const float FR_FACTOR = 1373374.48;
static const float FR_POW = -1.32993;

static const int LEFT_THRESH = 200;
static const int RIGHT_THRESH = 200;
static const int FL_THRESH = 200;
static const int FR_THRESH = 200;
