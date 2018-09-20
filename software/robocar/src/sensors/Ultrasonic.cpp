//
// Created by christian on 14.09.18.
//

#include "sensors/Ultrasonic.h"
#include <iostream>
#include <thread>


/**
 * Constructor for a Ultrasonic instance
 *
 * USAGE:
 *      Ultrasonic* us = new Ultrasonic(4, 5);
 *
 * ATTENTION: WIRINGPI pin numbering is used!
 *
 * @param TriggerPin: Int gpio number
 * @param EchoPin Int gpio number
 */
Ultrasonic::Ultrasonic(int TriggerPin, int EchoPin) : Trigger(TriggerPin), Echo(EchoPin) {

    wiringPiSetup();

    pinMode(Trigger, OUTPUT);
    pinMode(Echo, INPUT);

    digitalWrite(Trigger, LOW);

}

Ultrasonic::~Ultrasonic() {

}


double Ultrasonic::currentDistance() {

    std::chrono::steady_clock::time_point pulseStart;
    std::chrono::steady_clock::time_point pulseEnd;
    double pulseDuration, distance;

    digitalWrite(Trigger, HIGH);
    usleep(10);
    digitalWrite(Trigger, LOW);

    while (digitalRead(Echo) == LOW) {
        pulseStart = std::chrono::steady_clock::now();
    }

    while (digitalRead(Echo) == HIGH) {
        pulseEnd = std::chrono::steady_clock::now();
    }

    pulseDuration = std::chrono::duration_cast<std::chrono::microseconds>(pulseEnd - pulseStart).count();

    distance = pulseDuration * 1e-6 * 17150;

    return distance;
}
