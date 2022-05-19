//
// Created by idan alperin on 19/05/2022.
//

#ifndef MAIN_CPP_RELAY_H
#define MAIN_CPP_RELAY_H


class Relay {
private:
    uint8_t pin;
    bool status = false;
public:
    Relay(uint8_t pin_) : pin(pin_) {
        pinMode(pin, OUTPUT);
        off();
    }

    void on() {
        status = true;
        digitalWrite(pin, HIGH);
    }

    void off() {
        status = false;
        digitalWrite(pin, LOW);
    }

    bool toggle() {
        status ? off() : on();
        return status;
    }
};


#endif //MAIN_CPP_RELAY_H
