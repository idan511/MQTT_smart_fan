//
// Created by idan alperin on 19/05/2022.
//

#ifndef MAIN_CPP_LED_H
#define MAIN_CPP_LED_H


class LED {
private:
    uint8_t pin, channel, resolution = 10;
    uint16_t brightness = 1023, freq= 5000;
    bool status = false;
public:
    LED(uint8_t pin_, uint8_t channel_) : pin(pin_), channel(channel_){
        ledcSetup(channel, freq, resolution);
        ledcAttachPin(pin, channel);
        off();
    }

    void on() {
        status = true;
        ledcWrite(channel, brightness);
    }

    void off() {
        status = false;
        //analogWrite(pin, 0);
        ledcWrite(channel, 0);
    }

    LED& operator+=(int delta) {
        delta += brightness;
        if (delta > 255) {
            brightness = 255;
        } else if(delta < 0) {
            brightness = 0;
        } else {
            brightness = delta;
        }
        //analogWrite(pin, brightness);
        ledcWrite(channel, brightness);
        return *this;
    }

    bool toggle() {
        status ? off() : on();
        return status;
    }

    void set_brightness(uint16_t new_brightness) {
        brightness = new_brightness;
        if (status) ledcWrite(channel, brightness);
    }
};



#endif //MAIN_CPP_LED_H
