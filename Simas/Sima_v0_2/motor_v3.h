//motor control v3.1
//changelog: update to use ledcAttachChannel, ledcWrite cmd
#include <stdio.h>
#include <stdlib.h>

class Motor_control {

  public:

    void begin(uint8_t pin_1, uint8_t pin_2)
    {
      _pin[0] = pin_1;
      _pin[1] = pin_2;
      ledcAttach(_pin[0], _freq[0], _res[0]);
      ledcAttach(_pin[1], _freq[0], _res[0]);
    }
    
    void begin(uint8_t pin_1, uint8_t pin_2, uint8_t ledc_1, uint8_t ledc_2)  //motor setup, pin*2 + ledc*2
    {
      _pin[0] = pin_1;
      _pin[1] = pin_2;
      _ledc[0] = ledc_1;
      _ledc[1] = ledc_2;
      
      for(uint8_t i = 0; i <= 1; i++)
      {
        ledcAttachChannel(_pin[i], _freq[0], _res[0], _ledc[i]);
        delay(2);
      }
    }

    void begin(uint8_t pin_1a, uint8_t pin_1b, uint8_t pin_2a, uint8_t pin_2b, uint8_t ledc_1, uint8_t ledc_2)  //motor setup, pin*2 + ledc*2
    {
      _pin[0] = pin_1a;
      _pin[1] = pin_2a;
      _pin[2] = pin_1b;
      _pin[3] = pin_2b;
      _ledc[0] = ledc_1;
      _ledc[1] = ledc_2;
      
      for(uint8_t i = 0; i <= 1; i++)
      {
        ledcAttachChannel(_pin[i], _freq[0], _res[0], _ledc[i]);
        ledcAttachChannel(_pin[i +2], _freq[0], _res[0], _ledc[i]);
        delay(2);
      }
    }

    //dir = true for forward
    void speed(uint8_t speed, bool dir)
    { 
      for (_freqTarget = 0; _freqTarget <= _gears-1; _freqTarget++){
        if (speed >= _targ[_freqTarget]){
          break;
        }
      }

      if (_freqTarget != _freqSelc){
        if (_lastFreqUpdate <= millis()){
          changeFreq();
          _freqSelc = _freqTarget;
          _lastFreqUpdate = millis() + 60;
        }
      }

      if (speed > 16){
        if (_freqTarget >= 3){
          ledcWrite(_pin[dir], ((uint16_t)speed << 4));
        } else {
          ledcWrite(_pin[dir], speed);
        }
        ledcWrite(_pin[!dir], 0);
      } else {
        ledcWrite(_pin[dir], 0);
        ledcWrite(_pin[!dir], 0);
      }      
    }

    void changeFreq(){
      ledcChangeFrequency(_pin[0], _freq[_freqTarget], _res[_freqTarget]);
      ledcChangeFrequency(_pin[1], _freq[_freqTarget], _res[_freqTarget]);
    }

    void stop()
    {
      ledcWrite(_pin[0], 0);
      ledcWrite(_pin[1], 0);
    }

    void hold()
    {
      ledcWrite(_pin[0], 255);
      ledcWrite(_pin[1], 255);
    }

  protected:
    static const uint8_t _gears = 5;
    const uint8_t _targ[_gears] = {192, 140, 96, 56, 0};
    const uint32_t _freq[_gears] = {512, 384, 256, 32, 16};
    const uint8_t _res[_gears]   = {  8,   8,   8, 12, 12};

  private:

    //wheel gpio
    uint8_t _pin[4];
    uint8_t _ledc[2];
    uint8_t _freqTarget = 0;
    uint8_t _freqSelc = 0;
    uint32_t _lastFreqUpdate = 0;
};