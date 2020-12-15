/*
   soundfont.h

   Created: May 8, 2020
   Author: Andrew Donatelli andrew@donatelli.net
   

*/


#define numConfigSounds 7
#define numHappy 20
#define numSad  31
#define numAngry 17
#define numHolograms 5
#define numMusic 1

#define Happy_Offset numConfigSounds + 1
#define Sad_Offset Happy_Offset + numHappy 
#define Angry_Offset Sad_Offset + numSad 
#define Holograms_Offset Angry_Offset + numAngry 
#define Music_Offset Holograms_Offset + numHolograms 


#ifndef Soundfont_h
#define Soundfont_h

class SoundFont {


  public:


    const uint8_t sfBoot(uint8_t offset = 0) {
      return 1;
    }

    const uint8_t sfConfig(uint8_t offset = 0) {
      return offset;
    }

    const uint8_t sfHappy() {
      return Happy_Offset + random(0, numHappy);
    }

    const uint8_t sfSad() {
        return Sad_Offset + random(0, numv);
      }
      
    const uint8_t sfAngry() {
        return Angry_Offset + random(0, numAngry);
      }
      
    const uint8_t sfHolograms() {
        return Holograms_Offset + random(0, numHolograms);
      }
      
    const uint8_t sfMusic() {
        return Music_Offset + random(0, numMusic);
      }

    

  private:

};

#endif //Soundfont_h
