#ifndef __EUROBIN_IOT_SOUND_H
#define __EUROBIN_IOT_SOUND_H

#include "ding.h"
#include "doorbell.h"

namespace eurobin_iot {
    namespace sound {
        static void play(const void* pcm, size_t size) {
            size_t bytes_written = 0;
			// we need SIGNED 8-bit PCM, headerless (of course)
  			i2s_write(Speak_I2S_NUMBER, pcm, size, &bytes_written, portMAX_DELAY);
        }
        static void ding() {
            play(ding_pcm, ding_size);
        }
        static void doorbell() {
            play(doorbell_pcm, doorbell_size);
        }
        
    }
}

#endif
