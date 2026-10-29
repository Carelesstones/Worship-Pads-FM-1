/**
 * @file main.cpp
 * @brief Master Firmware Entry Point for M-VAVE FM-1 Ambient Pad Chord Sequencer
 * 
 * Hardware Target: JieLi AC791N Dual-Core RISC Microcontroller
 * Compiler:        JieLi PI32v2 Toolchain (-target pi32v2 -mcpu=r3 -mfprev1)
 * Audio Engine:    48,000 Hz, 12-Voice Polyphony, Schroeder/Moorer Modulated Freeverb
 * UI / Control:    240x240 OLED (Minimalist Sloop Theme), Dual Rotary Encoders,
 *                  Analog Potentiometer with Anti-Zipper Smoothing & Soft Takeover,
 *                  Sector-aligned Flash Memory (syscfg / fm1_flash)
 * 
 * Architecture & Concurrency Model:
 *   - Core 0 / Interrupt Context (High-Priority):
 *     fm1_engine_render() executes inside the hardware audio interrupt context
 *     via fm1_audio_irq(). It processes the sample-accurate AudioClock, advances
 *     ambient chord progressions, updates polyphonic ADSRs, runs the modulated
 *     Freeverb algorithm, and outputs signed Q23 PCM samples to the DAC DMA.
 *     This ensures the 48kHz audio clock NEVER stutters during UI interactions.
 * 
 *   - Core 1 / Control Loop (Background Task):
 *     main() executes the infinite while(1) loop. It continuously updates hardware
 *     controls (ADC scanning, 1-pole smoothing, soft takeover logic, encoder turns,
 *     pushbutton events) and performs band-by-band DMA blitting of the minimalist
 *     Sloop OLED user interface.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Official M-VAVE FM-1 SDK Headers */
#include "fm1.h"
#include "fm1_device.h"
#include "fm1_audio.h"
#include "fm1_engine_control.h"

/* Ambient Sequencer Firmware Architecture Headers */
#include "fm1_ambient_sequencer.h"
#include "fm1_sequencer.h"
#include "fm1_synth_engine.h"
#include "fm1_hardware_ui.h"

/* ============================================================================
 * STATIC MEMORY ALLOCATION (ZERO HEAP / NO MALLOC / NO NEW)
 * ============================================================================
 * Memory Footprint:
 *   - g_sequencer:        ~400 bytes
 *   - g_synth:            ~139.7 KB (including 8 comb + 4 allpass delay lines)
 *   - g_hw:               ~5.2 KB (including 4 cached songs & Sloop UI state)
 *   - g_floatAudioBlock:  1,024 bytes (128 stereo frames * 2 channels * 4 bytes)
 *   - g_screenBandBuffer: 7,680 bytes (240x16 pixels * 2 bytes RGB565)
 * Total Static RAM: ~154 KB (fits within the 256 KB internal SRAM budget)
 */
static SequencerState    g_sequencer;
static SynthEngine       g_synth;
static HardwareInterface g_hw;

/* Audio intermediate block buffer (stereo floating point) */
static float g_floatAudioBlock[FM1_BLOCK_FRAMES * 2];

/* OLED DMA band buffer (16 scanlines of 240 RGB565 pixels) */
static uint16_t g_screenBandBuffer[FM1_SCREEN_WIDTH * 16];

/* ============================================================================
 * AUDIO INTERRUPT SERVICE ROUTINE / HIGH-PRIORITY ENGINE HOOK
 * ============================================================================
 * Placed in zero-wait-state volatile RAM (.volatile_ram_code).
 * Called by app/board/audio.c inside fm1_audio_irq() every 128 frames (2.67ms).
 * 
 * Sample Conversion:
 *   Floats in [-1.0f, +1.0f] are converted to 24-bit signed Q23 audio samples
 *   (int32_t scaled by 8,388,607) for direct transmission to the DAC DMA ring.
 */
extern "C" FM1_AUDIO_CODE int fm1_engine_render(int32_t *output, unsigned frames) {
    if (!output || frames == 0) {
        return -1;
    }

    /* 
     * 1. Process Sample-Accurate Sequencer Clock & Advance Chords
     * The Sequencer clock is evaluated per-sample inside this block.
     */
    for (unsigned f = 0; f < frames; f++) {
        bool chordTriggered = g_sequencer.processSample();
        if (chordTriggered) {
            if (g_sequencer.isPlaying) {
                /* Crossfade new chord: Voices transition with 1.5s Attack & 3.0s Release */
                g_synth.playChord(g_sequencer.currentMidiNotes, g_sequencer.currentVelocity);
            }
            /* 
             * When the song ends, g_sequencer.isPlaying becomes false, but
             * advanceToNextChord() leaves currentMidiNotes active and stops triggering.
             * The final chord sustains infinitely through the synth voices and reverb tail.
             */
        }
    }

    /* 
     * 2. Render Polyphonic Oscillator Voices & Modulated Freeverb Wash
     * Sums 12 dual saw/sine voices and routes through 8 combs + 4 LFO allpasses.
     */
    g_synth.renderAudio(g_floatAudioBlock, (int)frames);

    /* 
     * 3. Convert Floating Point Output to Signed Q23 Hardware DAC Format
     * Interleaved stereo: output[2*i] = Left, output[2*i+1] = Right.
     */
    for (unsigned i = 0; i < frames * 2; i++) {
        float sample = g_floatAudioBlock[i];

        /* Hard brickwall limit to prevent integer overflow wrap-around */
        if (sample > 1.0f) sample = 1.0f;
        else if (sample < -1.0f) sample = -1.0f;

        /* Convert to Q23 format (-8388608 .. +8388607) */
        output[i] = (int32_t)(sample * 8388607.0f);
    }

    return 0; /* Success */
}

/* ============================================================================
 * EXTERNAL MIDI INPUT HOOK (DIN / USB / BLE)
 * ============================================================================
 * Allows external controllers or keyboards to play the synth or trigger chords.
 */
extern "C" void fm1_app_midi_received(unsigned source, const uint8_t *message, size_t bytes) {
    (void)source;
    if (!message || bytes < 2) return;

    uint8_t status = message[0] & 0xF0;
    uint8_t note   = message[1];
    uint8_t vel    = (bytes > 2) ? message[2] : 64;

    if (status == 0x90 && vel > 0) {
        /* Note On */
        g_synth.noteOn(note, vel);
    } else if (status == 0x80 || (status == 0x90 && vel == 0)) {
        /* Note Off */
        g_synth.noteOff(note);
    }
}

/* ============================================================================
 * FIRMWARE ENTRY POINT & MAIN CONTROL TASK
 * ============================================================================
 */
int main(void) {
    /* ------------------------------------------------------------------------
     * PHASE 1: HARDWARE INITIALIZATION
     * ------------------------------------------------------------------------ */
    /* 1. Initialize JieLi AC791N System Clocks (192 MHz PLL / 24 MHz Crystal) */
    uint32_t sysClock = fm1_device_clock();
    (void)sysClock;

    /* 2. Initialize Hardware Controls, OLED Screen, and Flash Storage */
    g_hw.init();

    /* 3. Load Active Song into Sequencer and Configure Soft Takeover */
    if (g_hw.totalSongs > 0) {
        const Song* initialSong = &g_hw.library[0];
        g_sequencer.loadSong(initialSong);
        g_hw.controls.onSongLoaded(initialSong);
        g_sequencer.play(); /* Auto-start playback on boot */
    }

    /* 4. Start Hardware Audio DAC & Interrupt Service Routine */
    int audioStatus = fm1_audio_start();
    if (audioStatus != 0) {
        g_hw.menu.setStatus("AUDIO DAC ERR", 3000);
    }

    /* ------------------------------------------------------------------------
     * PHASE 2: MAIN CONTROL LOOP (INFINITE TASK)
     * ------------------------------------------------------------------------ */
    uint32_t lastFrameMs = 0;
    unsigned currentBand = 0;

    while (1) {
        /*
         * Execution Step A: Poll Hardware Controls & Process Soft Takeover
         * - Reads 10-bit potentiometer ADC (Cutoff) on PB6
         * - Reads rotary encoders for Reverb Wet mix and UI navigation
         * - Evaluates 1-pole low-pass filtering and soft takeover pickup
         * - Dispatches encoder clicks (Song Load vs Sub-Menu)
         */
        g_hw.updateHardwareControls(g_sequencer, g_synth);

        /*
         * Execution Step B: Progressive Band-by-Band OLED Refresh (240x240 RGB565)
         * The FM-1 OLED uses DMA band rendering (15 bands of 16 scanlines each).
         * We verify that the previous DMA transfer has completed before writing
         * to the shared DMA buffer.
         */
        if (!fm1_screen_pending()) {
            unsigned bandY = currentBand * 16;

            /* Render minimalist Sloop theme UI into band buffer */
            g_hw.drawMenu(g_screenBandBuffer, bandY);

            /* Submit band buffer to hardware display controller */
            fm1_screen_submit(0, bandY, FM1_SCREEN_WIDTH, 16,
                              g_screenBandBuffer, FM1_SCREEN_DMA_PIXELS);

            /* Advance to next band (0..14) */
            currentBand = (currentBand + 1) % FM1_SCREEN_BANDS;
        }

        /*
         * Execution Step C: Frame Rate Governor / Background Task Yield
         * Limits UI loop spin to ~60 FPS while keeping control latency low (<2ms).
         */
        uint32_t now = fm1_millis();
        if (now - lastFrameMs < 16) {
            /* Small spin/yield to avoid bus saturation on Core 1 */
            for (volatile int i = 0; i < 200; i++) {
                __asm__ volatile("nop");
            }
        } else {
            lastFrameMs = now;
        }
    }

    return 0;
}