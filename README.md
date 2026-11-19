# FeluccaSynth

FeluccaSynth is an eight-voice, one-track MPC instrument that ports the six
MIT-licensed Felucca-Engines into the MPC Plugin Builder SDK. The original
Felucca firmware targets the M-VAVE FM-1; this project is an independent MPC
plugin adaptation, not the complete firmware.

## Credits for Felucac and Felucca-Engines:
https://github.com/hugelton/Felucca-Engines ( MIT )
https://github.com/hugelton/Felucca ( GPL3.0 )
>> Leo Kuroshita (@kurogedelic), Hügelton Instruments

## Features

- Six selectable synthesis engines: ANALOG, LOFI, TRIO, WHEEL, NOISE and PHASE.
- Eight-voice polyphony with MIDI notes, velocity, sustain and pitch bend.
- One engine is active per plugin instance. All six VCO tabs remain accessible,
  so you can edit each engine's settings even when it is not selected.
- Global ADSR controls and eight built-in programs.
- Stereo delay with feedback, high-pass and low-pass filters, and ping-pong
  routing; stereo room reverb with size and damping controls.
- Effect settings and engine parameters are saved with the MPC project/plugin
  state. The built-in programs also have their own effect settings.
- Custom MAIN, VCO and EFFECTS pages, with the selected engine name shown on
  MAIN and a separate tab for each engine.

### Startup and CPU

The six engine sources were written for a small 32-bit microcontroller and use
compact fixed-point DSP. FeluccaSynth does not load a large sample library, so
startup is designed to be quick and memory use modest. The synthesis engines
are designed for low CPU use; actual load depends on the MPC model, voice count
and effects settings. Desktop simulator timings are not a substitute for
measurement on MPC hardware.

## Felucca-Engines

FeluccaSynth uses the six MIT-licensed engines from the separate
Felucca-Engines project. Their main sound-design features are:

| Engine | Sound and controls |
| --- | --- |
| **ANALOG** | Two band-limited oscillators, hard sync, sub oscillator, noise, drive and a low/band/high-pass state-variable filter. |
| **LOFI** | Chip-style pulse, triangle, saw and shift-register noise, plus internal wave memory, bit-depth reduction, sample hold, pitch sweep and a stepped envelope. |
| **TRIO** | Three interacting oscillators with ring modulation, hard sync and combined waves, feeding a multimode filter. |
| **WHEEL** | Tonewheel organ with nine drawbar partials, registrations, percussion, key click, drive and rotary effect. |
| **NOISE** | White, pink and brown noise, random impulses and shift-register noise through a resonant filter, with drift and bit crushing. |
| **PHASE** | Phase-distortion waves, a second wave line, sub oscillator and sine wavefolder. |

These are sound engines, not the whole FM-1 firmware. FeluccaSynth also
includes an MPC-specific voice host and its own stereo delay/reverb processing.

## UI and programs

Choose the active engine on MAIN. Its name updates below the Engine control.
The six engine tabs are grouped under VCO and remain manually accessible
regardless of which engine is active. The EFFECTS tab contains the delay and
reverb controls.

Factory programs:

- Analog Saw
- Analog Pad
- LoFi Wave
- Trio Bass
- Wheel Organ
- Noise Metal
- Phase Brass
- Phase Bell

The custom skin is in `skins/`; `build.py` copies it into the plugin package.
The PNG backgrounds are in `assets/`. Edit `skins/TUI.json` for the custom UI;
the copy under `build/` is replaced on the next build.

## Scope and known limitations

- This is one stereo instrument track per plugin instance, not Felucca's
  four-part FM-1 firmware.
- Only the selected engine produces sound. The other engine tabs remain
  available for editing their parameters.
- Felucca's track sequencing, song mode, web editor/emulator and modulation
  matrix are not included.
- The LOFI engine's arpeggio is disabled; this plugin does not include an
  arpeggiator or send MIDI back to the MPC.
- The separate Felucca sample engine and its sample pack are not used.
- The delay and reverb are MPC plugin implementations, not the full Felucca
  firmware effects bus.
- No confirmed bugs are currently known. Test on the target MPC as well as with
  the host simulator; desktop CPU measurements do not predict MPC performance.

## Build and test

Run these commands from the MPC Plugin Builder SDK root. The full build compiles
the native test binary and both MPC architectures (ARM32 and ARM64), packages
the skin, and validates the plugin:

```sh
python3 tools/build.py examples/FeluccaSynth
```

Run the host simulator test on macOS:

```sh
.sdk-tools/host_sim examples/FeluccaSynth/build/native/felucca-synth.dylib test \
  --seconds 3 \
  --wav examples/FeluccaSynth/build/test.wav
```

The deployable folder is:

```text
examples/FeluccaSynth/build/FeluccaMPC - VST - Felucca Synth/
```

Copy the whole folder into a drive's `VST2 Plugins` folder. The MPC requires
MPC OS 3.10.1 or later, with **Enable Enhancements** and **External Plugin
Support** enabled under Preferences > Enhancements (Beta).

## Source and licence

The copied Felucca-Engines source and its MIT licence are in `src/felucca/` and
`licenses/Felucca-Engines-MIT.txt`. The generated fixed-point lookup tables are in `src/felucca/fe_tables.h`.
From `examples/FeluccaSynth/`, regenerate them with
`python3 tools/gen_engine_tables.py src/felucca/fe_tables.h`. The effect code
uses the SDK's MIT-licensed DSP utilities. The full Felucca firmware tree and
sample pack are not required to build or run this plugin.
