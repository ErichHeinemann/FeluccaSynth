// Felucca Synth: six Felucca-Engines synthesis engines and stereo effects in one MPC instrument.
// The copied engine sources and their MIT licence are in src/felucca/ and licenses/.

#include "mpc_plugin.h"
#include "dsp/adsr.h"
#include "dsp/delay_line.h"
#include "dsp/onepole.h"
#include "dsp/smoother.h"
#include "dsp/voice.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#define NVOICE 8
#define NPART 1
#include "felucca/fe_tables.h"
#include "felucca/felucca_engine.h"

enum { P_ATK, P_REL, P_FTYPE, P_E0, P_E1, P_E2, P_E3, P_E4, P_E5, P_E6, P_E7, P_COUNT };

struct track {
    int16_t p[P_COUNT];
    voice_t v[NVOICE];
    drw_trk_t wheel_track;
    drw_vc_t wheel_voice[NVOICE];
};
typedef struct track track_t;

#include "felucca/dsp.c"
#include "felucca/eng_analog.c"
#include "felucca/eng_lofi.c"
#include "felucca/eng_trio.c"
#include "felucca/eng_wheel.c"
#include "felucca/eng_noise.c"
#include "felucca/eng_phase.c"

namespace {

constexpr int kNumEngines = 6;
constexpr int kNumEngineParams = 8;
constexpr int kNumVoices = NVOICE;
constexpr int kControlSize = CTL;
constexpr int kLofiArpParam = 6;

const engine_t* const kEngines[kNumEngines] = {
    &ENG_ANALOG, &ENG_LOFI, &ENG_TRIO, &ENG_WHEEL, &ENG_NOISE, &ENG_PHASE
};
const char* const kEnginePrefixes[kNumEngines] = {"a", "c", "t", "w", "n", "p"};
const char* const kEngineLabels[kNumEngines] = {"A", "C", "T", "W", "N", "P"};

enum ParamIndex
{
    kVolume = 0,
    kEngine,
    kAttack,
    kDecay,
    kSustain,
    kRelease,
    kBendRange,
    kAnalogFilter
};

int32_t clampEngineValue(int32_t value, int32_t lo, int32_t hi)
{
    return value < lo ? lo : value > hi ? hi : value;
}

float rawToPlain(const param_desc_t& d, int32_t raw)
{
    raw = clampEngineValue(raw, d.min, d.max);
    switch (d.fmt)
    {
        case F_PCT:
        case F_BIPCT:
            return (float)raw * 100.0f / 127.0f;
        case F_CUTOFF:
            return 30.0f * std::pow(16000.0f / 30.0f, (float)raw / 127.0f);
        default:
            return (float)raw;
    }
}

int32_t plainToRaw(const param_desc_t& d, float value)
{
    int32_t raw = d.min;
    switch (d.fmt)
    {
        case F_ENUM:
            raw = d.min + (int32_t)std::lround(value);
            break;
        case F_PCT:
        case F_BIPCT:
            raw = (int32_t)std::lround(value * 127.0f / 100.0f);
            break;
        case F_CUTOFF:
            value = mpc::clamp(value, 30.0f, 16000.0f);
            raw = (int32_t)std::lround(127.0f * std::log(value / 30.0f) / std::log(16000.0f / 30.0f));
            break;
        default:
            raw = (int32_t)std::lround(value);
            break;
    }
    return clampEngineValue(raw, d.min, d.max);
}

float envIndexToMs(int32_t value)
{
    return std::pow(5000.0f, (float)clampEngineValue(value, 0, 127) / 127.0f);
}

int16_t envMsToIndex(float ms)
{
    ms = mpc::clamp(ms, 1.0f, 5000.0f);
    return (int16_t)std::lround(127.0f * std::log(ms) / std::log(5000.0f));
}

int32_t q15(float value)
{
    return clampEngineValue((int32_t)std::lround(value * 32767.0f), 0, 32767);
}

float softLimit(float sample)
{
    const float magnitude = std::fabs(sample);
    if (magnitude > 0.95f)
    {
        const float excess = magnitude - 0.95f;
        sample = std::copysign(0.95f + 0.05f * excess / (excess + 0.05f), sample);
    }
    return sample;
}

struct EffectsProgram
{
    float delayTimeMs;
    float delayFeedback;
    float delayHighpass;
    float delayLowpass;
    float delayMix;
    bool delayPingPong;
    float reverbMix;
    float reverbSize;
    float reverbDamp;
};

class SchroederReverb
{
public:
    void prepare(double sampleRate)
    {
        sampleRate_ = sampleRate > 0.0 ? sampleRate : 44100.0;
        constexpr int combLengths[] = {1116, 1188, 1277, 1356};
        constexpr int allpassLengths[] = {556, 441};
        for (int channel = 0; channel < 2; ++channel)
        {
            for (int i = 0; i < 4; ++i)
            {
                const int offset = channel == 0 ? 0 : 23;
                combLength_[channel][i] = (int)std::lround(
                    (float)(combLengths[i] + offset) * (float)sampleRate_ / 44100.0f);
                comb_[channel][i].prepare(combLength_[channel][i] + 1);
            }
            for (int i = 0; i < 2; ++i)
            {
                const int offset = channel == 0 ? 0 : 31;
                allpassLength_[channel][i] = (int)std::lround(
                    (float)(allpassLengths[i] + offset) * (float)sampleRate_ / 44100.0f);
                allpass_[channel][i].prepare(allpassLength_[channel][i] + 1);
            }
        }
        reset();
    }

    void reset()
    {
        for (int channel = 0; channel < 2; ++channel)
        {
            for (int i = 0; i < 4; ++i)
            {
                comb_[channel][i].reset();
                damping_[channel][i].reset();
            }
            for (int i = 0; i < 2; ++i)
                allpass_[channel][i].reset();
        }
    }

    void setParameters(float size, float damp)
    {
        const float boundedSize = mpc::clamp(size, 0.0f, 100.0f);
        const float cutoff = 12000.0f - 10000.0f * mpc::clamp(damp, 0.0f, 100.0f) * 0.01f;
        feedback_ = 0.55f + 0.0034f * boundedSize;
        for (auto& channel : damping_)
            for (auto& filter : channel)
                filter.setCutoff(cutoff, sampleRate_);
    }

    void process(float inputL, float inputR, float& outputL, float& outputR)
    {
        outputL = processChannel(0, inputL);
        outputR = processChannel(1, inputR);
    }

private:
    float processChannel(int channel, float input)
    {
        float signal = 0.0f;
        for (int i = 0; i < 4; ++i)
        {
            const float delayed = comb_[channel][i].readInt(combLength_[channel][i]);
            const float damped = damping_[channel][i].process(delayed);
            comb_[channel][i].write(input + damped * feedback_);
            signal += delayed * 0.25f;
        }
        for (int i = 0; i < 2; ++i)
        {
            const float delayed = allpass_[channel][i].readInt(allpassLength_[channel][i]);
            const float output = delayed - 0.5f * signal;
            allpass_[channel][i].write(signal + 0.5f * output);
            signal = output;
        }
        return signal;
    }

    double sampleRate_ = 44100.0;
    float feedback_ = 0.75f;
    int combLength_[2][4]{};
    int allpassLength_[2][2]{};
    mpc::dsp::DelayLine comb_[2][4];
    mpc::dsp::DelayLine allpass_[2][2];
    mpc::dsp::OnePoleLP damping_[2][4];
};

} // namespace

class FeluccaSynth : public mpc::Plugin
{
public:
    FeluccaSynth() : mpc::Plugin(makeInfo())
    {
        for (auto& engineParams : engineParamIndex_)
            for (int& index : engineParams)
                index = -1;

        addParam(mpc::floatParam("volume", "Volume", -36.0f, 6.0f, -18.0f, "dB", 1));
        addParam(mpc::choiceParam("engine", "Engine",
                                  {"Analog", "LoFi", "Trio", "Wheel", "Noise", "Phase"}, 0));
        addParam(mpc::logParam("attack", "Attack", 1.0f, 5000.0f, 8.0f, "ms", 0));
        addParam(mpc::logParam("decay", "Decay", 1.0f, 5000.0f, 180.0f, "ms", 0));
        addParam(mpc::floatParam("sustain", "Sustain", 0.0f, 100.0f, 75.0f, "%", 0));
        addParam(mpc::logParam("release", "Release", 1.0f, 5000.0f, 250.0f, "ms", 0));
        addParam(mpc::intParam("bend", "Bend", 0, 24, 2, "st"));
        addParam(mpc::choiceParam("ana_filter", "A Filter", {"LP", "BP", "HP"}, 0));

        for (int engine = 0; engine < kNumEngines; ++engine)
            addEngineParameters(engine);

        delayTime_ = addParam(mpc::logParam("fx_dly_time", "Dly Time", 10.0f, 2000.0f, 375.0f, "ms", 0));
        delayFeedback_ = addParam(mpc::floatParam("fx_dly_fb", "Dly FB", 0.0f, 90.0f, 25.0f, "%", 0));
        delayHighpass_ = addParam(mpc::logParam("fx_dly_hpf", "Dly HPF", 20.0f, 2000.0f, 100.0f, "Hz", 0));
        delayLowpass_ = addParam(mpc::logParam("fx_dly_lpf", "Dly LPF", 500.0f, 18000.0f, 8000.0f, "Hz", 0));
        delayMix_ = addParam(mpc::floatParam("fx_dly_mix", "Dly Mix", 0.0f, 100.0f, 0.0f, "%", 0));
        delayPingPong_ = addParam(mpc::boolParam("fx_dly_pingpong", "Ping Pong", false));
        reverbMix_ = addParam(mpc::floatParam("fx_rev_mix", "Rev Mix", 0.0f, 100.0f, 0.0f, "%", 0));
        reverbSize_ = addParam(mpc::floatParam("fx_rev_size", "Rev Size", 0.0f, 100.0f, 55.0f, "%", 0));
        reverbDamp_ = addParam(mpc::floatParam("fx_rev_damp", "Rev Damp", 0.0f, 100.0f, 50.0f, "%", 0));

        addPresetProgram("Analog Saw", 0, 0, {280.0f, 26.0f, 120.0f, 7000.0f, 10.0f, true, 8.0f, 35.0f, 55.0f});
        addPresetProgram("Analog Pad", 0, 1, {420.0f, 32.0f, 90.0f, 6500.0f, 7.0f, false, 25.0f, 75.0f, 45.0f});
        addPresetProgram("LoFi Wave", 1, 3, {300.0f, 25.0f, 200.0f, 5000.0f, 8.0f, true, 12.0f, 45.0f, 65.0f});
        addPresetProgram("Trio Bass", 2, 0, {160.0f, 15.0f, 160.0f, 6500.0f, 3.0f, false, 0.0f, 45.0f, 60.0f});
        addPresetProgram("Wheel Organ", 3, 0, {350.0f, 28.0f, 100.0f, 6500.0f, 4.0f, true, 16.0f, 55.0f, 50.0f});
        addPresetProgram("Noise Metal", 4, 3, {410.0f, 34.0f, 350.0f, 4500.0f, 5.0f, true, 20.0f, 68.0f, 70.0f});
        addPresetProgram("Phase Brass", 5, 0, {280.0f, 22.0f, 120.0f, 7500.0f, 8.0f, true, 15.0f, 50.0f, 45.0f});
        addPresetProgram("Phase Bell", 5, 4, {520.0f, 32.0f, 140.0f, 6000.0f, 8.0f, true, 30.0f, 85.0f, 32.0f});
    }

    void prepare(double sampleRate, int maxBlockSize) override
    {
        (void)maxBlockSize;
        sampleRate_ = sampleRate;
        volumeSlew_ = 1.0f - std::exp(-1.0f / (0.01f * (float)sampleRate_));
        const int maxDelay = (int)std::lround(sampleRate_ * 2.0);
        for (auto& delay : delays_)
            delay.prepare(maxDelay);
        reverb_.prepare(sampleRate_);
        delayTimeSmoother_.setTime(80.0f, sampleRate_);
        delayFeedbackSmoother_.setTime(20.0f, sampleRate_);
        delayMixSmoother_.setTime(20.0f, sampleRate_);
        reverbMixSmoother_.setTime(20.0f, sampleRate_);
        for (auto& envelope : envelopes_)
            envelope.setSampleRate(sampleRate_);
        reset();
    }

    void reset() override
    {
        std::memset(&track_, 0, sizeof(track_));
        allocator_.reset();
        for (auto& envelope : envelopes_)
            envelope.reset();
        for (int channel = 0; channel < 2; ++channel)
        {
            delays_[channel].reset();
            delayHighpassFilters_[channel].reset();
            delayLowpassFilters_[channel].reset();
        }
        reverb_.reset();
        for (uint32_t& tick : ampTick_)
            tick = UINT32_MAX;
        selectedEngine_ = -1;
        noteAge_ = 0;
        controlTick_ = 0;
        volumeGain_ = dbToGain(get(kVolume));
        targetGain_ = volumeGain_;
        const float maxDelay = std::max(1.0f, (float)delays_[0].maxDelay());
        delayTimeSmoother_.snap(mpc::clamp(get(delayTime_) * 0.001f * (float)sampleRate_, 1.0f, maxDelay));
        delayFeedbackSmoother_.snap(get(delayFeedback_) * 0.01f);
        delayMixSmoother_.snap(get(delayMix_) * 0.01f);
        reverbMixSmoother_.snap(get(reverbMix_) * 0.01f);
    }

    void process(mpc::ProcessContext& ctx) override
    {
        if (ctx.numFrames <= 0 || ctx.outputs == nullptr || ctx.outputs[0] == nullptr)
            return;

        updateSettings();
        const engine_t* engine = kEngines[selectedEngine_];
        targetGain_ = dbToGain(get(kVolume));
        const float maxDelay = std::max(1.0f, (float)delays_[0].maxDelay());
        delayTimeSmoother_.setTarget(mpc::clamp(get(delayTime_) * 0.001f * (float)sampleRate_, 1.0f, maxDelay));
        delayFeedbackSmoother_.setTarget(get(delayFeedback_) * 0.01f);
        delayMixSmoother_.setTarget(get(delayMix_) * 0.01f);
        reverbMixSmoother_.setTarget(get(reverbMix_) * 0.01f);
        for (int channel = 0; channel < 2; ++channel)
        {
            delayHighpassFilters_[channel].setCutoff(get(delayHighpass_), sampleRate_);
            delayLowpassFilters_[channel].setCutoff(get(delayLowpass_), sampleRate_);
        }
        reverb_.setParameters(get(reverbSize_), get(reverbDamp_));
        const float attack = get(kAttack);
        const float decay = get(kDecay);
        const float sustain = get(kSustain) * 0.01f;
        const float release = get(kRelease);
        for (auto& envelope : envelopes_)
            envelope.set(attack, decay, sustain, release);

        int frame = 0;
        int midiIndex = 0;
        while (frame < ctx.numFrames)
        {
            while (midiIndex < ctx.numMidiEvents && ctx.midi[midiIndex].sampleOffset <= frame)
                handleMidi(ctx.midi[midiIndex++]);

            if ((frame % kControlSize) == 0)
            {
                ++controlTick_;
                if (engine->block)
                    engine->block(&track_);
            }

            const int controlEnd = std::min(ctx.numFrames, ((frame / kControlSize) + 1) * kControlSize);
            int segmentEnd = controlEnd;
            if (midiIndex < ctx.numMidiEvents)
            {
                const int nextEvent = mpc::clamp(ctx.midi[midiIndex].sampleOffset, frame, ctx.numFrames);
                if (nextEvent > frame)
                    segmentEnd = std::min(segmentEnd, nextEvent);
                else
                    continue;
            }

            renderSegment(ctx, frame, segmentEnd, *engine);
            frame = segmentEnd;
        }

    }

private:
    static mpc::PluginInfo makeInfo()
    {
        mpc::PluginInfo info;
        info.name = "Felucca Synth";
        info.vendor = "FeluccaMPC";
        info.uniqueId = MPC_FOURCC('F', 'W', 'd', 'b');
        info.version = 1100;
        info.isInstrument = true;
        info.numInputs = 0;
        info.numOutputs = 2;
        return info;
    }

    void addEngineParameters(int engine)
    {
        const engine_t& e = *kEngines[engine];
        for (int slot = 0; slot < kNumEngineParams; ++slot)
        {
            if (engine == 1 && slot == kLofiArpParam)
                continue;

            const param_desc_t& d = e.edit[slot];
            const std::string id = std::string(kEnginePrefixes[engine]) + "_" + d.label;
            const std::string name = std::string(kEngineLabels[engine]) + " " + d.label;
            const float def = rawToPlain(d, d.def);
            int index = -1;

            if (d.fmt == F_ENUM)
            {
                std::vector<std::string> choices;
                for (int32_t value = d.min; value <= d.max; ++value)
                {
                    const char* label = d.names ? d.names[value - d.min] : nullptr;
                    choices.emplace_back(label ? label : std::to_string(value));
                }
                index = addParam(mpc::choiceParam(id.c_str(), name.c_str(), std::move(choices), d.def - d.min));
            }
            else if (d.fmt == F_CUTOFF)
            {
                index = addParam(mpc::logParam(id.c_str(), name.c_str(), 30.0f, 16000.0f, def, "Hz", 0));
            }
            else if (d.fmt == F_PCT || d.fmt == F_BIPCT)
            {
                index = addParam(mpc::floatParam(id.c_str(), name.c_str(), 0.0f, 100.0f, def, "%", 0));
            }
            else
            {
                index = addParam(mpc::intParam(id.c_str(), name.c_str(), d.min, d.max, d.def, d.unit ? d.unit : ""));
            }

            engineParamIndex_[engine][slot] = index;
        }
    }

    std::vector<float> defaultProgramValues() const
    {
        std::vector<float> values((size_t)numParams());
        for (int i = 0; i < numParams(); ++i)
            values[(size_t)i] = param(i).def;
        return values;
    }

    void addPresetProgram(const char* name, int engine, int presetIndex, const EffectsProgram& effects)
    {
        const preset_t& preset = kEngines[engine]->presets[presetIndex];
        std::vector<float> values = defaultProgramValues();
        values[kEngine] = (float)engine;
        values[kAttack] = envIndexToMs(preset.env[0]);
        values[kDecay] = envIndexToMs(preset.env[1]);
        values[kSustain] = (float)preset.env[2] * 100.0f / 127.0f;
        values[kRelease] = envIndexToMs(preset.env[3]);
        for (int slot = 0; slot < kNumEngineParams; ++slot)
        {
            const int index = engineParamIndex_[engine][slot];
            if (index >= 0)
                values[(size_t)index] = rawToPlain(kEngines[engine]->edit[slot], preset.e[slot]);
        }
        values[(size_t)delayTime_] = effects.delayTimeMs;
        values[(size_t)delayFeedback_] = effects.delayFeedback;
        values[(size_t)delayHighpass_] = effects.delayHighpass;
        values[(size_t)delayLowpass_] = effects.delayLowpass;
        values[(size_t)delayMix_] = effects.delayMix;
        values[(size_t)delayPingPong_] = effects.delayPingPong ? 1.0f : 0.0f;
        values[(size_t)reverbMix_] = effects.reverbMix;
        values[(size_t)reverbSize_] = effects.reverbSize;
        values[(size_t)reverbDamp_] = effects.reverbDamp;
        addProgram(name, std::move(values));
    }

    void updateSettings()
    {
        const int engine = mpc::clamp(getInt(kEngine), 0, kNumEngines - 1);
        if (engine != selectedEngine_)
        {
            std::memset(&track_, 0, sizeof(track_));
            allocator_.reset();
            for (auto& envelope : envelopes_)
                envelope.reset();
            selectedEngine_ = engine;
            noteAge_ = 0;
        }

        track_.p[P_ATK] = envMsToIndex(get(kAttack));
        track_.p[P_REL] = envMsToIndex(get(kRelease));
        track_.p[P_FTYPE] = (int16_t)getInt(kAnalogFilter);
        for (int slot = 0; slot < kNumEngineParams; ++slot)
        {
            const int paramIndex = engineParamIndex_[engine][slot];
            if (paramIndex < 0)
            {
                track_.p[P_E0 + slot] = 0;
                continue;
            }
            const param_desc_t& desc = kEngines[engine]->edit[slot];
            track_.p[P_E0 + slot] = (int16_t)plainToRaw(desc, get(paramIndex));
        }
        allocator_.setBendRange((float)getInt(kBendRange));
    }

    void handleMidi(const mpc::MidiEvent& event)
    {
        if (event.isNoteOn())
        {
            const int index = allocator_.noteOn(mpc::clamp(event.note(), 0, 127),
                                                mpc::clamp(event.velocity(), 1, 127));
            voice_t& voice = track_.v[index];
            const bool wasSounding = voice.active && voice.stage != 0;
            uint32_t phases[3] = {voice.ph[0], voice.ph[1], voice.ph[2]};
            int32_t state[8];
            std::memcpy(state, voice.s, sizeof(state));

            voice.note = (uint8_t)event.note();
            voice.vel = (uint8_t)event.velocity();
            voice.gate = 1;
            voice.active = 1;
            voice.stage = 1;
            voice.age = ++noteAge_;
            voice.pitch16 = voice.pitch_cur = (int32_t)voice.note * 16;
            voice.fine = 0;
            if (!wasSounding)
                voice.env = voice.env_out = 0;
            kEngines[selectedEngine_]->note_on(&track_, &voice);
            if (wasSounding)
            {
                for (int i = 0; i < 3; ++i)
                    voice.ph[i] = phases[i];
                for (int i = 0; i < 8; ++i)
                    if (i != 2 && i != 3 && (kEngines[selectedEngine_]->keep >> i & 1u))
                        voice.s[i] = state[i];
            }

            envelopes_[index].noteOn();
            ampTick_[index] = UINT32_MAX;
        }
        else if (event.isNoteOff())
        {
            const int index = allocator_.noteOff(event.note());
            if (index >= 0)
                releaseVoice(index);
        }
        else if (event.isAllNotesOff())
        {
            allocator_.allNotesOff([this](int index) { releaseVoice(index); });
        }
        else if (event.isController() && event.controller() == 64)
        {
            allocator_.setSustain(event.controllerValue() >= 64,
                                  [this](int index) { releaseVoice(index); });
        }
        else if (event.isPitchBend())
        {
            allocator_.setPitchBend(event.pitchBend());
        }
    }

    void releaseVoice(int index)
    {
        if (index < 0 || index >= kNumVoices)
            return;
        track_.v[index].gate = 0;
        envelopes_[index].noteOff();
    }

    void renderSegment(mpc::ProcessContext& ctx, int from, int to, const engine_t& engine)
    {
        const int frames = to - from;
        if (frames <= 0)
            return;
        std::memset(output_, 0, sizeof(int32_t) * (size_t)frames);

        for (int index = 0; index < kNumVoices; ++index)
        {
            voice_t& voice = track_.v[index];
            if (!voice.active)
                continue;

            const float startLevel = envelopes_[index].level();
            float endLevel = startLevel;
            for (int i = 0; i < frames; ++i)
                endLevel = envelopes_[index].process();

            voice.env = (int32_t)std::lround(endLevel * (float)(1 << 24));
            const auto stage = envelopes_[index].stage();
            voice.stage = stage == mpc::dsp::ADSR::Stage::Release ? 3
                        : stage == mpc::dsp::ADSR::Stage::Idle ? 0
                        : stage == mpc::dsp::ADSR::Stage::Attack ? 1 : 2;
            voice.active = envelopes_[index].isActive() ? 1 : 0;
            if (!voice.active)
            {
                voice.env = 0;
                voice.stage = 0;
                allocator_.voiceFinished(index);
                continue;
            }

            const float velocity = (float)voice.vel / 127.0f;
            int32_t amp0 = q15(startLevel * velocity);
            int32_t amp1 = q15(endLevel * velocity);
            const int32_t envq15 = q15(endLevel);
            voice.env_out = amp1;
            if (engine.amp)
            {
                if (ampTick_[index] != controlTick_)
                {
                    cachedEngineAmp_[index] = engine.amp(&track_, &voice, envq15);
                    ampTick_[index] = controlTick_;
                }
                amp1 = q15((float)cachedEngineAmp_[index] / 32767.0f * velocity);
                amp0 = amp1;
                if (!voice.active)
                {
                    allocator_.voiceFinished(index);
                    continue;
                }
            }

            const float bent = allocator_.bendSemitones() * 16.0f;
            const int32_t pitch16 = clampEngineValue((int32_t)std::lround((float)voice.note * 16.0f + bent), 0, 2047);
            voice.pitch16 = pitch16;
            vmod_t mod{};
            mod.pitch16 = pitch16;
            mod.inc = pitch_inc((uint32_t)pitch16);
            mod.amp0 = amp0;
            mod.amp1 = amp1;
            mod.envq15 = envq15;
            mod.shape = 64 << 8;
            engine.render(&track_, &voice, output_, (uint32_t)frames, &mod);
        }

        float* left = ctx.outputs[0];
        float* right = ctx.numOutputs > 1 ? ctx.outputs[1] : nullptr;
        const bool pingPong = getBool(delayPingPong_);
        for (int i = 0; i < frames; ++i)
        {
            output_[i] = clampEngineValue(output_[i], -262144, 262144);
            volumeGain_ += volumeSlew_ * (targetGain_ - volumeGain_);
            const float dry = softLimit((float)output_[i] * (1.0f / 32768.0f) * volumeGain_);
            const float delaySamples = delayTimeSmoother_.process();
            const float feedback = delayFeedbackSmoother_.process();
            const float delayMix = delayMixSmoother_.process();
            const float reverbMix = reverbMixSmoother_.process();
            float wetL = delayHighpassFilters_[0].process(delays_[0].readLinear(delaySamples));
            float wetR = delayHighpassFilters_[1].process(delays_[1].readLinear(delaySamples));
            wetL = delayLowpassFilters_[0].process(wetL);
            wetR = delayLowpassFilters_[1].process(wetR);

            if (pingPong)
            {
                delays_[0].write(softLimit(dry + wetR * feedback));
                delays_[1].write(softLimit(wetL * feedback));
            }
            else
            {
                delays_[0].write(softLimit(dry + wetL * feedback));
                delays_[1].write(softLimit(dry + wetR * feedback));
            }

            const float afterDelayL = dry + (wetL - dry) * delayMix;
            const float afterDelayR = dry + (wetR - dry) * delayMix;
            float reverbL, reverbR;
            reverb_.process(afterDelayL, afterDelayR, reverbL, reverbR);
            float outL = afterDelayL + (reverbL - afterDelayL) * reverbMix;
            float outR = afterDelayR + (reverbR - afterDelayR) * reverbMix;
            if (delayMix != 0.0f || reverbMix != 0.0f)
            {
                outL = softLimit(outL);
                outR = softLimit(outR);
            }
            left[from + i] += outL;
            if (right)
                right[from + i] += outR;
        }
    }

    static float dbToGain(float db)
    {
        return std::pow(10.0f, db / 20.0f);
    }

    static constexpr char kUid[5] = "FWdb";
    track_t track_{};
    mpc::dsp::VoiceAllocator<kNumVoices> allocator_;
    mpc::dsp::ADSR envelopes_[kNumVoices];
    int engineParamIndex_[kNumEngines][kNumEngineParams]{};
    int selectedEngine_ = -1;
    uint32_t noteAge_ = 0;
    uint32_t controlTick_ = 0;
    uint32_t ampTick_[kNumVoices]{};
    int32_t cachedEngineAmp_[kNumVoices]{};
    double sampleRate_ = 44100.0;
    float volumeGain_ = 0.1259f;
    float targetGain_ = 0.1259f;
    float volumeSlew_ = 0.01f;
    int32_t output_[kControlSize]{};
    int delayTime_ = -1;
    int delayFeedback_ = -1;
    int delayHighpass_ = -1;
    int delayLowpass_ = -1;
    int delayMix_ = -1;
    int delayPingPong_ = -1;
    int reverbMix_ = -1;
    int reverbSize_ = -1;
    int reverbDamp_ = -1;
    mpc::dsp::DelayLine delays_[2];
    mpc::dsp::OnePoleHP delayHighpassFilters_[2];
    mpc::dsp::OnePoleLP delayLowpassFilters_[2];
    mpc::dsp::Smoother delayTimeSmoother_;
    mpc::dsp::Smoother delayFeedbackSmoother_;
    mpc::dsp::Smoother delayMixSmoother_;
    mpc::dsp::Smoother reverbMixSmoother_;
    SchroederReverb reverb_;
};

MPC_PLUGIN_ENTRY(FeluccaSynth)
