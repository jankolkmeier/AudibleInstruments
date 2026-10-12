#include "plugin.hpp"
#include "stmlib/dsp/delay_line.h"
#include "stmlib/utils/gate_flags.h"
#include "beads/dsp/processor.h"
#include "beads/dsp/synth.h"


// Firmware block rate, independent of the codec sample rate
static const int BEADS_BLOCK_RATE = 4000;
static const int BEADS_MAX_BLOCK_SIZE = 12;
// Three 256k banks, as in the hardware
static const size_t BEADS_BANK_SIZE = 256 * 1024;
static const size_t BEADS_BUFFER_SIZE = 3 * BEADS_BANK_SIZE;
static const size_t BEADS_REVERB_BUFFER_SIZE = 32768;
static const uint32_t BEADS_BUFFER_TAG = 0x5a5a5246; // FRZZ

// Input gain range of the hardware. 0dB on the gain parameter is 2x, which puts 5V at full scale
static const float BEADS_INPUT_SCALE = 1.f / 10.f;
// Input limit in volts, before the gain stage
static const float BEADS_INPUT_LIMIT = 12.f;
static const float BEADS_MIN_GAIN = 1.f;
static const float BEADS_MAX_GAIN = 40.f;
// The firmware's dry gain is 0.75. Scale so that a fully dry signal is at unity at 0dB
static const float BEADS_OUTPUT_SCALE = 5.f / 0.75f;


// Attenurandomizer pot law from beads/cv_reader.h, [-1, 1] to [-1, 1]
static float beadsAttenuverter(float v) {
	v *= 0.5f;
	return (2.f / 0.47f) * v * std::max(std::fabs(v) - 0.03f, 0.f);
}


// Pitch pot law from beads/cv_reader.h, [-1, 1] to semitones with plateaus on musical intervals
static float beadsDetune(float v) {
	static const float intervals[8] = {0.f, 1.f, 3.f, 4.f, 7.f, 12.f, 24.f, 24.f};
	float sign = v < 0.f ? -1.f : 1.f;
	float detune = clamp(v * sign, 0.f, 1.f) * 6.f;
	int integral = std::min((int) detune, 6);
	float fractional = detune - integral;
	fractional = std::max((fractional - 1.f / 3.f) * 1.5f, 0.f);
	float a = intervals[integral];
	float b = intervals[integral + 1];
	return stmlib::Crossfade(a, b, stmlib::SmoothStep(fractional)) * sign;
}


// Inverse of beadsDetune() by bisection. Returns the middle of a plateau
static float beadsDetuneInverse(float semitones) {
	float edges[2];
	for (int edge = 0; edge < 2; edge++) {
		float lo = -1.f;
		float hi = 1.f;
		for (int i = 0; i < 24; i++) {
			float mid = (lo + hi) / 2.f;
			float detune = beadsDetune(mid);
			if (edge == 0 ? detune < semitones : detune <= semitones)
				lo = mid;
			else
				hi = mid;
		}
		edges[edge] = (lo + hi) / 2.f;
	}
	return (edges[0] + edges[1]) / 2.f;
}


struct BeadsPitchQuantity : ParamQuantity {
	float getDisplayValue() override {
		return beadsDetune(getValue());
	}

	void setDisplayValue(float displayValue) override {
		setValue(beadsDetuneInverse(displayValue));
	}
};


// Port of beads/gain_control.cc, which depends on the hardware codec frame format
struct BeadsGainControl {
	enum SoftClippingMode {
		SOFT_CLIPPING_MODE_OFF,
		SOFT_CLIPPING_MODE_ON,
		SOFT_CLIPPING_MODE_DYNAMIC
	};

	float peak = 0.5f;
	float agcPeak = 0.5f;
	float stableAgcPeak = 0.5f;
	float gain = 1.f;
	float manualGainLp = 1.f;
	float softClippingAmount = 0.f;
	SoftClippingMode softClippingMode = SOFT_CLIPPING_MODE_OFF;
	bool autoGain = false;
	float oneHertz = 1.f / 48000.f;
	size_t time = 0;
	size_t ignoreTime = 0;
	size_t activeTime = 0;

	void init(float sampleRate, SoftClippingMode softClippingMode, bool reset) {
		if (reset) {
			stableAgcPeak = agcPeak = peak = 0.5f;
			gain = 1.f;
			softClippingAmount = 0.f;
		}
		this->softClippingMode = softClippingMode;

		ignoreTime = size_t(sampleRate * 1.f);
		bool noReadjust = !reset && time && time >= activeTime;
		activeTime = size_t(sampleRate * (1.f + 5.f));
		time = noReadjust ? activeTime : 0;
		oneHertz = 1.f / sampleRate;
	}

	void restart() {
		time = ignoreTime;
	}

	bool adjusting() const {
		return time < activeTime && time > 0 && autoGain;
	}

	float vuMeter() const {
		return peak * gain;
	}

	void process(bool autoGain, float manualGain, beads::FloatFrame* inOut, size_t size) {
		const float hz = float(size) * oneHertz;
		if (time < ignoreTime) {
			agcPeak = 0.5f;
		}
		else if (time < activeTime) {
			// Reduce gain in 0.1s if a peak is detected, adjust in 0.5s if the level is too low
			SLOPE(agcPeak, peak, 10.f * hz, 2.f * hz);
			stableAgcPeak = agcPeak;
		}
		else {
			// Recover from a peak reduction in 2s, or in 100s below the level measured while learning
			SLOPE(agcPeak, peak, 5.f * hz, (peak > stableAgcPeak ? 0.5f : 0.01f) * hz);
		}

		if (time > ignoreTime || peak > 0.001f) {
			time += size;
		}
		if (time >= activeTime) {
			time = activeTime;
		}

		if (autoGain && !this->autoGain) {
			restart();
		}
		this->autoGain = autoGain;

		ONE_POLE(manualGainLp, manualGain, 0.01f);

		const float optimalGain = std::min(0.9f / std::min(agcPeak, 0.9f), BEADS_MAX_GAIN);
		const float targetGain = autoGain ? optimalGain : manualGainLp;

		float targetSoftClippingAmount;
		if (softClippingMode == SOFT_CLIPPING_MODE_OFF) {
			targetSoftClippingAmount = 0.f;
		}
		else if (softClippingMode == SOFT_CLIPPING_MODE_ON) {
			targetSoftClippingAmount = 1.f;
		}
		else {
			targetSoftClippingAmount = clamp(2.f * (manualGainLp / optimalGain - 0.75f), 0.f, 1.f);
		}

		stmlib::ParameterInterpolator gainModulation(&gain, targetGain, size);
		stmlib::ParameterInterpolator softClippingAmountModulation(&softClippingAmount, targetSoftClippingAmount, size);
		for (size_t i = 0; i < size; i++) {
			float l = inOut[i].l;
			float r = inOut[i].r;

			const float framePeak = std::max(std::fabs(l), std::fabs(r));
			SLOPE(peak, framePeak, 0.1f, 2.f * oneHertz);

			const float g = gainModulation.Next();
			l *= g;
			r *= g;

			if (!autoGain) {
				if (softClippingMode == SOFT_CLIPPING_MODE_OFF) {
					l = clamp(l, -1.f, 1.f);
					r = clamp(r, -1.f, 1.f);
				}
				else {
					// Crossfade between the hard-clipped and the soft-clipped signal
					const float amount = softClippingAmountModulation.Next();
					const float bias = 0.05f;
					const float softL = (1.f - bias) * stmlib::SoftClip(l + bias) - bias;
					const float softR = (1.f - bias) * stmlib::SoftClip(r + bias) - bias;
					l = stmlib::Crossfade(clamp(l, -1.f, 1.f), softL, amount);
					r = stmlib::Crossfade(clamp(r, -1.f, 1.f), softR, amount);
				}
			}

			inOut[i].l = l;
			inOut[i].r = r;
		}
	}
};


struct Beads : Module {
	enum ParamIds {
		FREEZE_PARAM,
		SEED_PARAM,
		MEDIA_PARAM,
		ASSIGN_PARAM,
		DENSITY_PARAM,
		TIME_PARAM,
		PITCH_PARAM,
		SIZE_PARAM,
		SHAPE_PARAM,
		FEEDBACK_PARAM,
		DRY_WET_PARAM,
		REVERB_PARAM,
		TIME_CV_PARAM,
		SIZE_CV_PARAM,
		SHAPE_CV_PARAM,
		PITCH_CV_PARAM,
		// Not on the panel, edited from the context menu
		INPUT_GAIN_PARAM,
		FEEDBACK_ASSIGN_PARAM,
		DRY_WET_ASSIGN_PARAM,
		REVERB_ASSIGN_PARAM,
		NUM_PARAMS
	};
	enum InputIds {
		ASSIGN_INPUT,
		TIME_INPUT,
		SIZE_INPUT,
		SHAPE_INPUT,
		PITCH_INPUT,
		DENSITY_INPUT,
		IN_L_INPUT,
		IN_R_INPUT,
		FREEZE_INPUT,
		SEED_INPUT,
		NUM_INPUTS
	};
	enum OutputIds {
		OUT_L_OUTPUT,
		OUT_R_OUTPUT,
		NUM_OUTPUTS
	};
	enum LightIds {
		FREEZE_LIGHT,
		SEED_LIGHT,
		ENUMS(MEDIA_LIGHT, 3),
		ENUMS(ASSIGN_LIGHTS, 3),
		ENUMS(VU_LIGHT, 2),
		NUM_LIGHTS
	};

	enum AssignDestinations {
		ASSIGN_FEEDBACK,
		ASSIGN_DRY_WET,
		ASSIGN_REVERB,
		ASSIGN_CUSTOM
	};

	// Audio L/R, seed gate, freeze gate
	dsp::SampleRateConverter<4> inputSrc;
	dsp::SampleRateConverter<2> outputSrc;
	dsp::DoubleRingBuffer<dsp::Frame<4>, 1024> inputBuffer;
	dsp::DoubleRingBuffer<dsp::Frame<2>, 1024> outputBuffer;
	dsp::Frame<4> nativeFrames[64];
	int nativeLen = 0;
	bool outputPrimed = false;

	uint8_t* delayBuffer;
	uint16_t* reverbBuffer;
	beads::Processor* processor;
	beads::Synth* synth;
	BeadsGainControl gainControl;

	// Settings
	int media = beads::Processor::MEDIA_DIGITAL;
	int assignDestination = ASSIGN_FEEDBACK;
	bool seedLocked = true;
	bool gateOutput = false;
	bool autoGain = false;
	bool freeze = false;

	// State
	bool synthMode = true;
	bool frozenBlock = false;
	bool seedButton = false;
	bool seedIgnoreRelease = false;
	int seedHoldBlocks = 0;
	uint32_t blockCounter = 0;
	stmlib::GateFlags previousSeed = stmlib::GATE_FLAG_LOW;
	stmlib::DelayLine<bool, 256> seedDelay;
	dsp::BooleanTrigger freezeTrigger;
	dsp::BooleanTrigger mediaTrigger;
	dsp::BooleanTrigger assignTrigger;
	dsp::BooleanTrigger seedTrigger;
	bool controlsInitialized = false;
	// Bumped whenever the recording buffer changes, to write a frozen buffer to patch storage only once
	std::atomic<uint32_t> bufferVersion{1};
	uint32_t savedBufferVersion = 0;
	float potLp[NUM_PARAMS] = {};
	float cvLp[NUM_INPUTS] = {};

	Beads() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
		configButton(FREEZE_PARAM, "Freeze");
		configButton(SEED_PARAM, "Seed");
		configButton(MEDIA_PARAM, "Quality");
		configButton(ASSIGN_PARAM, "CV assign");
		// Density generates no grains at noon, so default to a setting that makes sound
		configParam(DENSITY_PARAM, 0.0, 1.0, 0.7, "Grain density", "%", 0, 100);
		configParam(TIME_PARAM, 0.0, 1.0, 0.0, "Grain time", "%", 0, 100);
		configParam<BeadsPitchQuantity>(PITCH_PARAM, -1.0, 1.0, 0.0, "Grain pitch", " semitones");
		configParam(SIZE_PARAM, 0.0, 1.0, 0.5, "Grain size", "%", 0, 100);
		configParam(SHAPE_PARAM, 0.0, 1.0, 0.5, "Grain shape", "%", 0, 100);
		configParam(FEEDBACK_PARAM, 0.0, 1.0, 0.0, "Feedback", "%", 0, 100);
		configParam(DRY_WET_PARAM, 0.0, 1.0, 0.5, "Dry/wet", "%", 0, 100);
		configParam(REVERB_PARAM, 0.0, 1.0, 0.0, "Reverb", "%", 0, 100);
		configParam(TIME_CV_PARAM, -1.0, 1.0, 0.0, "Time CV / randomization", "%", 0, 100);
		configParam(SIZE_CV_PARAM, -1.0, 1.0, 0.0, "Size CV / randomization", "%", 0, 100);
		configParam(SHAPE_CV_PARAM, -1.0, 1.0, 0.0, "Shape CV / randomization", "%", 0, 100);
		configParam(PITCH_CV_PARAM, -1.0, 1.0, 0.0, "Pitch CV / randomization", "%", 0, 100);
		configParam(INPUT_GAIN_PARAM, -6.0, 26.0, 0.0, "Input gain", " dB");
		getParamQuantity(INPUT_GAIN_PARAM)->randomizeEnabled = false;
		configParam(FEEDBACK_ASSIGN_PARAM, 0.0, 1.0, 1.0, "Assigned CV to feedback", "%", 0, 100);
		configParam(DRY_WET_ASSIGN_PARAM, 0.0, 1.0, 0.0, "Assigned CV to dry/wet", "%", 0, 100);
		configParam(REVERB_ASSIGN_PARAM, 0.0, 1.0, 0.5, "Assigned CV to reverb", "%", 0, 100);
		configInput(ASSIGN_INPUT, "Assignable CV");
		configInput(TIME_INPUT, "Time");
		configInput(SIZE_INPUT, "Size");
		configInput(SHAPE_INPUT, "Shape");
		configInput(PITCH_INPUT, "Pitch (1V/oct)");
		configInput(DENSITY_INPUT, "Density");
		configInput(IN_L_INPUT, "Left");
		configInput(IN_R_INPUT, "Right");
		configInput(FREEZE_INPUT, "Freeze");
		configInput(SEED_INPUT, "Seed");
		configOutput(OUT_L_OUTPUT, "Left");
		configOutput(OUT_R_OUTPUT, "Right");

		configBypass(IN_L_INPUT, OUT_L_OUTPUT);
		configBypass(IN_R_INPUT, OUT_R_OUTPUT);

		delayBuffer = new uint8_t[BEADS_BUFFER_SIZE]();
		reverbBuffer = new uint16_t[BEADS_REVERB_BUFFER_SIZE]();
		// The firmware expects zero-initialized globals
		processor = new beads::Processor();
		memset((void*) processor, 0, sizeof(*processor));
		synth = new beads::Synth();
		memset((void*) synth, 0, sizeof(*synth));

		uint8_t* buffers[3];
		for (int i = 0; i < 3; i++) {
			buffers[i] = delayBuffer + i * BEADS_BANK_SIZE;
		}
		processor->Init(buffers, reverbBuffer);
		synth->Init(reverbBuffer);
		seedDelay.Init();
		gainControl.init(48000.f, BeadsGainControl::SOFT_CLIPPING_MODE_OFF, true);
	}

	~Beads() {
		delete synth;
		delete processor;
		delete[] reverbBuffer;
		delete[] delayBuffer;
	}

	void onReset() override {
		media = beads::Processor::MEDIA_DIGITAL;
		assignDestination = ASSIGN_FEEDBACK;
		seedLocked = true;
		gateOutput = false;
		autoGain = false;
		freeze = false;
		frozenBlock = false;

		// Clear the recording
		processor->Configure((beads::Processor::Media) media, processor->mono());
		bufferVersion++;
		startGainControl(true);
		flushBuffers();
	}

	void flushBuffers() {
		inputBuffer.clear();
		outputBuffer.clear();
		nativeLen = 0;
		outputPrimed = false;
	}

	float nativeSampleRate() const {
		return synthMode ? 48000.f : processor->sample_rate();
	}

	void startGainControl(bool reset) {
		beads::Processor::Media m = processor->media();
		gainControl.init(
			nativeSampleRate(),
			m == beads::Processor::MEDIA_DIRTY_TAPE ? BeadsGainControl::SOFT_CLIPPING_MODE_ON
			: (m == beads::Processor::MEDIA_CLEAN_TAPE ? BeadsGainControl::SOFT_CLIPPING_MODE_DYNAMIC
			: BeadsGainControl::SOFT_CLIPPING_MODE_OFF),
			reset);
		seedDelay.Init();
	}

	// From MonitorStateChange() in beads.cc
	void monitorStateChange() {
		// Nothing can change under a frozen buffer
		if (frozenBlock)
			return;

		bool left = inputs[IN_L_INPUT].isConnected();
		bool right = inputs[IN_R_INPUT].isConnected();
		if (!left && !right) {
			if (!synthMode) {
				synthMode = true;
				startGainControl(true);
			}
			return;
		}

		bool mono = left != right;
		bool mediaChanged = media != (int) processor->media();
		bool monoChanged = mono != processor->mono();
		if (synthMode || mediaChanged || monoChanged) {
			// Only a change of format clears the recording, so that it survives moving a cable
			if (mediaChanged || monoChanged) {
				processor->Configure((beads::Processor::Media) media, mono);
				bufferVersion++;
			}
			bool reset = monoChanged || synthMode;
			synthMode = false;
			startGainControl(reset);
		}
	}

	float readPot(int paramId, float value) {
		if (!controlsInitialized)
			potLp[paramId] = value;
		ONE_POLE(potLp[paramId], value, 0.05f);
		return potLp[paramId];
	}

	float readCv(int inputId, float scale, float limit, float coefficient) {
		float value = clamp(inputs[inputId].getVoltage() * scale, -limit, limit);
		if (!controlsInitialized)
			cvLp[inputId] = value;
		ONE_POLE(cvLp[inputId], value, coefficient);
		return cvLp[inputId];
	}

	void readRandomizedParameter(beads::RandomizedParameter* p, int paramId, int cvParamId, int inputId) {
		p->base = readPot(paramId, params[paramId].getValue());
		p->attenuverter = readPot(cvParamId, beadsAttenuverter(params[cvParamId].getValue()));
		p->has_cv = inputs[inputId].isConnected();
		p->cv = readCv(inputId, 1.f / 5.f, 1.6f, 0.5f);
	}

	void processBlock(const dsp::Frame<4>* in, dsp::Frame<2>* out, int size) {
		beads::FloatFrame floatIn[BEADS_MAX_BLOCK_SIZE];
		beads::FloatFrame floatOut[BEADS_MAX_BLOCK_SIZE];
		bool freezes[BEADS_MAX_BLOCK_SIZE];
		stmlib::GateFlags seeds[BEADS_MAX_BLOCK_SIZE];

		// Buttons
		if (freezeTrigger.process(params[FREEZE_PARAM].getValue() > 0.f)) {
			freeze ^= true;
		}
		if (mediaTrigger.process(params[MEDIA_PARAM].getValue() > 0.f)) {
			media = (media + 3) % 4;
		}
		if (assignTrigger.process(params[ASSIGN_PARAM].getValue() > 0.f)) {
			assignDestination = (assignDestination == ASSIGN_CUSTOM) ? 0 : ((assignDestination + 1) % 3);
		}

		// Pressing the seed button unlatches automatic seeding, holding it for 4s latches again
		bool seedPressed = params[SEED_PARAM].getValue() > 0.f;
		if (seedTrigger.process(seedPressed)) {
			seedLocked = false;
			seedIgnoreRelease = false;
			seedHoldBlocks = 0;
		}
		if (seedPressed && !seedIgnoreRelease) {
			if (++seedHoldBlocks >= 4 * BEADS_BLOCK_RATE) {
				seedLocked = true;
				seedIgnoreRelease = true;
			}
		}
		seedButton = seedPressed && !seedIgnoreRelease;

		// Gates
		bool freezeBlock = false;
		for (int i = 0; i < size; i++) {
			freezes[i] = freeze || in[i].samples[3] >= 0.5f;
			freezeBlock |= freezes[i];
		}

		bool seedBlock = false;
		stmlib::GateFlags* seedInput = NULL;
		if (inputs[SEED_INPUT].isConnected()) {
			for (int i = 0; i < size; i++) {
				seedDelay.Write(in[i].samples[2] >= 0.5f);
				// Delay the seed input by 2.5ms to let the CVs settle
				bool hasSeed = seedDelay.Read(size_t(size * 10));
				previousSeed = seeds[i] = stmlib::ExtractGateFlags(previousSeed, hasSeed);
				seedBlock |= hasSeed;
			}
			seedInput = seeds;
		}
		else {
			previousSeed = stmlib::GATE_FLAG_LOW;
		}

		// Parameters
		beads::Parameters p;
		readRandomizedParameter(&p.time, TIME_PARAM, TIME_CV_PARAM, TIME_INPUT);
		readRandomizedParameter(&p.size, SIZE_PARAM, SIZE_CV_PARAM, SIZE_INPUT);
		readRandomizedParameter(&p.shape, SHAPE_PARAM, SHAPE_CV_PARAM, SHAPE_INPUT);
		p.pitch.base = readPot(PITCH_PARAM, beadsDetune(params[PITCH_PARAM].getValue()));
		p.pitch.attenuverter = readPot(PITCH_CV_PARAM, beadsAttenuverter(params[PITCH_CV_PARAM].getValue()));
		p.pitch.has_cv = inputs[PITCH_INPUT].isConnected();
		p.pitch.cv = readCv(PITCH_INPUT, 12.f, 60.f, 1.f);
		p.monophonic_mode = false;

		beads::ModulatedParameter d;
		d.base = readPot(DENSITY_PARAM, params[DENSITY_PARAM].getValue());
		d.cv = readCv(DENSITY_INPUT, 1.f / 5.f, 1.f, 1.f);

		float assignCv = readCv(ASSIGN_INPUT, 1.f / 5.f, 1.f, 0.1f);
		float mix[3];
		for (int i = 0; i < 3; i++) {
			float base = readPot(FEEDBACK_PARAM + i, params[FEEDBACK_PARAM + i].getValue()) * 1.005f;
			float amount = (assignDestination == ASSIGN_CUSTOM)
				? params[FEEDBACK_ASSIGN_PARAM + i].getValue()
				: (assignDestination == i ? 1.f : 0.f);
			mix[i] = clamp(base + assignCv * amount, 0.f, 1.f);
		}
		controlsInitialized = true;

		beads::Processor::MixParameters m;
		m.feedback = mix[0];
		m.dry_wet = mix[1];
		m.reverb_amount = mix[2];
		m.output_gate = gateOutput;

		beads::SeedButtonState seedButtonState = seedLocked
			? beads::SEED_BUTTON_STATE_LOCKED
			: (seedButton ? beads::SEED_BUTTON_STATE_ON : beads::SEED_BUTTON_STATE_OFF);

		// Render
		if (synthMode) {
			synth->Render(seedButtonState, seedInput, freezes, (beads::Synth::Resolution) media, d, p, m, floatOut, size);
		}
		else {
			for (int i = 0; i < size; i++) {
				floatIn[i].l = in[i].samples[0];
				floatIn[i].r = in[i].samples[1];
			}
			float manualGain = clamp(2.f * std::pow(10.f, params[INPUT_GAIN_PARAM].getValue() / 20.f), BEADS_MIN_GAIN, BEADS_MAX_GAIN);
			gainControl.process(autoGain, manualGain, floatIn, size);
			processor->Process(seedButtonState, seedInput, freezes, d, p, m, floatIn, floatOut, size);
		}

		for (int i = 0; i < size; i++) {
			out[i].samples[0] = clamp(floatOut[i].l, -1.f, 1.f);
			out[i].samples[1] = clamp(floatOut[i].r, -1.f, 1.f);
		}

		// Lights
		if (!synthMode && !freezeBlock)
			bufferVersion++;
		frozenBlock = freezeBlock;
		blockCounter++;
		uint32_t milliseconds = blockCounter / (BEADS_BLOCK_RATE / 1000);
		float lightTime = 1.f / BEADS_BLOCK_RATE;

		lights[FREEZE_LIGHT].setBrightness(freezeBlock ? 1.f : 0.f);

		if (seedLocked) {
			int triangle = (milliseconds & 2047) >> 2;
			triangle = triangle >= 256 ? 511 - triangle : triangle;
			lights[SEED_LIGHT].setBrightness(((triangle * 3 >> 2) + 64) / 255.f);
		}
		else {
			lights[SEED_LIGHT].setBrightnessSmooth((seedButton || seedBlock) ? 1.f : 0.f, lightTime);
		}

		// The quality light fades out when a frozen buffer is about to be overwritten
		static const float mediaPalette[4][3] = {
			{128, 0, 48},
			{255, 144, 0},
			{0, 255, 192},
			{192, 192, 208},
		};
		float reserve = synthMode ? 1.f : processor->overwrite_reserve();
		lights[MEDIA_LIGHT + 0].setBrightness(mediaPalette[media][0] / 255.f * reserve * reserve);
		lights[MEDIA_LIGHT + 1].setBrightness(mediaPalette[media][1] / 255.f * reserve);
		lights[MEDIA_LIGHT + 2].setBrightness(mediaPalette[media][2] / 255.f * reserve);

		for (int i = 0; i < 3; i++) {
			float brightness = (assignDestination == ASSIGN_CUSTOM)
				? params[FEEDBACK_ASSIGN_PARAM + i].getValue() * 0.5f
				: (assignDestination == i ? 0.5f : 0.f);
			lights[ASSIGN_LIGHTS + i].setBrightness(brightness);
		}

		if (synthMode) {
			lights[VU_LIGHT + 0].setBrightness(0.f);
			lights[VU_LIGHT + 1].setBrightness(0.f);
		}
		else if (gainControl.adjusting()) {
			lights[VU_LIGHT + 0].setBrightness((milliseconds & 128) ? 0.5f : 0.f);
			lights[VU_LIGHT + 1].setBrightness(0.f);
		}
		else {
			float intensity = gainControl.vuMeter();
			bool clipping = intensity > 1.f;
			lights[VU_LIGHT + 0].setBrightness(clipping ? 0.f : 0.75f * intensity * intensity);
			lights[VU_LIGHT + 1].setBrightnessSmooth(clipping ? 1.f : 0.f, lightTime);
		}
	}

	void process(const ProcessArgs& args) override {
		// Get input
		if (!inputBuffer.full()) {
			dsp::Frame<4> inputFrame;
			bool left = inputs[IN_L_INPUT].isConnected();
			bool right = inputs[IN_R_INPUT].isConnected();
			float l = inputs[IN_L_INPUT].getVoltageSum();
			float r = inputs[IN_R_INPUT].getVoltageSum();
			l = std::isfinite(l) ? clamp(l, -BEADS_INPUT_LIMIT, BEADS_INPUT_LIMIT) * BEADS_INPUT_SCALE : 0.f;
			r = std::isfinite(r) ? clamp(r, -BEADS_INPUT_LIMIT, BEADS_INPUT_LIMIT) * BEADS_INPUT_SCALE : 0.f;
			inputFrame.samples[0] = left ? l : r;
			inputFrame.samples[1] = right ? r : l;
			inputFrame.samples[2] = inputs[SEED_INPUT].getVoltage() >= 1.f ? 1.f : 0.f;
			inputFrame.samples[3] = inputs[FREEZE_INPUT].getVoltage() >= 1.f ? 1.f : 0.f;
			inputBuffer.push(inputFrame);
		}

		// Render blocks once enough input has accumulated for one
		float nativeRate = nativeSampleRate();
		int blockSize = (int) nativeRate / BEADS_BLOCK_RATE;
		int blockFrames = (int) std::ceil(blockSize * args.sampleRate / nativeRate);
		if ((int) inputBuffer.size() >= blockFrames) {
			inputSrc.setRates(args.sampleRate, nativeRate);
			int inLen = inputBuffer.size();
			int outLen = LENGTHOF(nativeFrames) - nativeLen;
			inputSrc.process(inputBuffer.startData(), &inLen, nativeFrames + nativeLen, &outLen);
			inputBuffer.startIncr(inLen);
			nativeLen += outLen;

			while (nativeLen >= blockSize) {
				dsp::Frame<2> outputFrames[BEADS_MAX_BLOCK_SIZE];
				processBlock(nativeFrames, outputFrames, blockSize);

				nativeLen -= blockSize;
				std::memmove(nativeFrames, nativeFrames + blockSize, nativeLen * sizeof(nativeFrames[0]));

				outputSrc.setRates(nativeRate, args.sampleRate);
				int blockInLen = blockSize;
				int blockOutLen = outputBuffer.capacity();
				outputSrc.process(outputFrames, &blockInLen, outputBuffer.endData(), &blockOutLen);
				outputBuffer.endIncr(blockOutLen);

				// The native sample rate depends on the quality setting
				monitorStateChange();
				if (nativeSampleRate() != nativeRate) {
					flushBuffers();
					break;
				}
			}
		}

		// Set output
		// Keep a block in hand so that the output doesn't run dry between bursts of blocks
		dsp::Frame<2> outputFrame = {};
		if (!outputPrimed && (int) outputBuffer.size() > blockFrames) {
			outputPrimed = true;
		}
		if (outputPrimed) {
			if (!outputBuffer.empty())
				outputFrame = outputBuffer.shift();
			else
				outputPrimed = false;
		}
		outputs[OUT_L_OUTPUT].setVoltage(BEADS_OUTPUT_SCALE * outputFrame.samples[0]);
		if (gateOutput) {
			// Inverting op-amp on outputs
			outputs[OUT_R_OUTPUT].setVoltage(outputFrame.samples[1] < -0.4f ? 8.f : 0.f);
		}
		else {
			outputs[OUT_R_OUTPUT].setVoltage(BEADS_OUTPUT_SCALE * outputFrame.samples[1]);
		}
	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();

		json_object_set_new(rootJ, "media", json_integer(media));
		json_object_set_new(rootJ, "assignDestination", json_integer(assignDestination));
		json_object_set_new(rootJ, "seedLocked", json_boolean(seedLocked));
		json_object_set_new(rootJ, "gateOutput", json_boolean(gateOutput));
		json_object_set_new(rootJ, "autoGain", json_boolean(autoGain));
		json_object_set_new(rootJ, "freeze", json_boolean(freeze));

		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {
		json_t* mediaJ = json_object_get(rootJ, "media");
		if (mediaJ) {
			media = clamp((int) json_integer_value(mediaJ), 0, 3);
		}

		json_t* assignDestinationJ = json_object_get(rootJ, "assignDestination");
		if (assignDestinationJ) {
			assignDestination = clamp((int) json_integer_value(assignDestinationJ), 0, 3);
		}

		json_t* seedLockedJ = json_object_get(rootJ, "seedLocked");
		if (seedLockedJ) {
			seedLocked = json_boolean_value(seedLockedJ);
		}

		json_t* gateOutputJ = json_object_get(rootJ, "gateOutput");
		if (gateOutputJ) {
			gateOutput = json_boolean_value(gateOutputJ);
		}

		json_t* autoGainJ = json_object_get(rootJ, "autoGain");
		if (autoGainJ) {
			autoGain = json_boolean_value(autoGainJ);
		}

		json_t* freezeJ = json_object_get(rootJ, "freeze");
		if (freezeJ) {
			freeze = json_boolean_value(freezeJ);
		}
	}

	// The frozen buffer is saved with the patch, like the hardware saves it to flash

	struct BufferHeader {
		uint32_t tag;
		int32_t writePtr;
		uint8_t media;
		uint8_t mono;
		uint8_t padding[2];
	};

	void onSave(const SaveEvent& e) override {
		// Nothing to keep
		if (!freeze || synthMode) {
			std::string dir = getPatchStorageDirectory();
			if (!dir.empty()) {
				system::remove(system::join(dir, "buffer.dat"));
			}
			return;
		}
		// A frozen buffer doesn't change, so write it once
		std::string path = system::join(createPatchStorageDirectory(), "buffer.dat");
		// The audio thread keeps running, so note the version before writing
		uint32_t version = bufferVersion;
		if (version == savedBufferVersion && system::exists(path))
			return;
		if (saveBuffer(path))
			savedBufferVersion = version;
	}

	void onAdd(const AddEvent& e) override {
		if (!freeze)
			return;
		// Unfreeze if the recording is missing
		freeze = loadBuffer(system::join(getPatchStorageDirectory(), "buffer.dat"));
	}

	bool saveBuffer(const std::string& path) {
		FILE* file = std::fopen(path.c_str(), "wb");
		if (!file) {
			WARN("Could not write %s", path.c_str());
			return false;
		}
		DEFER({std::fclose(file);});

		BufferHeader header = {};
		header.tag = BEADS_BUFFER_TAG;
		header.writePtr = *processor->mutable_delay_write_ptr();
		header.media = processor->media();
		header.mono = processor->mono();
		return std::fwrite(&header, sizeof(header), 1, file) == 1
			&& std::fwrite(delayBuffer, 1, BEADS_BUFFER_SIZE, file) == BEADS_BUFFER_SIZE;
	}

	bool loadBuffer(const std::string& path) {
		FILE* file = std::fopen(path.c_str(), "rb");
		if (!file)
			return false;
		DEFER({std::fclose(file);});

		BufferHeader header;
		if (std::fread(&header, sizeof(header), 1, file) != 1)
			return false;
		if (header.tag != BEADS_BUFFER_TAG || header.media > 3)
			return false;
		static const int bitsPerSample[4] = {8, 12, 12, 16};
		int maxDelay = beads::kDelayBufferSizeBits / (bitsPerSample[header.media] * 2) * (header.mono ? 2 : 1);
		if (header.writePtr < 0 || header.writePtr >= maxDelay)
			return false;

		// Configure() clears the buffer
		processor->Configure((beads::Processor::Media) header.media, header.mono);
		if (std::fread(delayBuffer, 1, BEADS_BUFFER_SIZE, file) != BEADS_BUFFER_SIZE) {
			processor->Configure((beads::Processor::Media) header.media, header.mono);
			return false;
		}
		*processor->mutable_delay_write_ptr() = header.writePtr;

		media = header.media;
		savedBufferVersion = bufferVersion;
		synthMode = false;
		// Block reconfiguration until the first block has run
		frozenBlock = true;
		startGainControl(true);
		return true;
	}
};


struct BeadsParamSlider : ui::Slider {
	BeadsParamSlider(ParamQuantity* paramQuantity) {
		quantity = paramQuantity;
		box.size.x = 200.f;
	}
};


struct BeadsWidget : ModuleWidget {
	BeadsWidget(Beads* module) {
		setModule(module);
		setPanel(Svg::load(asset::plugin(pluginInstance, "res/Beads.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<CKD6>(mm2px(Vec(8.726, 17.749)), module, Beads::FREEZE_PARAM));
		addParam(createParamCentered<CKD6>(mm2px(Vec(62.076, 17.749)), module, Beads::SEED_PARAM));
		addParam(createParamCentered<Rogan3PSWhite>(mm2px(Vec(35.395, 24.099)), module, Beads::DENSITY_PARAM));
		addParam(createParamCentered<Rogan3PSWhite>(mm2px(Vec(11.575, 38.399)), module, Beads::TIME_PARAM));
		addParam(createParamCentered<Rogan3PSWhite>(mm2px(Vec(59.225, 38.399)), module, Beads::PITCH_PARAM));
		addParam(createParamCentered<TL1105>(mm2px(Vec(35.395, 45.049)), module, Beads::MEDIA_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(6.775, 60.599)), module, Beads::FEEDBACK_PARAM));
		addParam(createParamCentered<Rogan2PSWhite>(mm2px(Vec(24.275, 60.599)), module, Beads::SIZE_PARAM));
		addParam(createParamCentered<Rogan2PSWhite>(mm2px(Vec(46.525, 60.599)), module, Beads::SHAPE_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(64.025, 60.599)), module, Beads::DRY_WET_PARAM));
		addParam(createParamCentered<TL1105>(mm2px(Vec(6.776, 76.799)), module, Beads::ASSIGN_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(64.025, 76.799)), module, Beads::REVERB_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(18.225, 80.599)), module, Beads::TIME_CV_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(29.675, 80.599)), module, Beads::SIZE_CV_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(41.126, 80.599)), module, Beads::SHAPE_CV_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(52.576, 80.599)), module, Beads::PITCH_CV_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.776, 96.499)), module, Beads::ASSIGN_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(18.226, 96.499)), module, Beads::TIME_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(29.676, 96.499)), module, Beads::SIZE_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(41.125, 96.499)), module, Beads::SHAPE_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(52.575, 96.499)), module, Beads::PITCH_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(64.026, 96.499)), module, Beads::DENSITY_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.776, 111.099)), module, Beads::IN_L_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(18.226, 111.099)), module, Beads::IN_R_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(29.676, 111.099)), module, Beads::FREEZE_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(41.125, 111.099)), module, Beads::SEED_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(52.575, 111.099)), module, Beads::OUT_L_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(64.026, 111.099)), module, Beads::OUT_R_OUTPUT));

		addChild(createLightCentered<CKD6Light<WhiteLight>>(mm2px(Vec(8.726, 17.749)), module, Beads::FREEZE_LIGHT));
		addChild(createLightCentered<CKD6Light<WhiteLight>>(mm2px(Vec(62.076, 17.749)), module, Beads::SEED_LIGHT));
		addChild(createLightCentered<MediumLight<RedGreenBlueLight>>(mm2px(Vec(29.026, 45.049)), module, Beads::MEDIA_LIGHT));
		addChild(createLightCentered<MediumLight<WhiteLight>>(mm2px(Vec(6.776, 69.799)), module, Beads::ASSIGN_LIGHTS + 0));
		addChild(createLightCentered<MediumLight<WhiteLight>>(mm2px(Vec(64.025, 69.799)), module, Beads::ASSIGN_LIGHTS + 1));
		addChild(createLightCentered<MediumLight<WhiteLight>>(mm2px(Vec(64.025, 85.999)), module, Beads::ASSIGN_LIGHTS + 2));
		addChild(createLightCentered<MediumLight<GreenRedLight>>(mm2px(Vec(12.526, 104.749)), module, Beads::VU_LIGHT));
	}

	void appendContextMenu(Menu* menu) override {
		Beads* module = dynamic_cast<Beads*>(this->module);

		menu->addChild(new MenuSeparator);

		menu->addChild(createIndexPtrSubmenuItem("Quality", {
			"Scorched cassette (24kHz 8-bit µ-law)",
			"Sunny tape (24kHz 12-bit)",
			"Cold digital (32kHz 12-bit)",
			"Brand new digital (48kHz 16-bit)",
		}, &module->media));

		menu->addChild(createSubmenuItem("Input gain", "",
			[=](Menu* menu) {
				menu->addChild(createBoolPtrMenuItem("Automatic", "", &module->autoGain));
				menu->addChild(new BeadsParamSlider(module->paramQuantities[Beads::INPUT_GAIN_PARAM]));
			}
		));

		menu->addChild(createIndexPtrSubmenuItem("CV assign", {
			"Feedback",
			"Dry/wet",
			"Reverb",
			"Custom amounts",
		}, &module->assignDestination));

		menu->addChild(createSubmenuItem("CV assign custom amounts", "",
			[=](Menu* menu) {
				for (int i = 0; i < 3; i++) {
					menu->addChild(new BeadsParamSlider(module->paramQuantities[Beads::FEEDBACK_ASSIGN_PARAM + i]));
				}
			}
		));

		menu->addChild(createBoolPtrMenuItem("Latch seed (generate grains automatically)", "", &module->seedLocked));
		menu->addChild(createBoolPtrMenuItem("Grain gate on R output", "", &module->gateOutput));
	}
};


Model* modelBeads = createModel<Beads, BeadsWidget>("Beads");
