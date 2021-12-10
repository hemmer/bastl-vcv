#include "plugin.hpp"


struct Kompas : Module {
	enum ParamIds {
		LATITUDE_PARAM,
		ALTITUDE_PARAM,
		LONGITUDE_PARAM,
		NUM_PARAMS
	};
	enum InputIds {
		CV_LAT_INPUT,
		CV_ALT_INPUT,
		CV_LON_INPUT,
		RESET_INPUT,
		CLOCK_INPUT,
		NUM_INPUTS
	};
	enum OutputIds {
		LAT_OUTPUT,
		ALT_OUTPUT,
		LON_OUTPUT,
		NUM_OUTPUTS
	};
	enum LightIds {
		LAT_LIGHT,
		ALT_LIGHT,
		LON_LIGHT,
		LAT_CHANGED_LIGHT,
		ALT_CHANGED_LIGHT,
		LON_CHANGED_LIGHT,
		RESET_LIGHT,
		CLOCK_LIGHT,
		NUM_LIGHTS
	};

	enum {
		LATITUDE,
		ALTITUDE,
		LONGITUDE,
		NUM_MODES
	};

	enum Firmware {
		DEFAULT,
		PER_CHANNEL_RESETS
	};
	Firmware firmware = DEFAULT;


	bool generate[NUM_MODES] = {true, true, true};

	// for processing rising edges (clock / reset)
	dsp::SchmittTrigger clockTrigger;
	dsp::SchmittTrigger resetTrigger;
	dsp::SchmittTrigger channelResetTrigger[NUM_MODES];

	dsp::PulseGenerator coordinateTrigger[NUM_MODES];
	dsp::PulseGenerator resetLEDPulse;

	bool patternArray[NUM_MODES][32] = {};
	int randomArray[NUM_MODES][32] = {};
	const int length[NUM_MODES] = {32, 32, 32};
	int pos[NUM_MODES] = {0, 0, 0};

	int prob[NUM_MODES] = {0, 0, 0};
	int prevProb[NUM_MODES] = {0, 0, 0};

	// Euclidean parameters (Longitude)
	int noStepsLongitude;

	int step[NUM_MODES] = {};
	int stepValue[NUM_MODES];

	Kompas() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
		configParam(LATITUDE_PARAM, 0.f, 1023.f, 0.f, "Latitude", "", 0.f, 1. / 1023.f);
		configParam(ALTITUDE_PARAM, 0.f, 1023.f, 0.f, "Altitude", "", 0.f, 1. / 1023.f);
		configParam(LONGITUDE_PARAM, 0.f, 1023.f, 0.f, "Longitude", "", 0.f, 1. / 1023.f);

		configInput(CV_LAT_INPUT, "Latitude CV");
		configInput(CV_ALT_INPUT, "Altitude CV");
		configInput(CV_LON_INPUT, "Longitude CV");
		configInput(RESET_INPUT, "Reset");
		configInput(CLOCK_INPUT, "Clock");

		configOutput(LAT_OUTPUT, "Latitude");
		configOutput(ALT_OUTPUT, "Altitude");
		configOutput(LON_OUTPUT, "Longitude");

		configLight(LAT_LIGHT, "Latitude trigger");
		configLight(ALT_LIGHT, "Altitude trigger");
		configLight(LON_LIGHT, "Longitude trigger");
		configLight(LAT_CHANGED_LIGHT, "Latitude changed");
		configLight(ALT_CHANGED_LIGHT, "Altitude changed");
		configLight(LON_CHANGED_LIGHT, "Longitude changed");
		configLight(RESET_LIGHT, "Reset");
		configLight(CLOCK_LIGHT, "Clock");
	}

	void process(const ProcessArgs& args) override {

		for (int mode = 0; mode < NUM_MODES; ++mode) {
			if (generate[mode]) {
				
				// regenerate the random array
				for (int i = 0; i < length[mode]; i++) {
					randomArray[mode][i] = random::u32() % 1011;
				}
				// trigger for LED
				coordinateTrigger[mode].trigger(0.01);
				// unset re-generate flag
				generate[mode] = false;
			}
		}

		if (resetTrigger.process(rescale(inputs[RESET_INPUT].getVoltage(), 0.1f, 2.f, 0.f, 1.f))) {
			for (int mode = 0; mode < NUM_MODES; ++mode) {
				step[mode] = length[mode] - 1;
			}
			resetLEDPulse.trigger(0.01f);
		}
		// if enabled in the context menu, CV ins can instead act as per-channel resets
		if (firmware == PER_CHANNEL_RESETS) {
			for (int mode = 0; mode < NUM_MODES; ++mode) {
				if (channelResetTrigger[mode].process(rescale(inputs[CV_LAT_INPUT + mode].getVoltage(), 0.1f, 2.f, 0.f, 1.f))) {
					step[mode] = length[mode] - 1;
				}
			}
		}

		// TODO: grace period after reset
		const bool clockTick = clockTrigger.process(rescale(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f, 0.f, 1.f));

		if (clockTick) {

			for (int mode = 0; mode < NUM_MODES; ++mode) {
				step[mode] = (step[mode] + 1) % length[mode];
				pos[mode] = step[mode];

				const float modeCV = inputs[CV_LAT_INPUT + mode].getVoltage();
				// CV only contributes if we're not in the mode where CV is used to provide per-channel resets
				const int rescaledCV = (firmware == DEFAULT) ? rescale(clamp(modeCV, 0.f, 5.f), 0.f, 10.f, 0.f, 1023.f) : 0;

				stepValue[mode] = (rescaledCV > 1011) ? 1 : patternArray[mode][pos[mode]];
				prob[mode] = params[mode].getValue() + rescaledCV;
			}

			noStepsLongitude = rescale(prob[LONGITUDE], 0, 1021, 1, 8);
		}

		for (int mode = 0; mode < NUM_MODES; ++mode) {
			outputs[mode].setVoltage(stepValue[mode] * clockTrigger.isHigh() * 10.f);

			// output the pattern for the current LED
			lights[mode].setSmoothBrightness(stepValue[mode] * clockTrigger.isHigh(), args.sampleTime);

			// also indicate if the pattern has recently changed
			lights[LAT_CHANGED_LIGHT + mode].setSmoothBrightness(coordinateTrigger[mode].process(args.sampleTime), args.sampleTime);
		}

		lights[CLOCK_LIGHT].setBrightness(rescale(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f, 0.f, 1.f));
		lights[RESET_LIGHT].setSmoothBrightness(resetLEDPulse.process(args.sampleTime), args.sampleTime);

		processLongitude();
		processLatitude();
		processAltitude();
	}

	void processLongitude() {
		const int mode = LONGITUDE;

		// LONGITUDE randomarray over euclidean
		if (prob[mode] > (prevProb[mode] + 2) || prob[mode] < (prevProb[mode] - 2)) {

			prevProb[mode] = prob[mode];

			if (prob[mode] > 2) {
				if (prob[mode] < 1011) {
					generate[mode] = true;

					// in non-trivial patterns always start with beat on first
					patternArray[mode][0] = 1;

					// first half of the pattern
					for (int i = 1; i < (length[mode] / 2); i++) {
						if (((i * noStepsLongitude) % (length[mode] / 2)) < noStepsLongitude) {
							patternArray[mode][i] = 1;
							if ((randomArray[mode][i] < prob[mode]) & patternArray[mode][i]) {
								patternArray[mode][i] = 1;
							}
							else {
								patternArray[mode][i] = 0;
							}
						}
						else {
							patternArray[mode][i] = 0;
						}
					}
					// second half
					for (int i = (length[mode] / 2); i < length[mode]; i++) {
						if (((i * noStepsLongitude) % (length[mode] / 2)) < noStepsLongitude) {
							patternArray[mode][i] = 1;
							if ((randomArray[mode][i] < prob[mode]) & patternArray[mode][i]) {
								patternArray[mode][i] = 1;
							}
							else {
								patternArray[mode][i] = 0;
							}
						}
						else {
							patternArray[mode][i] = 0;
						}
					}

				}

			}
			// dial fully left: nothing
			else {
				for (int i = 0; i < length[mode]; i++) {
					patternArray[mode][i] = 0;
				}
			}

			// dial fully right: everything
			if (params[LONGITUDE_PARAM].getValue() > 1011) {
				for (int i = 0; i < length[mode]; i++) {
					patternArray[mode][i] = 1;
				}
			}

			debugPattern(mode);
		}
	}

	void processLatitude() {
		const int mode = LATITUDE;
		if (prob[mode] > (prevProb[mode] + 2) || prob[mode] < (prevProb[mode] - 2)) {
			prevProb[mode] = prob[mode];

			if (prob[mode] > 2) {
				if (prob[mode] < 1011) {
					generate[mode] = true;

					for (int i = 0; i < length[mode]; i++) {
						if (randomArray[mode][i] < prob[mode]) {
							patternArray[mode][i] = 1;
						}
						else {
							patternArray[mode][i] = 0;
						}
					}

					//makes sure at least one step will be filled
					int rnd2 = random::u32() % length[mode];
					patternArray[mode][rnd2] = 1;
				}
			}
			else {
				for (int i = 0; i < length[mode]; i++) {
					patternArray[mode][i] = 0;
				}
				prevProb[mode] = 0;
			}

			if (params[LATITUDE_PARAM].getValue() > 1011) {
				for (int i = 0; i < length[mode]; i++) {
					patternArray[mode][i] = 1;
				}
			}

			debugPattern(mode);
		}
	}

	void processAltitude() {
		const int mode = ALTITUDE;

		// ALTITUDE if Long == Lat - it makes a pattern based on matching steps
		if (prob[mode] > (prevProb[mode] + 2) || prob[mode] < (prevProb[mode] - 2)) {
			prevProb[mode] = prob[mode];

			if (prob[mode] > 2) {
				if (prob[mode] < 1011) {
					generate[mode] = true;

					// wherever they match, toss a coin to decide if a step is active
					for (int i = 0; i < length[mode]; i++) {
						if (patternArray[LATITUDE][i] == patternArray[LONGITUDE][i]) {
							if (randomArray[mode][i] < prob[mode]) {
								patternArray[mode][i] = 1;
							}
							else {
								patternArray[mode][i] = 0;
							}
						}
						else
							patternArray[mode][i] = 0;
					}

					// also add (only) the first step where they match
					for (int i = 0; i < length[mode]; i++) {
						if (patternArray[LATITUDE][i] == patternArray[LONGITUDE][i]) {
							patternArray[mode][i] = 1;
							i = length[mode];
						}
					}
				}
			}
			else {
				for (int i = 0; i < length[mode]; i++) {
					patternArray[mode][i] = 0;
				}
			}

			if (params[ALTITUDE_PARAM].getValue() > 1011) {
				for (int i = 0; i < length[mode]; i++) {
					patternArray[mode][i] = 1;
				}
			}

			debugPattern(mode);
		}
	}

	void debugPattern(int mode) {
#ifdef DEBUG_MODE
		std::string output = string::f("regenerating new pattern for mode %d: ", mode);
		for (int i = 0; i < length[mode]; i++) {
			output += string::f("%d ", patternArray[mode][i]);
		}
		DEBUG(output.c_str());
#endif
	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_object_set_new(rootJ, "firmwareMode", json_integer(firmware));
		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {
		json_t* rangeJ = json_object_get(rootJ, "firmwareMode");
		if (rangeJ) {
			firmware = (Firmware) json_integer_value(rangeJ);
		}
	}
};


struct KompasWidget : ModuleWidget {
	KompasWidget(Kompas* module) {
		setModule(module);
		setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/Kompas.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(12.607, 16.749)), module, Kompas::LATITUDE_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(12.613, 36.429)), module, Kompas::ALTITUDE_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(12.604, 56.102)), module, Kompas::LONGITUDE_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.899, 75.961)), module, Kompas::RESET_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(18.328, 75.961)), module, Kompas::CLOCK_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.899, 87.372)), module, Kompas::CV_LAT_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.899, 98.8)), module, Kompas::CV_ALT_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.899, 110.229)), module, Kompas::CV_LON_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(18.328, 87.372)), module, Kompas::LAT_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(18.328, 98.8)), module, Kompas::ALT_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(18.328, 110.229)), module, Kompas::LON_OUTPUT));

		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(19.933, 13.266)), module, Kompas::LAT_LIGHT));
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(19.933, 32.949)), module, Kompas::ALT_LIGHT));
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(19.933, 52.613)), module, Kompas::LON_LIGHT));
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(5.33, 20.25)), module, Kompas::LAT_CHANGED_LIGHT));
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(5.344, 39.933)), module, Kompas::ALT_CHANGED_LIGHT));
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(5.344, 59.597)), module, Kompas::LON_CHANGED_LIGHT));
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(5.326, 67.552)), module, Kompas::RESET_LIGHT));
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(19.933, 67.552)), module, Kompas::CLOCK_LIGHT));

	}

	void appendContextMenu(Menu* menu) override {
		Kompas* module = dynamic_cast<Kompas*>(this->module);
		assert(module);

		menu->addChild(new MenuSeparator());

		menu->addChild(createIndexPtrSubmenuItem("Mode", {"Per-channel CV control", "Per-channel reset"}, &module->firmware));
	}
};


Model* modelKompas = createModel<Kompas, KompasWidget>("Kompas");