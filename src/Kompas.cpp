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
		LAT_LED,
		ALT_LED,
		LON_LED,
		LAT_CHANGED_LED,
		ALT_CHANGED_LED,
		LON_CHANGED_LED,
		RESET_LED,
		CLOCK_LED,
		NUM_LIGHTS
	};

	enum {
		LATITUDE,
		ALTITUDE,
		LONGITUDE,
		NUM_MODES
	};

	bool generate[NUM_MODES] = {true, true, true};

	// for processing rising edges (clock / reset)
	dsp::SchmittTrigger clockTrigger;
	dsp::SchmittTrigger resetTrigger;

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
		configParam(LATITUDE_PARAM, 0.f, 1023.f, 0.f, "Latitude", "", 0.f, 1./1023.f);
		configParam(ALTITUDE_PARAM, 0.f, 1023.f, 0.f, "Altitude", "", 0.f, 1./1023.f);
		configParam(LONGITUDE_PARAM, 0.f, 1023.f, 0.f, "Longitude", "", 0.f, 1./1023.f);
	}

	void process(const ProcessArgs& args) override {

		for (int mode = 0; mode < NUM_MODES; ++mode) {
			if (generate[mode]) {
				DEBUG((string::f("regenerating for mode: %d", mode)).c_str());
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

		const bool clockTick = clockTrigger.process(rescale(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f, 0.f, 1.f));

		if (clockTick) {

			for (int mode = 0; mode < NUM_MODES; ++mode) {
				step[mode] = (step[mode] + 1) % length[mode];
			}

			for (int mode = 0; mode < NUM_MODES; ++mode) {
				pos[mode] = step[mode];

				const float modeCV = inputs[CV_LAT_INPUT + mode].getVoltage();
				int rescaledCV = rescale(clamp(modeCV, 0.f, 10.f), 0.f, 10.f, 0.f, 1023.f);

				stepValue[mode] = (rescaledCV > 1011) ? 1 : patternArray[mode][pos[mode]];
				prob[mode] = params[mode].getValue() + rescaledCV;

				DEBUG((string::f("mode %d: %d %d %d %d", mode, pos[mode], step[mode], prob[mode], stepValue[mode], patternArray[mode][pos[mode]])).c_str());
			}

			noStepsLongitude = rescale(prob[LONGITUDE], 0, 1021, 1, 8);
			DEBUG(string::f("noSteps: %d", noStepsLongitude).c_str());

		}

		for (int mode = 0; mode < NUM_MODES; ++mode) {
			outputs[mode].setVoltage(stepValue[mode] * clockTrigger.isHigh() * 10.f);

			// output the pattern for the current LED
			lights[mode].setSmoothBrightness(stepValue[mode] * clockTrigger.isHigh(), args.sampleTime);

			// also indicate if the pattern has recently changed
			lights[LAT_CHANGED_LED + mode].setSmoothBrightness(coordinateTrigger[mode].process(args.sampleTime), args.sampleTime);
		}

		lights[CLOCK_LED].setBrightness(rescale(inputs[CLOCK_INPUT].getVoltage(), 0.1f, 2.f, 0.f, 1.f));
		lights[RESET_LED].setSmoothBrightness(resetLEDPulse.process(args.sampleTime), args.sampleTime);

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
		std::string output = string::f("regenerating new pattern for mode %d: ", mode);
		for (int i = 0; i < length[mode]; i++) {
			output += string::f("%d ", patternArray[mode][i]);
		}
		DEBUG(output.c_str());
	}
};


struct KompasWidget : ModuleWidget {
	KompasWidget(Kompas* module) {
		setModule(module);
		setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/Kompas.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(12.908, 13.607)), module, Kompas::LATITUDE_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(12.908, 34.774)), module, Kompas::ALTITUDE_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(12.908, 54.996)), module, Kompas::LONGITUDE_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(7.049, 87.892)), module, Kompas::CV_LAT_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(7.049, 99.567)), module, Kompas::CV_ALT_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(7.049, 111.243)), module, Kompas::CV_LON_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(7.049, 76.216)), module, Kompas::RESET_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(18.944, 76.216)), module, Kompas::CLOCK_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(18.944, 87.842)), module, Kompas::LAT_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(18.944, 99.467)), module, Kompas::ALT_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(18.944, 111.093)), module, Kompas::LON_OUTPUT));

		// pattern out LEDs
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(20.318, 12.495)), module, Kompas::LAT_LED));
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(20.318, 32.273)), module, Kompas::ALT_LED));
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(20.318, 52.385)), module, Kompas::LON_LED));
		// has pattern updated LED
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(5.546, 19.559)), module, Kompas::LAT_CHANGED_LED));
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(5.546, 39.556)), module, Kompas::ALT_CHANGED_LED));
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(5.546, 59.467)), module, Kompas::LON_CHANGED_LED));

		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(5.546, 67.486)), module, Kompas::RESET_LED));
		addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(20.318, 67.486)), module, Kompas::CLOCK_LED));
	}
};


Model* modelKompas = createModel<Kompas, KompasWidget>("Kompas");