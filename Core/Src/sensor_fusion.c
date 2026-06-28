#include "sensor_fusion.h"

#define EMA_ALPHA               0.3f   // Smoothing factor (0.0 to 1.0). Lower = smoother, slower reaction.
#define YELLOW_WARN_DIST_CM     50.0f  // Distance threshold to enter YELLOW
#define YELLOW_CLEAR_DIST_CM    55.0f  // Distance threshold to drop back to GREEN (Hysteresis deadband)
#define RED_CRIT_DIST_CM        30.0f  // Distance threshold to enter RED
#define RED_CLEAR_DIST_CM       35.0f  // Distance threshold to drop back to YELLOW (Hysteresis deadband)

#define HIGH_RAIN_ENERGY        50     // Threshold for active, heavy rain
#define FAST_RISE_RATE          2.0f   // cm per minute
#define PRESSURE_DROP_WARN      -0.5f  // hPa drop between cycles
#define MAX_WATER_DELTA_CM 		50.0f  // Water cannot fall or rise 50cm between sleep cycles
#define WATER_RATE_UNAVAILABLE	-999.0f // Sentinel value for blinded/timeout sensor states

typedef struct {
	bool is_first_run;
	float prev_ema_water_cm;
	float prev_pressure_hpa;
	uint32_t prev_sleep_duration_s;
	RiskLevel_t current_state;
} SensorFusion_Context_t;

static SensorFusion_Context_t ctx = {
	.is_first_run = true,
	.prev_ema_water_cm = 0.0f,
	.prev_pressure_hpa = 0.0f,
	.prev_sleep_duration_s = SLEEP_NORMAL,
	.current_state = STATUS_GREEN
};

void SensorFusion_Init(void) {
	ctx.is_first_run = true;
	ctx.current_state = STATUS_GREEN;
}

FusedData_t SensorFusion_Process(RawSensorData_t raw) {
    FusedData_t result = {0};

    bool data_is_reliable = true;

    // Handle Invalid Hardware Data
    if (raw.water_distance_cm < 0.0f) {
        // Sensor timeout/error. Use the last known good value to prevent dividing by zero
        // or triggering false alarms.
        raw.water_distance_cm = ctx.prev_ema_water_cm;
        data_is_reliable = false;
    }

    // First Run Initialization
    if (ctx.is_first_run) {
        ctx.prev_ema_water_cm = raw.water_distance_cm;
        ctx.prev_pressure_hpa = (raw.atm.is_valid) ? raw.atm.pressure_hPa : 1013.25f;	// 1 atm
        ctx.prev_sleep_duration_s = SLEEP_NORMAL;
        ctx.is_first_run = false;
        data_is_reliable = false;
    } else {
        // Slew rate limiter (blind spot protection)
        float delta = raw.water_distance_cm - ctx.prev_ema_water_cm;

        // If the reading jumps massively AND we are near the blind spot
        if (delta > MAX_WATER_DELTA_CM && ctx.prev_ema_water_cm < 35.0f) {
            raw.water_distance_cm = 25.0f; // Clamp to minimum physical distance
            ctx.current_state = STATUS_RED;    // Force critical threat
            data_is_reliable = false;      // Sensor is blinded, rate is garbage
        }
        // General noise rejection: Impossible jumps in the open air
        else if (delta > MAX_WATER_DELTA_CM || delta < -MAX_WATER_DELTA_CM) {
            raw.water_distance_cm = ctx.prev_ema_water_cm;
            data_is_reliable = false;      // Glitch detected, rate is garbage
        }
    }

    // High-Level Smoothing (Exponential Moving Average)
    // This smooths out physical water ripples that the median filter couldn't catch.
    result.smoothed_water_cm = (raw.water_distance_cm * EMA_ALPHA) +
                               (ctx.prev_ema_water_cm * (1.0f - EMA_ALPHA));

    if (data_is_reliable) {
		float time_elapsed_mins = (float)ctx.prev_sleep_duration_s / 60.0f;
		result.water_rise_rate_cm_min = (ctx.prev_ema_water_cm - result.smoothed_water_cm) / time_elapsed_mins;
	} else {
		result.water_rise_rate_cm_min = WATER_RATE_UNAVAILABLE;
	}

    if (raw.atm.is_valid) {
        // Negative means pressure is dropping (rain coming)
        result.pressure_trend_hpa = raw.atm.pressure_hPa - ctx.prev_pressure_hpa;
    } else {
        result.pressure_trend_hpa = 0.0f;
    }

    // State machine & hysteresis evaluation
    bool is_raining = (raw.rain_energy > HIGH_RAIN_ENERGY);
    bool pressure_dropping = (result.pressure_trend_hpa <= PRESSURE_DROP_WARN);
    bool rising_fast = (result.water_rise_rate_cm_min >= FAST_RISE_RATE);

    switch (ctx.current_state) {
        case STATUS_GREEN:
            // Check for escalation
            if (result.smoothed_water_cm <= RED_CRIT_DIST_CM || rising_fast) {
                ctx.current_state = STATUS_RED;
            }
            else if (result.smoothed_water_cm <= YELLOW_WARN_DIST_CM || pressure_dropping || is_raining) {
                ctx.current_state = STATUS_YELLOW;
            }
            break;

        case STATUS_YELLOW:
            // Check for escalation to RED
            if (result.smoothed_water_cm <= RED_CRIT_DIST_CM || rising_fast) {
                ctx.current_state = STATUS_RED;
            }
            // Check for de-escalation to GREEN (Must clear the deadband, and not be actively storming)
            else if (result.smoothed_water_cm >= YELLOW_CLEAR_DIST_CM && !pressure_dropping && !is_raining) {
                ctx.current_state = STATUS_GREEN;
            }
            break;

        case STATUS_RED:
            // Check for de-escalation to YELLOW (Water must recede past the RED deadband)
            if (result.smoothed_water_cm >= RED_CLEAR_DIST_CM && !rising_fast) {
                ctx.current_state = STATUS_YELLOW;
            }
            break;
    }

    // Map State to outputs
    result.risk_level = ctx.current_state;

    if (ctx.current_state == STATUS_RED) result.next_sleep_s = SLEEP_CRITICAL;
    else if (ctx.current_state == STATUS_YELLOW) result.next_sleep_s = SLEEP_WARNING;
    else result.next_sleep_s = SLEEP_NORMAL;

    // Update History for Next Cycle
    ctx.prev_ema_water_cm = result.smoothed_water_cm;
    if (raw.atm.is_valid) {
        ctx.prev_pressure_hpa = raw.atm.pressure_hPa;
    }
    ctx.prev_sleep_duration_s = result.next_sleep_s;

    return result;
}

