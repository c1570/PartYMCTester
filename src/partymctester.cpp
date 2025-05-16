// PartYMCTester - a flashy microcomputer/microcontroller parts tester
// https://github.com/c1570/PartYMCTester
// GNU Affero General Public License v3

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/vreg.h"
#include "pico-ws2812/ws2812.h"
#include "samplepin.pio.h"
#ifdef PICO_RP2350
#include "hardware/clocks.h"
#endif

const uint32_t MAX_BRIGHTNESS = 100;
const uint32_t FREQ_LED_COL = ((MAX_BRIGHTNESS >> 5) << 16) | ((MAX_BRIGHTNESS >> 5) << 8) | (MAX_BRIGHTNESS >> 1); // blue-ish
const uint32_t SAMPLE_LED_COL_ONE = ((MAX_BRIGHTNESS >> 3) << 16) | ((MAX_BRIGHTNESS >> 4) << 8); // orange
const uint32_t SAMPLE_LED_COL_ZERO = MAX_BRIGHTNESS >> 6; // dark blue

const uint32_t PULSE_PIN_ADC = 26;
const uint32_t PULSE_PIN = 21; // hardcoded in PIO, has to be a PWM B pin
const uint32_t COMPARE_PIN = 19; // has to be a PWM B pin
const uint32_t STROBE_POSEDGE_PIN = 16; // hardcoded in PIO
const uint32_t STROBE_NEGEDGE_PIN = 17; // hardcoded in PIO
const uint32_t START_POSEDGE_PIN = 18;
const uint32_t START_NEGEDGE_PIN = 20;

const uint32_t SAMPLE_DELAY = 10; // delay (in clks) after rising edge on strobe pin until sampling

const uint32_t WSLEDS_PIN = 2;

const uint32_t WSLEDS_WIDTH = 8;
const uint32_t WSLEDS_HEIGHT = 8;
const uint32_t WSLEDS_COUNT = WSLEDS_WIDTH * WSLEDS_HEIGHT;

volatile uint32_t pulse_count = 0;
volatile uint32_t compare_count = 0;

volatile uint64_t sampled_bits = 0;
volatile uint sampled_bits_remaining = 64;

const PIO pio = pio0;
uint counter1_slice;
uint counter2_slice;
uint samplepin_pio_sm;

static inline void __attribute__((always_inline)) get_counter_with_overflow(const uint slice, volatile uint32_t *count) {
  uint16_t current = pwm_get_counter(slice);
  if(current < (*count & 0xffff)) { // uint16 PWM counter just wrapped
    *count = (((*count >> 16) + 1) << 16) | current;
  } else {
    *count = ((*count >> 16) << 16) | current;
  }
}

void core1_loop() {
  while(1) {
    get_counter_with_overflow(counter1_slice, &pulse_count);
    get_counter_with_overflow(counter2_slice, &compare_count);
    while((!pio_sm_is_rx_fifo_empty(pio, samplepin_pio_sm)) && sampled_bits_remaining) {
      //printf("got pulse on strobe pin\n");
      uint32_t pio_bits = pio_sm_get(pio, samplepin_pio_sm);
      sampled_bits = (sampled_bits << 1) | (pio_bits & 1);
      sampled_bits_remaining--;
    }
    //printf("pulse_count: %lu , counter1: %u , slice: %d, pio_pc: %d, remaining: %d\n", pulse_count, counter1, counter1_slice, pio_sm_get_pc(pio, samplepin_pio_sm), sampled_bits_remaining); busy_wait_us(300000);
    __compiler_memory_barrier();
  }
}

// Initialise pulse counter
uint pulse_counter_init(uint pin)
{
    assert(pwm_gpio_to_channel(pin) == PWM_CHAN_B);
    uint counter_slice = pwm_gpio_to_slice_num(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_config_set_clkdiv(&cfg, 1);
    pwm_init(counter_slice, &cfg, false);
    pwm_set_counter(counter_slice, 0);
    return counter_slice;
}

uint32_t color_intensity(uint32_t rgb, float intensity) {
  uint32_t r = ((rgb >> 16) & 0xff) * intensity;
  uint32_t g =  ((rgb >> 8) & 0xff) * intensity;
  uint32_t b =  ((rgb >> 0) & 0xff) * intensity;
  return (r << 16) | (g << 8) | (b << 0);
}

int main() {
  // init LED
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, 1);

  // init pins
  gpio_init(PULSE_PIN);
  gpio_set_dir(PULSE_PIN, GPIO_IN);
  gpio_init(COMPARE_PIN);
  gpio_set_dir(COMPARE_PIN, GPIO_IN);
  gpio_pull_down(COMPARE_PIN);
  gpio_init(STROBE_POSEDGE_PIN);
  gpio_set_dir(STROBE_POSEDGE_PIN, GPIO_IN);
  gpio_pull_down(STROBE_POSEDGE_PIN);
  gpio_init(STROBE_NEGEDGE_PIN);
  gpio_set_dir(STROBE_NEGEDGE_PIN, GPIO_IN);
  gpio_pull_up(STROBE_NEGEDGE_PIN);
  gpio_init(START_POSEDGE_PIN);
  gpio_set_dir(START_POSEDGE_PIN, GPIO_IN);
  gpio_pull_up(START_POSEDGE_PIN);
  gpio_init(START_NEGEDGE_PIN);
  gpio_set_dir(START_NEGEDGE_PIN, GPIO_IN);
  gpio_pull_down(START_NEGEDGE_PIN);

  // set system clock
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  busy_wait_us(10000);
  set_sys_clock_khz(400000, true);
  busy_wait_us(10000);

  gpio_put(LED_PIN, 0);

  // init WS2812
  WS2812 neopixel = WS2812(WSLEDS_COUNT, WSLEDS_PIN);
  neopixel.begin();

  uint32_t analog_buckets[WSLEDS_HEIGHT];
  uint32_t bucket_colors[WSLEDS_HEIGHT];
  for(uint j = 0; j < WSLEDS_HEIGHT; j++) {
    // high is orange, low is blue
    float q = ((float) j) / ((float) WSLEDS_HEIGHT - 1);
    uint32_t r8 = +q * 0xe0 + 0x10;
    uint32_t g8 = +q * 0x05 + 0x85;
    uint32_t b8 = -q * 0x90 + 0x38;
    bucket_colors[j] = color_intensity((r8 << 16) | (g8 << 8) | b8, 0.2);
  }

  uint32_t counter_colors[WSLEDS_COUNT];
  for(uint j = 0; j < WSLEDS_COUNT; j++) {
    switch(j) {
      case 6: // 2^5 = ~60Hz (IRQ)
      case 11: // 2^10 = ~1kHz (BA/Badlines)
      case 20: // 2^19 = ~1MHz (Phi)
      case 21: // 2^20 = ~2MHz (CAS/RAS)
      case 23: // 2^22 = ~8MHz (Dot Clock)
        counter_colors[j] = (MAX_BRIGHTNESS >> 2) << 8;
        break;
      default:
        counter_colors[j] = (MAX_BRIGHTNESS >> 5) << 16;
        break;
    }
  }

  stdio_init_all();
  adc_gpio_init(PULSE_PIN_ADC);
  adc_init();

  // set up PIO pin sampling
  uint offset = pio_add_program(pio, &samplepin_pio_program);
  samplepin_pio_sm = pio_claim_unused_sm(pio, true);
  samplepin_pio_program_init(pio, samplepin_pio_sm, offset, PULSE_PIN, STROBE_POSEDGE_PIN, STROBE_NEGEDGE_PIN, SAMPLE_DELAY, 1);
  //busy_wait_us(1000000);
  //printf("samplepin pio offset: %d\n", offset);

  // set up PWM pulse counters
  counter1_slice = pulse_counter_init(PULSE_PIN);
  counter2_slice = pulse_counter_init(COMPARE_PIN);

  // wait for start signal on pins
  uint32_t waitcount = 0;
  while((gpio_get_all() & ((1 << START_POSEDGE_PIN) | (1 << START_NEGEDGE_PIN))) != (1 << START_POSEDGE_PIN)) {
    waitcount++;
    if(!(waitcount & 0x1fffff)) {
      uint32_t intensity = ((waitcount >> 21) & 0xf) + 8;
      neopixel.fillPixelColor(intensity, intensity, 0);
      neopixel.show();
    }
  }

  // start things
  pwm_set_mask_enabled((1 << counter1_slice) | (1 << counter2_slice));
  pio_sm_set_enabled(pio, samplepin_pio_sm, true);
  multicore_launch_core1(core1_loop);

  // main loop
  uint64_t have_signal_since = 0;
  bool compare_matching = 1;
  bool compare_mode = 0;
  uint32_t count_prev = 0;
  uint32_t count2_prev = 0;
  while(1) {
    const uint32_t iterations = 50*10; // for about 10 seconds...
    uint32_t pulse_counts[iterations];
    neopixel.clear();
    float freq_led = WSLEDS_COUNT + 1;
    for(uint j = 0; j < iterations; j++) {
      uint32_t psrand = j;
      uint32_t count = pulse_count;
      __compiler_memory_barrier();
      uint32_t count2 = compare_count;
      __compiler_memory_barrier();
      if(count < count2 && pulse_count >= count2) count2 = count; // workaround for race condition in compare mode
      __compiler_memory_barrier();
      pulse_counts[j] = count;
      if(j > 0) {
        uint32_t start_window = j > 30 ? j - 30 : 0;
        uint32_t elapsed_ms = (j - start_window) * 20; // TODO consider processing time, too
        uint32_t start_count = pulse_counts[start_window];
        uint32_t pulses = count > start_count ? count - start_count : (0xffffffff - start_count) + count + 1;
        uint32_t bkgcol = 0;
        // bit of blue flash on first signal
        if(!pulses) {
          freq_led = 0;
          have_signal_since = 0;
        } else {
          float mult = 1000.0 / elapsed_ms;
          freq_led = log2(pulses * mult) * 2;
          if(have_signal_since) {
            float timediff = time_us_64() - have_signal_since;
            if(timediff < 250000) {
              bkgcol = (1.0 - (timediff / 250000)) * 6; // blue
            }
          } else {
            have_signal_since = time_us_64();
          }
        }
        // frequency "eye"
        uint32_t freq_led_y = round(freq_led) / WSLEDS_WIDTH;
        float freq_led_x = freq_led - freq_led_y * WSLEDS_WIDTH;
        for(uint32_t led = 0; led < WSLEDS_COUNT; led++) {
          float led_x = led % WSLEDS_WIDTH;
          float led_y = led / WSLEDS_WIDTH;
          float distance = (pow(freq_led_x - led_x, 2) + pow(freq_led_y - led_y, 2)) / 1.5;
          float intensity = pow(2, -distance);
          if(intensity < 0.1) {
            neopixel.setPixelColor(led, bkgcol);
          } else {
            neopixel.setPixelColor(led, color_intensity(FREQ_LED_COL, std::min(intensity / 3, 0.2f)));
          }
        }
      }

      compare_mode = count2 > 1;
      if(compare_mode && compare_matching) compare_matching = (count2 <= (count + 1)) && (count2 >= (count - 1));

      // binary counter
      uint32_t freq_led_int = round(freq_led);
      uint32_t mask = 1;
      for(uint i = 0; i < 32; i++) {
        float intensity = 0;
        psrand++;
        if((count - count_prev) > mask) {
          // counting too fast for this digit: prevent aliasing effects, make LED flicker dimly
          intensity = (psrand & 1) ? 0.4 : 0.7;
        } else if(count & mask) intensity = 1;
        if(intensity > 0) {
          uint32_t color;
          if(compare_mode) {
            color = compare_matching ? 0x002000 : 0x200000; // green: match, red: mismatch
          } else {
            color = counter_colors[i];
          }
          if(intensity == 1)
            if((i << 1) != freq_led_int) intensity *= 0.5;
          neopixel.setPixelColor(i << 1, color_intensity(color, intensity));
        }
        mask <<= 1;
      }
      count_prev = count;

      if(compare_mode) {
        // compare counter
        mask = 1;
        uint32_t color = compare_matching ? 0x0a200a : 0x200a00; // pale green: match, pale red: mismatch
        for(uint i = 0; i < 32; i++) {
          float intensity = 0;
          psrand++;
          if((count2 - count2_prev) > mask) {
            // counting too fast for this digit: prevent aliasing effects, make LED flicker dimly
            intensity = (psrand & 1) ? 0.4 : 0.7;
          } else if(count2 & mask) intensity = 1;
          if(intensity > 0) neopixel.setPixelColor(1 + (i << 1), color_intensity(color, intensity));
          mask <<= 1;
        }
      } else {
        // sampled bits
        uint64_t sample = sampled_bits;
        if(sampled_bits_remaining) {
          for(uint i = 0; i < 32; i++) {
            if(sample&1) neopixel.setPixelColor((i << 1) + 1, SAMPLE_LED_COL_ONE);
            sample >>= 1;
          }
        } else {
          for(uint i = 0; i < 64; i++) {
            neopixel.setPixelColor(i, (sample&1) ? SAMPLE_LED_COL_ONE : SAMPLE_LED_COL_ZERO);
            sample >>= 1;
          }
        }
      }
      count2_prev = count2;

      neopixel.show();
      busy_wait_us(20000);
    }

    // in compare mode, ADC capacitance would mess with COMPARE input readings
    if(compare_mode) continue;

    // pulse pin analog histogram
    adc_select_input(PULSE_PIN_ADC - 26);
    for(uint j = 0; j < 10*3; j++) { // for about 3 seconds...
      for(uint k = 0; k < WSLEDS_HEIGHT; k++) analog_buckets[k] = 0;
      // read adc
      for(uint i = 0; i < 50000; i++) { // for about 100ms (at 500kS/s)...
        analog_buckets[adc_read() >> 9]++; // adc reads 12 bits, we only want 3 (= 0..7) TODO consider WSLEDS_HEIGHT
      }
      // get max bucket count
      float max = 0;
      for(uint i = 0; i < WSLEDS_HEIGHT; i++) {
        if(analog_buckets[i] > max) max = analog_buckets[i];
      }
      // display histogram
      for(uint y = 0; y < WSLEDS_HEIGHT; y++) {
        float buck_count = analog_buckets[y];
        float q = buck_count / max;
        uint32_t color = bucket_colors[y];
        for(uint x = 0; x < WSLEDS_WIDTH; x++) {
          float f_x = x;
          float middle = 1 - abs((f_x / (WSLEDS_WIDTH - 1) - 0.5) * 2); // 0 if on the side, 1 in the middle
          float att = q + middle;
          if(att > 1) att = 1;
          float attenuation = ((1 - att) * 10) + 1;
          neopixel.setPixelColor(y * WSLEDS_HEIGHT + x, color_intensity(color, pow(q, attenuation)));
        }
      }
      neopixel.show();
      have_signal_since = time_us_64() - 300000;
    }
    adc_select_input(4); // onboard temp sensor
    __compiler_memory_barrier();
  }
}



