/*
 * stub_main.cpp -- smoke-test entry point for the sketch builds.
 *
 * The Arduino runtime calls setup() once and loop() forever.  Here we call
 * setup() once and loop() a bounded number of times, which turns
 * `sketch-check` from a compile test into a real execution test: a null
 * pointer, a divide by zero, an infinite inner loop or a bad static
 * initialiser in either sketch becomes a failing check instead of a crashed
 * robot arm.
 *
 * The fake clock in stubs.cpp advances one millisecond per millis() call, so
 * the millis()-based schedulers inside the sketches do fire.
 */
extern void setup();
extern void loop();

int main() {
    setup();
    for (int i = 0; i < 20000; ++i) {
        loop();
    }
    return 0;
}
