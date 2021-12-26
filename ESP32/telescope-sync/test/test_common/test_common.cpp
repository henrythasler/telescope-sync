#include <unity.h>
#include <telescope.h>

#ifndef ARDUINO
#include <stdio.h>
#include <chrono>
#endif

void test_function_rad(void)
{
  Telescope telescope;
  TEST_ASSERT_EQUAL_FLOAT(M_PI_2, telescope.rad(90));
  TEST_ASSERT_EQUAL_FLOAT(M_PI, telescope.rad(180));
  TEST_ASSERT_EQUAL_FLOAT(2 * M_PI, telescope.rad(360));

  TEST_ASSERT_EQUAL_FLOAT(-M_PI_2, telescope.rad(-90));
  TEST_ASSERT_EQUAL_FLOAT(-M_PI, telescope.rad(-180));
  TEST_ASSERT_EQUAL_FLOAT(-2 * M_PI, telescope.rad(-360));
}

void test_function_deg(void)
{
  Telescope telescope;
  TEST_ASSERT_EQUAL_FLOAT(90, telescope.deg(M_PI_2));
  TEST_ASSERT_EQUAL_FLOAT(180, telescope.deg(M_PI));
  TEST_ASSERT_EQUAL_FLOAT(360, telescope.deg(2 * M_PI));

  TEST_ASSERT_EQUAL_FLOAT(-90, telescope.deg(-M_PI_2));
  TEST_ASSERT_EQUAL_FLOAT(-180, telescope.deg(-M_PI));
  TEST_ASSERT_EQUAL_FLOAT(-360, telescope.deg(-2 * M_PI));
}

void test_function_degToHours(void)
{
  Telescope telescope;
  TEST_ASSERT_EQUAL_FLOAT(0, telescope.degToHours(0));
  TEST_ASSERT_EQUAL_FLOAT(6, telescope.degToHours(90));
  TEST_ASSERT_EQUAL_FLOAT(12, telescope.degToHours(180));
  TEST_ASSERT_EQUAL_FLOAT(0, telescope.degToHours(360));
  TEST_ASSERT_EQUAL_FLOAT(0, telescope.degToHours(-360));
}

void test_function_fromHorizontalPosition(void)
{
  Telescope telescope;
  telescope.fromHorizontalPosition(180, 60.34, 38.59, 297.93);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 297.92, telescope.position.ra);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 8.93, telescope.position.dec);

  telescope.fromHorizontalPosition(269.14634, 49.169122, 52.5, 304.80762);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 250.43, telescope.position.ra);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 36.47, telescope.position.dec);
}

void test_function_toHorizontalPosition(void)
{
  Telescope telescope(250.425, 36.467);
  Telescope::Horizontal res = telescope.toHorizontalPosition(52.5, 304.808);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 269.14634, res.az);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 49.169122, res.alt);
}


void process(void)
{
  UNITY_BEGIN();
  RUN_TEST(test_function_rad);
  RUN_TEST(test_function_deg);
  RUN_TEST(test_function_degToHours);
  RUN_TEST(test_function_fromHorizontalPosition);
  RUN_TEST(test_function_toHorizontalPosition);
  UNITY_END();
}

#ifdef ARDUINO
#include <Arduino.h>
#define LED_BUILTIN (13) // LED is connected to IO13

void setup()
{
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  delay(2000);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.printf("Heap: %u KiB free\n", ESP.getFreeHeap() / 1024);
  process();
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
}

#else

int main(int argc, char **argv)
{
  auto start = std::chrono::high_resolution_clock::now();
  process();
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  printf("Elapsed time: %lums\n", duration.count() / 1000);
  return 0;
}
#endif