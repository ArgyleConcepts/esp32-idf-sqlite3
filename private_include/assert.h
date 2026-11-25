/*
 * ESP-IDF's platform <assert.h> evaluates assert() expressions even under NDEBUG
 * when CONFIG_COMPILER_ASSERT_NDEBUG_EVALUATE=y. Upstream SQLite 3.25.2 places
 * SQLITE_DEBUG-only helper calls and variables inside assert(), relying on ISO C99
 * NDEBUG semantics where assert(x) expands to ((void)0).
 */
#ifndef ESP32_SQLITE_ASSERT_H
#define ESP32_SQLITE_ASSERT_H

#ifdef NDEBUG
#define assert(expr) ((void)0)
#else
#include_next <assert.h>
#endif

#endif
