#ifndef DIFF_TEST_CONTROL__VISIBILITY_CONTROL_H_
#define DIFF_TEST_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIFF_TEST_CONTROL_EXPORT __attribute__((dllexport))
#define DIFF_TEST_CONTROL_IMPORT __attribute__((dllimport))
#else
#define DIFF_TEST_CONTROL_EXPORT __declspec(dllexport)
#define DIFF_TEST_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef DIFF_TEST_CONTROL_BUILDING_DLL
#define DIFF_TEST_CONTROL_PUBLIC DIFF_TEST_CONTROL_EXPORT
#else
#define DIFF_TEST_CONTROL_PUBLIC DIFF_TEST_CONTROL_IMPORT
#endif
#define DIFF_TEST_CONTROL_PUBLIC_TYPE DIFF_TEST_CONTROL_PUBLIC
#define DIFF_TEST_CONTROL_LOCAL
#else
#define DIFF_TEST_CONTROL_EXPORT __attribute__((visibility("default")))
#define DIFF_TEST_CONTROL_IMPORT
#if __GNUC__ >= 4
#define DIFF_TEST_CONTROL_PUBLIC __attribute__((visibility("default")))
#define DIFF_TEST_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define DIFF_TEST_CONTROL_PUBLIC
#define DIFF_TEST_CONTROL_LOCAL
#endif
#define DIFF_TEST_CONTROL_PUBLIC_TYPE
#endif

#endif  // DIFF_TEST_CONTROL__VISIBILITY_CONTROL_H_
