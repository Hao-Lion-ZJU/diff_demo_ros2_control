#ifndef DIFF_TEST_CONTROLLER_VISIBILITY_CONTROL_H
#define DIFF_TEST_CONTROLLER_VISIBILITY_CONTROL_H


// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIFF_TEST_CONTROLLER_EXPORT __attribute__((dllexport))
#define DIFF_TEST_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define DIFF_TEST_CONTROLLER_EXPORT __declspec(dllexport)
#define DIFF_TEST_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef DIFF_TEST_CONTROLLER_BUILDING_DLL
#define DIFF_TEST_CONTROLLER_PUBLIC DIFF_TEST_CONTROLLER_EXPORT
#else
#define DIFF_TEST_CONTROLLER_PUBLIC DIFF_TEST_CONTROLLER_IMPORT
#endif
#define DIFF_TEST_CONTROLLER_PUBLIC_TYPE DIFF_TEST_CONTROLLER_PUBLIC
#define DIFF_TEST_CONTROLLER_LOCAL
#else
#define DIFF_TEST_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define DIFF_TEST_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define DIFF_TEST_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define DIFF_TEST_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define DIFF_TEST_CONTROLLER_PUBLIC
#define DIFF_TEST_CONTROLLER_LOCAL
#endif
#define DIFF_TEST_CONTROLLER_PUBLIC_TYPE
#endif

#endif /* DIFF_TEST_CONTROLLER_VISIBILITY_CONTROL_H */

