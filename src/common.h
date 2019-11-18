#pragma once
/**
 * Some useful common things
 */

#define __countof(x) (sizeof(x)/sizeof(x[0]))


// Stringify defines
#define __TO_STR2(x) #x
#define TO_STR(x) __TO_STR2(x)

template <class T> T _min(T a, T b) {
    return a > b ? b : a;
}