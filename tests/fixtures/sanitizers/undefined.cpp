#include <climits>

int main(int argc, char**) {
    volatile int value = argc > 1 ? INT_MAX : 1;
    volatile int result = value + 1;
    (void)result;
    return 0;
}
