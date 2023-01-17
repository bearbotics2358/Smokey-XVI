#pragma once

namespace lg {
// prints out info, with a newline at the end
void info(const char *fmt, ...);

// prints out a warning, with a newline at the end
void warn(const char *fmt, ...);

// prints out an error, with a newline at the end
void error(const char *fmt, ...);

// prints out a critical error, with a newline at the end
// terminates the program
[[noreturn]] void critical(const char *fmt, ...);
}
