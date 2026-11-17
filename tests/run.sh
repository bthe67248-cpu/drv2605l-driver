#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM
cc -std=c11 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror -c drv2605l.c -o "$build_dir/c.o"
c++ -x c++ -std=c++11 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror -c drv2605l.c -o "$build_dir/cpp.o"
cc -std=c11 -Wall -Wextra -Wpedantic -Werror drv2605l.c tests/test_driver.c -I. -o "$build_dir/test_driver"
"$build_dir/test_driver"
