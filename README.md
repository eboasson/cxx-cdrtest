# C/C++ serialization testing for Cyclone DDS

`cxx-cdrtest.pl` generates random types along with test code, writing:
* `xxx-types.idl`: the IDL file containing one or more topic definitions
* `xxx.cpp`: a C++ program that uses both the C and C++ language bindings to read and
  write data and to compare the contents

It additionally has to option of maintaining a database of types that have previously been
generated and should not be generated again. If specified, it reads that file (**eval**ing
it, because it is a `Dumper` output with one type per line) before generating anything,
and appends the newly generated types to it.

The script is a mess with some options:
* `-a`: annotate with `@data_representation(XCDR2)`
* `-c FILE`: aforemention database
* `-i CNT`: generate CNT random instances of each generated type instead of 50
* `-l DEPTH`: limit type generation depth to DEPTH instead of 10 (typedefs don't count)
* `-n CNT`: generate CNT types, defaults to 1 if `-c` given, else to 100

It can additionally be passed an initial seed. If started with the same database, passing
the same seed will result in the same output.  Hopefully.

Beware that compiling the output with many types can take a while (and some memory).

It has a table of likelihoods for the different types it can generate as well as whole
bunch of workarounds to deal with (old) limitations of the IDL compiler and the
serializers.  Eventually they should all end up being 0; for now, do check if they are
what you need.

`cxx-cdrtest.bash` is a wrapper that automates the process of generating random types,
compiling the output and running it.  If a test fails or crashes, it generates a zip file
named `failure-SEED.zip` (where `SEED` is the seed used by the Perl script) containing all
the generated sources (`IDL`, C++ and C).

It recognizes one special argument `rebuild`, which skips the step generating the IDL file
and only compiles and runs the test.  Any other argument is assumed to be a seed to be
passed on to the Perl script.

It features some additional options:
* `-r`: repeat
* `-k`: if repeating, keep going after errors
* `-Pc:PATH`: Cyclone C is installed in `PATH`
* `-Pcxx:PATH`: Cyclone C++ is installed in `PATH`, or `PATH` is a C++ build directory
  that is a child of the C++ source repository
* `-Piox:PATH`: Iceoryx is installed in `PATH` (only relevant if Cyclone was compiled with
  Iceoryx support)

The defaults are probably not useful to anyone but me.  It attempts to figure out the
correct include and link paths from these with a few heuristics because of CMake/packaging
nonsense.

YMMV.
