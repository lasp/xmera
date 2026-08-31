
.. _fuzz-testing:

Fuzz Testing
============

A fuzz test states a property that must hold for every input, then lets
`Google FuzzTest <https://github.com/google/fuzztest>`__ search the input domain
for a counterexample. Where a unit test pins one input to one expected output, a
fuzz test pins the whole domain to an invariant:

.. code-block:: cpp

    // Unit test: one scenario
    EXPECT_EQ(algorithm.update(knownInput), expectedOutput);

    // Fuzz test: a property of every valid input
    void fuzzAlgorithm(int32_t x, int32_t y) {
        auto result = algorithm.update(x, y);
        EXPECT_GE(result.count, 0);
        EXPECT_LE(result.count, x + y);
    }
    FUZZ_TEST(MySuite, fuzzAlgorithm)
        .WithDomains(fuzztest::InRange(0, 1'000), fuzztest::InRange(0, 1'000));

The same ``FUZZ_TEST`` serves two roles. In a build without fuzzing
instrumentation it runs as an ordinary parameterized GTest over a small set of
generated inputs, which is quick enough for a normal ``ctest`` run. In a build
with ``FUZZTEST_FUZZING_MODE``, it becomes a coverage-guided fuzzer that mutates
inputs to reach new branches.

This page covers running the fuzz tests, writing one for your module, and
reproducing a finding.

There are two modules with fuzz tests today. Read them before writing your own:

- ``src/fswAlgorithms/imageProcessing/regionsOfInterest/tests/test_regionsOfInterest_fuzz.cpp``
  -- the plain idiom, with each property function followed by its ``FUZZ_TEST``.
- ``src/fswAlgorithms/imageProcessing/centerOfBrightness/tests/`` -- the variant
  that keeps the property functions and a fake image reader in
  ``centerOfBrightnessTestHelpers.hpp``, so the ``_fuzz.cpp`` file holds only the
  registrations. It also shows the reference oracle idiom.


Running the Fuzz Tests
----------------------

Fuzz targets are built only when ``XMERA_ENABLE_FUZZTESTS`` is on. Two presets in
``src/CMakePresets.json`` turn it on:

- ``fuzz-smoke-test`` -- sets ``XMERA_ENABLE_FUZZTESTS=ON``. Each ``FUZZ_TEST``
  runs as a normal GTest.
- ``fuzz-test`` -- inherits the above and adds ``FUZZTEST_FUZZING_MODE=ON``, which
  builds the coverage instrumentation and the sanitizer.

The first configure with either preset fetches Google FuzzTest through
``FetchContent`` (see ``src/cmake/XmeraGoogleTest.cmake``), thus it takes several
minutes. Later configures use the cached checkout.

As unit tests
~~~~~~~~~~~~~

.. code-block:: bash

   cd src
   cmake --preset fuzz-smoke-test
   cmake --build ../build --parallel
   cd ../build && ctest -C Release -L fuzz-smoke

This finishes in seconds. Use it to confirm a new fuzz test compiles, registers,
and passes its generated inputs before you start a long session.

As a coverage-guided fuzzer
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Build with the ``fuzz-test`` preset, then run one fuzz test for a duration:

.. code-block:: bash

   cd src
   cmake --preset fuzz-test
   cmake --build ../build --parallel

   BIN=../build/fswAlgorithms/imageProcessing/regionsOfInterest/tests/test_regionsOfInterest_fuzz
   "$BIN" --list_fuzz_tests
   "$BIN" --fuzz=RegionsOfInterestFuzz.fuzzRegionIdentification --fuzz_for=120s

``--list_fuzz_tests`` prints one ``[*] Fuzz test: <Suite.TestName>`` line for each
registered test and exits. ``--fuzz=`` takes one of those names, and accepts a
substring when the substring matches exactly one test. An empty ``--fuzz=`` works
only in a binary that holds a single ``FUZZ_TEST``. A name that matches no
``FUZZ_TEST`` stops the binary with a non-zero exit code.

The session prints new coverage as it finds it. On a counterexample it prints the
failing input and exits non-zero. See `Reproducing a Finding`_.

Keeping a corpus between sessions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A *corpus* is the set of inputs the fuzzer has found interesting, that is, the
inputs that reached a branch nothing else reached. Give a session the previous
corpus and it continues from that coverage instead of from nothing.

Two environment variables control this. Point both at the same directory:

.. code-block:: bash

   mkdir -p /tmp/fuzz-corpus/RegionsOfInterestFuzz.fuzzRegionIdentification
   FUZZTEST_TESTSUITE_IN_DIR=/tmp/fuzz-corpus/RegionsOfInterestFuzz.fuzzRegionIdentification \
   FUZZTEST_TESTSUITE_OUT_DIR=/tmp/fuzz-corpus/RegionsOfInterestFuzz.fuzzRegionIdentification \
   "$BIN" --fuzz=RegionsOfInterestFuzz.fuzzRegionIdentification --fuzz_for=60s

Give **each fuzz test its own directory**. A corpus file holds the serialized
parameters of one ``FUZZ_TEST``, and two tests rarely have the same parameter
signature. Two tests that share a directory each reject the files of the other
with ``Unexpected intermediate representation``.

The fuzzer writes corpus files as it finds them, not at exit, thus the corpus
survives a crash or an interrupt.

All targets at once
~~~~~~~~~~~~~~~~~~~

``.github/scripts/run_long_fuzzers.sh`` runs every fuzz test in the build tree and
handles the corpus layout:

.. code-block:: bash

   .github/scripts/run_long_fuzzers.sh ./build ./fuzz-logs --fuzz-for 120s
   .github/scripts/run_long_fuzzers.sh ./build ./fuzz-logs --fuzz-for 120s --corpus .fuzztest_corpus

The first argument is the build directory, because the script runs
``ctest --show-only=json-v1 -L fuzz`` there to find the fuzz *binaries*. It then
asks each binary for its test names with ``--list_fuzz_tests`` and runs them one at
a time. The duration applies to each test, not to each binary.

With ``--corpus`` it creates ``<corpus>/<binary>/<Suite.TestName>`` for each test.
It writes one log for each test to the log directory, and exits non-zero with a
list of the tests that reported a finding.


Writing a Fuzz Test
-------------------

File layout
~~~~~~~~~~~

Put the fuzz test beside the unit tests of the module, named
``test_<moduleName>_fuzz.cpp``:

.. code-block:: none

    src/fswAlgorithms/<category>/<moduleName>/
        tests/
            CMakeLists.txt
            test_<moduleName>.cpp          # unit tests
            test_<moduleName>_fuzz.cpp     # fuzz tests

Do not include the unit test file from the fuzz file. The two build into separate
binaries, and the include also gives the unit tests the fuzz labels.

The property function
~~~~~~~~~~~~~~~~~~~~~

A property function takes the fuzzed values as ordinary parameters, returns
``void``, and asserts with the GTest macros. This is ``fuzzRegionIdentification``
from the regions of interest module:

.. code-block:: cpp

   #include "../regionsOfInterestAlgorithm.h"
   #include "gtest/gtest.h"

   #include <fuzztest/fuzztest.h>

   void fuzzRegionIdentification(
       int32_t maxSeparation,
       int32_t minDetectionSize,
       std::vector<int32_t> regionXs,
       std::vector<int32_t> regionYs,
       std::vector<int32_t> regionPixels
   ) {
       size_t const numRegions = std::min({regionXs.size(), regionYs.size(), regionPixels.size(), FUZZ_MAX_REGIONS});

       std::array<RegionOfInterest, MAX_NUMBER_REGIONS> regions{};
       int32_t totalInputPixels = 0;
       for (size_t i = 0; i < numRegions; ++i) {
           regions[i].numberOfPixels = regionPixels[i];
           regions[i].centerOfBrightness << regionXs[i], regionYs[i];
           regions[i].regionCenter << regionXs[i], regionYs[i];
           totalInputPixels += regionPixels[i];
       }

       RegionsOfInterestAlgorithm algorithm;
       algorithm.setImageSize(FUZZ_MAX_IMAGE_SIZE, FUZZ_MAX_IMAGE_SIZE);
       algorithm.setMaxRoiSeparation(maxSeparation);
       algorithm.setMinimumDetectionSize(minDetectionSize);
       // Without reset() the window keeps its default size of 1024x1024 and rejects each
       // region outside it, whatever image size the test sets above.
       algorithm.reset();

       RegionOfInterest result = algorithm.update(regions);

       EXPECT_GE(result.numberOfPixels, 0);
       EXPECT_LE(result.numberOfPixels, totalInputPixels);
   }

Registration and domains
~~~~~~~~~~~~~~~~~~~~~~~~

``FUZZ_TEST`` registers the function, and ``.WithDomains`` gives one domain for
each parameter, in order:

.. code-block:: cpp

   FUZZ_TEST(RegionsOfInterestFuzz, fuzzRegionIdentification)
       .WithDomains(
           fuzztest::InRange(1, 2'000),  // maxSeparation
           fuzztest::InRange(0, 100),    // minDetectionSize
           // The length of the fuzzed vector gives the region count. Thus each slot
           // of the MAX_NUMBER_REGIONS array is available, and zero slots also.
           fuzztest::VectorOf(fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE)).WithMaxSize(FUZZ_MAX_REGIONS),
           fuzztest::VectorOf(fuzztest::InRange(0, FUZZ_MAX_IMAGE_SIZE)).WithMaxSize(FUZZ_MAX_REGIONS),
           fuzztest::VectorOf(fuzztest::InRange(0, FUZZ_MAX_PIXELS)).WithMaxSize(FUZZ_MAX_REGIONS)
       );

The domains used in this repository:

- ``fuzztest::InRange(lo, hi)`` -- an integer or floating point value in the
  inclusive range.
- ``fuzztest::Arbitrary<T>()`` -- any value of type ``T``.
- ``fuzztest::VectorOf(domain).WithMaxSize(n)`` -- a vector of at most ``n``
  elements, thus the fuzzer varies the length as well as the elements, and the
  empty vector is reachable. ``.WithSize(n)`` fixes the length instead.
- ``fuzztest::OneOf(a, b, c)`` -- one of several sub-domains. This concentrates the
  values where the interesting behaviour is. The regions of interest tests factor
  it into a helper:

  .. code-block:: cpp

     // The position domains give values at the image edges, adjacent to the edges, and at
     // the middle. regionInWindow uses strict inequalities, thus a value at the edge is
     // outside.
     static auto edgeCasePosition() {
         return fuzztest::OneOf(
             fuzztest::InRange(0, 1),
             fuzztest::InRange(FUZZ_MIN_IMAGE_SIZE, FUZZ_MAX_IMAGE_SIZE / 2),
             fuzztest::InRange(FUZZ_MAX_IMAGE_SIZE - 1, FUZZ_MAX_IMAGE_SIZE)
         );
     }

Add a trailing comment naming the parameter on each domain line. The domains are
positional, and a signature that changes without its domain list is a silent
error.

The `FuzzTest domain reference
<https://github.com/google/fuzztest/blob/main/doc/domains-reference.md>`__ lists
the rest.

CMake wiring
~~~~~~~~~~~~

Add a guarded block to the module's ``tests/CMakeLists.txt``:

.. code-block:: cmake

   if(XMERA_ENABLE_FUZZTESTS)
     fuzztest_setup_fuzzing_flags()

     add_executable(test_myModule_fuzz
       ../myModuleAlgorithm.cpp
       test_myModule_fuzz.cpp
     )

     target_include_directories(test_myModule_fuzz PRIVATE "../..")
     target_include_directories(test_myModule_fuzz PRIVATE "..")
     target_include_directories(test_myModule_fuzz PRIVATE "${CMAKE_SOURCE_DIR}")

     target_link_libraries(test_myModule_fuzz PRIVATE
       Eigen3::Eigen
       fuzztest::fuzztest
       fuzztest::fuzztest_gtest_main
     )

     # The labels come from xmera_label_discovered_tests, not from PROPERTIES. Refer to the
     # note on that function in src/cmake/XmeraGoogleTest.cmake.
     gtest_discover_tests(test_myModule_fuzz
       TEST_LIST myModuleFuzzTests
     )
     xmera_label_discovered_tests(test_myModule_fuzz myModuleFuzzTests fuzz fuzz-smoke)
   endif()

- Guard the whole block with ``if(XMERA_ENABLE_FUZZTESTS)``, so the default build
  does not change.
- Call ``fuzztest_setup_fuzzing_flags()`` before ``add_executable``. It adds the
  instrumentation flags to the targets that follow it.
- Link ``fuzztest::fuzztest`` and ``fuzztest::fuzztest_gtest_main``, not
  ``GTest::gtest_main``.
- Apply the labels with ``xmera_label_discovered_tests``, which needs the
  ``TEST_LIST`` from ``gtest_discover_tests`` and must be called after it in the
  same directory. ``PROPERTIES LABELS`` cannot give a test two labels; see
  `Only one of the two labels appears`_.

The two labels mean different things:

- ``fuzz-smoke`` -- the test is quick enough for a normal ``ctest`` run.
- ``fuzz`` -- the test is a target for a long fuzzing session.

Give a new test both. Give it ``fuzz`` alone when it becomes too slow for a normal
test run.


Choosing Invariants
-------------------

Choosing what to assert is the hard part. The usual strategies, with the form each
takes in this repository:

**No crash.** The weakest invariant and still a useful one. A segmentation fault,
an out of bounds read that the sanitizer catches, or an unhandled exception on any
input is a bug. This costs nothing to get, because calling the algorithm is
already part of the test.

**Output bounds.** ``EXPECT_GE(result.numberOfPixels, 0)``.

**Conservation.** The output cannot exceed a known function of the inputs, for
example ``EXPECT_LE(result.numberOfPixels, totalInputPixels)``. Where the
algorithm merges or accumulates, this is usually the strongest cheap invariant
available.

**Containment.** A derived value must stay inside the bounds of the values that
produced it. ``fuzzRegionMerging`` checks that the resulting center of brightness
lies inside the bounding box of the regions that contributed to it.

**An exception contract.** When part of the domain is invalid, assert the
rejection rather than steering the domain away from it.
``fuzzWindowingBehavior`` computes whether the window fits and then asserts the
matching outcome:

.. code-block:: cpp

   if (!fits) {
       ASSERT_THROW(algorithm.reset(), std::invalid_argument);
       return;  // the window was rejected, so there is no configured algorithm to drive
   }
   ASSERT_NO_THROW(algorithm.reset());

**A reference oracle.** Write a slow, obviously correct implementation beside the
test and compare against it. This is the strongest invariant, because it pins the
value and not only its range. The center of brightness helpers keep a
``referenceUpdate`` that recomputes the result from the visible pixels, and the
property function compares field by field:

.. code-block:: cpp

   CenterOfBrightnessResult refResult = referenceUpdate(visible, brightnessThreshold, refState);

   EXPECT_NEAR(result.centerOfBrightness[0], refResult.centerOfBrightness[0], 1e-9);
   EXPECT_EQ(result.pixelsFound, refResult.pixelsFound);
   EXPECT_NEAR(result.rollingAverageBrightness, refResult.rollingAverageBrightness, 1e-9);

**Idempotence and round trips.** Two calls with the same input give the same
result; an encode followed by a decode gives the original value.

Avoid an assertion that the code cannot break. An assertion derived from the same
expression the algorithm used is a tautology: it passes for every input, adds
fuzzing time, and hides the absence of a real check.


Reproducing a Finding
---------------------

When the fuzzer finds a counterexample, it prints a ``=== BUG FOUND!`` block with
the failing arguments, and, unless the test uses a fixture, a
``=== Regression test draft`` block below it:

.. code-block:: none

   =================================================================
   === Regression test draft

   TEST(RegionsOfInterestFuzz, fuzzRegionIdentificationRegression) {
     fuzzRegionIdentification(
       1,
       0,
       ...
     );
   }

Paste that draft into the module's unit test file. It calls the property function
directly, so the assertions come with it, and the case stays covered in the normal
test run after you fix the algorithm. This is the whole workflow for most
findings.

For an input too large to paste, or one that needs shrinking first, use the
reproducer file. **A local run writes no reproducer file unless you ask for one.**
Without ``FUZZTEST_REPRODUCERS_OUT_DIR`` the session prints
``[.] No reproducer output location specified - not writing the reproducer file.``
and the input is lost. Set it before the run:

.. code-block:: bash

   mkdir -p /tmp/fuzz-reproducers
   FUZZTEST_REPRODUCERS_OUT_DIR=/tmp/fuzz-reproducers \
   "$BIN" --fuzz=RegionsOfInterestFuzz.fuzzRegionIdentification --fuzz_for=120s

The session then prints ``Reproducer file was dumped at: <path>``. Replay that file
against the same test:

.. code-block:: bash

   FUZZTEST_REPLAY=/tmp/fuzz-reproducers/<file> \
   "$BIN" --gtest_filter=RegionsOfInterestFuzz.fuzzRegionIdentification

``FUZZTEST_REPLAY`` also accepts a directory, in which case it replays every file
in it. To shrink the input to the smallest one that still fails:

.. code-block:: bash

   FUZZTEST_MINIMIZE_REPRODUCER=/tmp/fuzz-reproducers/<file> \
   FUZZTEST_REPRODUCERS_OUT_DIR=/tmp/fuzz-reproducers \
   "$BIN" --gtest_filter=RegionsOfInterestFuzz.fuzzRegionIdentification

Each smaller failing input is written to the reproducer directory. Replay the
smallest one, take its regression test draft, and add that to the unit tests.


Troubleshooting
---------------

The coverage does not increase
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

An invariant has value only if the inputs can reach it. Two conditions stop the
inputs from reaching the code, and neither is easy to see, because the test keeps
reporting PASSED.

- **A fake that ignores its arguments.** A test double can accept a window center
  and a size, then give the same pixels for every value. The domains for those
  parameters are then dead. The fuzzer continues to change the values and to store
  them in each corpus entry, but the result does not change. When you write a fake,
  make sure each argument changes what the fake gives.

- **A test that does not call** ``reset()``. Some algorithms compute their window
  in ``reset()``. Until the test calls it, the algorithm uses the values from its
  constructor. A test can set an image size of 4096 pixels, skip ``reset()``, and
  leave the algorithm on its default window, which rejects most of the coordinate
  domain. If the default window rejects every input, the test always gets an empty
  result.

Watch the ``Edges covered`` count in the session output. If it does not increase
after a few seconds, check the test for these two conditions.

``Unexpected intermediate representation``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The corpus directory holds files from a different ``FUZZ_TEST``. Each test
serializes its own parameter signature, thus a test cannot parse another test's
files and rejects them. Give each fuzz test its own corpus directory. This is what
``run_long_fuzzers.sh`` does with ``<corpus>/<binary>/<Suite.TestName>``.

A ``ctest`` label filter removes more than expected
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

CTest compares labels with a regular expression that has no anchors. Thus
``-LE fuzz`` excludes every test whose label *contains* ``fuzz``, which includes
``fuzz-smoke``. Use an anchored expression, such as ``-LE '^fuzz$'``, when you mean
one label only.

Only one of the two labels appears
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``gtest_discover_tests(... PROPERTIES LABELS "fuzz;fuzz-smoke")`` loses the second
label. It sends the properties through a command line and expands them again
without quotation marks, thus ``LABELS "fuzz;fuzz-smoke"`` becomes the three tokens
``LABELS fuzz fuzz-smoke``. That is an odd number of tokens for a property list,
and the second label never becomes a label.

``xmera_label_discovered_tests`` in ``src/cmake/XmeraGoogleTest.cmake`` exists for
this. It writes a small CMake file into ``TEST_INCLUDE_FILES``, so the labels are
applied when ``ctest`` runs, which is after ``gtest_discover_tests`` has found the
test names.

Unit tests appear under the ``fuzz`` label
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``gtest_discover_tests`` labels every test in the binary. If the fuzz translation
unit includes the unit test one, the unit tests get the fuzz labels too. Keep the
two translation units separate.

``run_long_fuzzers.sh`` is written to tolerate this: it uses the label only to find
the *binaries*, then asks each binary for its real fuzz test names with
``--list_fuzz_tests``.

The iterations are slow
~~~~~~~~~~~~~~~~~~~~~~~

The fuzzer runs the property function thousands of times, thus per-iteration setup
dominates. Hoist expensive state into ``static`` locals and reset only what the
iteration dirtied. The center of brightness helpers keep the algorithm and the
reader alive between iterations:

.. code-block:: cpp

   // The algorithm contains an 8 MB pixel buffer, and the reader records how much of that
   // buffer it wrote. Thus both must stay alive between iterations.
   static CenterOfBrightnessAlgorithm alg;
   static FuzzImageReader reader;

   // reset() clears the brightness history, which is the only state kept between runs
   alg.reset();

The fake clears only the prefix of the pixel buffer it wrote on the previous call,
which keeps an 8 MB fill out of every iteration. State that survives an iteration
must be reset deliberately, or one input changes the result of the next.


In CI
-----

Pull request CI configures the ``fuzz-smoke-test`` preset and runs the whole test
suite with no label filter, thus the fuzz tests run beside the unit tests and must
stay fast. A nightly job runs ``run_long_fuzzers.sh`` for a longer duration against
a cached corpus and fails on a finding. See
``.github/workflows/nightly-long-tests.yml`` for the detail.
