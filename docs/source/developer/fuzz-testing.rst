
.. _fuzz-testing:

Fuzz Testing
============

Fuzz testing automatically generates randomized inputs to find crashes, assertion
violations, and other bugs that hand-written test cases miss. Xmera uses
`Google FuzzTest <https://github.com/google/fuzztest>`__ to write fuzz tests
alongside regular GTest unit tests.

Goals
-----

Fuzz testing in Xmera serves three purposes:

1. **Find crashes and undefined behavior** -- the fuzzer explores input
   combinations that humans rarely think to try (overflows, empty arrays,
   extreme values).

2. **Verify algorithmic invariants** -- rather than checking a single expected
   output, fuzz tests assert properties that must hold for *all* valid inputs
   (e.g., "pixel count is non-negative", "merged region does not exceed sum of
   inputs").

3. **Continuous regression coverage** -- a nightly CI job runs each fuzz target
   for an extended duration and persists a corpus of interesting inputs. Over
   time the corpus grows, giving each subsequent run a head start.


Property-Based Tests (Fuzz Tests)
---------------------------------

Traditional unit tests verify specific input/output pairs:

.. code-block:: cpp

    // Unit test: one specific scenario
    EXPECT_EQ(algorithm.update(knownInput), expectedOutput);

Property-based fuzz tests instead assert *invariants* that hold across the
entire input domain:

.. code-block:: cpp

    // Fuzz test: any valid input must satisfy these properties
    void fuzzAlgorithm(int32_t x, int32_t y) {
        auto result = algorithm.update(x, y);
        EXPECT_GE(result.count, 0);           // never negative
        EXPECT_LE(result.count, x + y);       // bounded by inputs
    }
    FUZZ_TEST(MySuite, fuzzAlgorithm)
        .WithDomains(fuzztest::InRange(0, 1000),
                     fuzztest::InRange(0, 1000));

When built without fuzzing instrumentation (the default), each ``FUZZ_TEST``
runs as a normal parameterized GTest with a fixed set of seed inputs -- so it
participates in regular CI without any special build flags. When built with
``FUZZTEST_FUZZING_MODE=ON``, the same test becomes a coverage-guided fuzzer
that mutates inputs to maximize code coverage.


Creating a Fuzz Test
--------------------

File layout
~~~~~~~~~~~

Place fuzz tests next to the existing unit tests for a module. The naming
convention is ``test_<moduleName>_fuzz.cpp``:

.. code-block:: none

    src/fswAlgorithms/<category>/<moduleName>/
        tests/
            CMakeLists.txt
            test_<moduleName>.cpp          # regular GTest
            test_<moduleName>_fuzz.cpp     # fuzz tests

Writing the test file
~~~~~~~~~~~~~~~~~~~~~

A fuzz test file has three parts:

1. **Includes** -- the algorithm header, GTest, and FuzzTest:

   .. code-block:: cpp

      #include "../myAlgorithm.h"
      #include "gtest/gtest.h"
      #include <fuzztest/fuzztest.h>

2. **Test functions** -- each function takes fuzzed parameters and asserts
   invariants. The function must not return a value; use ``EXPECT_*`` /
   ``ASSERT_*`` macros to check properties:

   .. code-block:: cpp

      void fuzzMyAlgorithm(int32_t inputA, double inputB) {
          MyAlgorithm algo;
          auto result = algo.compute(inputA, inputB);

          // Assert invariants -- properties that must hold for ALL inputs
          EXPECT_GE(result.value, 0);
          EXPECT_NO_THROW(algo.reset());
      }

3. **Registration** -- the ``FUZZ_TEST`` macro registers the function and
   specifies input domains:

   .. code-block:: cpp

      FUZZ_TEST(MyAlgorithmFuzz, fuzzMyAlgorithm)
          .WithDomains(fuzztest::InRange(0, 1000),      // inputA
                       fuzztest::InRange(-1.0, 1.0));    // inputB

   Common domain combinators:

   - ``fuzztest::InRange(lo, hi)`` -- uniformly sample an integer or float range
   - ``fuzztest::OneOf(domain1, domain2, ...)`` -- pick from several sub-domains
   - ``fuzztest::Arbitrary<T>()`` -- any value of type ``T``
   - ``fuzztest::VectorOf(domain).WithMaxSize(n)`` -- variable-length containers

   See the `FuzzTest domain reference <https://github.com/google/fuzztest/blob/main/doc/domains-reference.md>`__
   for the full list.

CMake integration
~~~~~~~~~~~~~~~~~

In the module's ``tests/CMakeLists.txt``, add a conditional fuzz target:

.. code-block:: cmake

   if(XMERA_ENABLE_FUZZTESTS)
     fuzztest_setup_fuzzing_flags()

     add_executable(test_myModule_fuzz
       ../myAlgorithm.cpp
       test_myModule_fuzz.cpp
     )

     target_link_libraries(test_myModule_fuzz PRIVATE
       Eigen3::Eigen
       fuzztest::fuzztest
       fuzztest::fuzztest_gtest_main
     )

     gtest_discover_tests(test_myModule_fuzz
       PROPERTIES
         LABELS "fuzz\;fuzz-smoke"
     )
   endif()

Key points:

- Guard everything with ``if(XMERA_ENABLE_FUZZTESTS)`` so the default build is
  unaffected.
- Call ``fuzztest_setup_fuzzing_flags()`` before ``add_executable`` to configure
  the necessary compiler instrumentation.
- Link against ``fuzztest::fuzztest`` and ``fuzztest::fuzztest_gtest_main``
  (not ``GTest::gtest_main``).
- Apply both the ``fuzz`` and ``fuzz-smoke`` CTest labels so the target is
  picked up by both smoke and long-running fuzz workflows.

Writing good invariants
~~~~~~~~~~~~~~~~~~~~~~~

The hardest part of fuzz testing is choosing what to assert. Some strategies:

- **No crash** -- the simplest invariant. If the function segfaults or throws an
  unhandled exception on *any* input, that is a bug.
- **Output bounds** -- result values stay within a known range
  (``EXPECT_GE(result, 0)``).
- **Conservation** -- output does not exceed a known function of the inputs
  (e.g., merged pixel count <= sum of input pixel counts).
- **Idempotence** -- calling the function twice with the same input gives the
  same result.
- **Round-trip** -- encode then decode returns the original value.
- **Reference oracle** -- compare a fast implementation against a known-correct
  slow implementation.


Running Fuzz Tests
------------------

There are three ways to run fuzz tests, depending on what you need.

As regular unit tests (smoke test)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Configure with the ``fuzz-smoke-test`` preset and run via CTest. Each
``FUZZ_TEST`` executes a small fixed set of seed inputs as a normal GTest --
no coverage-guided fuzzing, just a quick sanity check:

.. code-block:: bash

   cd src
   cmake --preset fuzz-smoke-test
   cmake --build ../build --parallel
   cd ../build && ctest -C Release -L fuzz-smoke

This is what PR CI runs. It finishes in seconds.

As a coverage-guided fuzzer (local)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Configure with the ``fuzz-test`` preset (enables ``FUZZTEST_FUZZING_MODE``),
then run a specific binary with ``--fuzz_for``:

.. code-block:: bash

   cd src
   cmake --preset fuzz-test
   cmake --build ../build --parallel
   ./build/path/to/test_myModule_fuzz --fuzz_for=120s

The fuzzer will mutate inputs for the specified duration, printing new coverage
discoveries to the terminal. If it finds a crash, it prints the reproducer
input and exits with a non-zero status.

To run all fuzz targets for a fixed duration, use the helper script:

.. code-block:: bash

   .github/scripts/run_long_fuzzers.sh ./build ./fuzz-logs --fuzz-for 120s

With a persistent corpus
~~~~~~~~~~~~~~~~~~~~~~~~~

A *corpus* is a set of inputs that the fuzzer has previously found interesting
(i.e., they exercise distinct code paths). Persisting the corpus between runs
lets the fuzzer pick up where it left off instead of starting from scratch.

To save and reuse a corpus, set the ``FUZZTEST_TESTSUITE_OUT_DIR`` and
``FUZZTEST_TESTSUITE_IN_DIR`` environment variables:

.. code-block:: bash

   # First run -- seed the corpus
   mkdir -p /tmp/fuzz-corpus
   FUZZTEST_TESTSUITE_OUT_DIR=/tmp/fuzz-corpus \
   FUZZTEST_TESTSUITE_IN_DIR=/tmp/fuzz-corpus \
   ./build/path/to/test_myModule_fuzz --fuzz_for=60s

   # Subsequent runs -- the fuzzer loads previous findings and continues
   FUZZTEST_TESTSUITE_OUT_DIR=/tmp/fuzz-corpus \
   FUZZTEST_TESTSUITE_IN_DIR=/tmp/fuzz-corpus \
   ./build/path/to/test_myModule_fuzz --fuzz_for=60s

Corpus files are written **incrementally during fuzzing** -- they survive
crashes and early termination. This is in contrast to ``--corpus_database``,
which only writes the corpus after a clean exit.

The ``run_long_fuzzers.sh`` script handles corpus management automatically when
given the ``--corpus`` flag:

.. code-block:: bash

   .github/scripts/run_long_fuzzers.sh ./build ./fuzz-logs \
       --fuzz-for 120s --corpus .fuzztest_corpus

This creates per-binary subdirectories under ``.fuzztest_corpus/`` (e.g.,
``.fuzztest_corpus/test_regionsOfInterest_fuzz/``) and sets the environment
variables for each binary.


Nightly CI Fuzzing
------------------

The ``nightly-long-tests.yml`` workflow runs every night and:

1. Restores the corpus cache from prior runs.
2. Configures and builds with the ``fuzz-test`` preset.
3. Runs ``run_long_fuzzers.sh`` with a 120-second (or configurable) duration
   per target.
4. Saves the updated corpus back to the GitHub Actions cache.
5. Uploads logs and the corpus as workflow artifacts.

The corpus accumulates over time. Each nightly run starts from the previous
night's corpus, so coverage improves monotonically.

To trigger a manual nightly run with a custom duration, use the workflow
dispatch from the Actions tab and set the ``fuzz_duration`` input (e.g.,
``10m``).

Reproducing a Failure
---------------------

When the fuzzer finds a crash, it prints the failing input to the console. To
reproduce it:

1. Copy the ``FUZZ_TEST_*`` reproducer line from the log.
2. Set the ``FUZZ_TEST_MINIMIZER_INPUT`` environment variable as shown in the
   log and re-run the binary. The fuzzer will minimize the input to the
   smallest case that still triggers the failure.
3. Add the minimal reproducing input as a regular GTest case so the fix is
   permanently covered by CI.
