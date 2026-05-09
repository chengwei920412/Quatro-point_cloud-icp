# Quatro PCL Compatibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Quatro registration class behave like a real `pcl::Registration` derivative (`align()` callable, `getFinalTransformation()` working) while building cleanly across PCL 1.10 – 1.14 and preserving the existing `computeTransformation(Eigen::Matrix4d&)` API.

**Architecture:** Refactor `include/quatro.hpp` in place. Move PCL utility code (`voxelize`) into a new `include/quatro/pcl_compat.hpp`. Extract the existing algorithm core into a private helper, then expose two thin entry points: the PCL virtual `computeTransformation(PointCloudSource&, const Matrix4&)` (new, enables `align()`) and the legacy `computeTransformation(Eigen::Matrix4d&)` (kept for back-compat). Both populate `final_transformation_` so `getFinalTransformation()` works after either call. The algorithm's internal math stays in `double`; only public boundary types use the `Scalar` template parameter.

**Tech Stack:** C++17, PCL 1.10+, Eigen 3.2+, Boost, OpenMP, catkin (ROS Noetic verified).

**Reference design:** `docs/superpowers/specs/2026-05-09-quatro-pcl-compatibility-design.md`

**Build / verify environment:** Every build and runtime check goes through the Docker harness committed at `docker/`:

| Command | What it does |
|---------|--------------|
| `./docker/run.sh build-image` | Build the `quatro-dev:noetic` image (one-time, ~1 min after base pulled). |
| `./docker/run.sh build-pkg`  | `catkin build quatro` inside the container; uses named volumes for incremental builds across runs. |
| `./docker/run.sh test`       | `catkin build quatro` + headless `roslaunch quatro_headless.launch` with a timeout. Pins `OMP_NUM_THREADS=1`. Prints `[QUATRO_OUTPUT] m00 m01 ... m33` on a single grep-able line. |
| `./docker/regress.sh`        | Calls `./docker/run.sh test` once, parses the matrix, validates against tolerance bounds. Prints `REGRESSION OK` or `REGRESSION FAILED (N check(s) violated)`. Exit 0 on pass, non-zero on fail. |
| `./docker/run.sh shell`      | Interactive shell inside the container for debugging. |
| `./docker/run.sh clean`      | Remove the named volumes (forces a clean build next time). |

**Why tolerance-based:** Quatro is a randomized algorithm (FLANN's randomized kd-tree splits, PMC heuristic max-clique). Outputs vary across runs even at the same git revision; an exact-string `diff` against a frozen baseline can't pass even at HEAD. The README acknowledges this ("multi-thread issues"). The regress.sh bounds were derived from 12+ pre-refactor runs and are deliberately wide enough to absorb the algorithm's natural variance, but tight enough to catch a refactor that broke the core pipeline (identity output, wrong sign, 90-degree-off solution, NaN, etc).

Every task that changes algorithm-relevant code verifies behavior with:

```bash
./docker/regress.sh 2>&1 | tail -20
```

Last line should be `REGRESSION OK`. Anything else = fail. For tasks that only change build files (e.g. CMake), `./docker/run.sh build-pkg` is enough.

---

## File Structure

| Path | Status | Responsibility |
|------|--------|----------------|
| `include/quatro/pcl_compat.hpp` | NEW | PCL smart-pointer-version-agnostic utilities. Today: `voxelize<T>(...)` overloads. |
| `include/quatro.hpp` | MODIFIED | The `Quatro<PointSource, PointTarget, Scalar>` class. PCL `Registration` derivative. Header hygiene cleaned up. Algorithm core extracted into a private helper. Real PCL virtual override added. |
| `examples/run_global_registration.cpp` | MODIFIED | `voxelize` callers re-qualified to `quatro::voxelize<T>(...)`. Switched to `quatro.align(...)` + `getFinalTransformation()` for the registration call. Legacy call documented in a comment. |
| `CMakeLists.txt` | MODIFIED | PCL minimum version bumped 1.8 → 1.10. `${PCL_LIBRARY_DIRS}` typo fixed to `${PCL_LIBRARIES}`. |

Already in place from the harness commits:
- `docker/Dockerfile`, `docker/run.sh` (pins `OMP_NUM_THREADS=1`), `docker/regress.sh` (tolerance-based regression check)
- `launch/quatro_headless.launch`
- `[QUATRO_OUTPUT]` print in `examples/run_global_registration.cpp`

Files explicitly NOT touched in this plan: `include/conversion.hpp`, `include/fpfh_manager.hpp`, `include/imageProjection.hpp`, `include/patchwork.hpp`, `include/utility.h`, `include/teaser/**`, `include/teaser_utils/**`, `src/**`, `package.xml`, `3rdparty/**`, `config/**`, `launch/quatro.launch`, `rviz/**`, `msg/**`, `materials/**`.

---

## Task 1: Sanity-check the Docker harness against baseline

**Why:** Confirms the harness still works on this branch before any code changes. If this fails, the rest of the plan can't be verified.

**Steps:**

- [ ] **Step 1.1: Make sure the image is built**

```bash
./docker/run.sh build-image
```

If the image already exists, the Docker build is a no-op (cache hit). Output ends with `naming to docker.io/library/quatro-dev:noetic`.

- [ ] **Step 1.2: Run the regression check**

```bash
./docker/regress.sh 2>&1 | tail -20
```

Expected: last line `REGRESSION OK`.

Expected: empty diff and `REGRESSION OK`. The exact baseline content is:
```
[QUATRO_OUTPUT] 0.8856 0.4644 0 -8.767 -0.4644 0.8856 0 -0.7905 0 0 1 1.132 0 0 0 1
```

If diff is non-empty, **stop**: either the harness is broken or the codebase has drifted from when the baseline was captured. Investigate before continuing.

- [ ] **Step 1.3: No commit needed**

This task is read-only.

---

## Task 2: Create `include/quatro/pcl_compat.hpp` with `voxelize` moved

**Why:** The `voxelize<T>` overloads in `quatro.hpp` use `boost::shared_ptr<pcl::PointCloud<T>>` in their signatures, which breaks when callers pass `pcl::PointCloud<T>::Ptr` (= `std::shared_ptr<...>`) on PCL 1.12+. They also use a `static pcl::VoxelGrid<T>` instance which retains state across calls.

**Files:**
- Create: `include/quatro/pcl_compat.hpp`

**Steps:**

- [ ] **Step 2.1: Create the new directory**

```bash
mkdir -p include/quatro
```

- [ ] **Step 2.2: Write `include/quatro/pcl_compat.hpp`**

```cpp
#ifndef QUATRO_PCL_COMPAT_HPP
#define QUATRO_PCL_COMPAT_HPP

// PCL smart-pointer-version-agnostic utilities for Quatro.
//
// PCL 1.10 (Ubuntu 20.04) uses boost::shared_ptr; PCL 1.11+ uses
// pcl::shared_ptr (= std::shared_ptr). Function signatures in this
// header MUST take typename pcl::PointCloud<T>::Ptr / ConstPtr and
// never boost::shared_ptr<pcl::PointCloud<T>> directly, so they
// remain correct on every supported PCL version.

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

namespace quatro {

template <typename PointT>
void voxelize(const typename pcl::PointCloud<PointT>::ConstPtr& src,
              const typename pcl::PointCloud<PointT>::Ptr& dst,
              double voxel_size) {
    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(src);
    filter.setLeafSize(static_cast<float>(voxel_size),
                       static_cast<float>(voxel_size),
                       static_cast<float>(voxel_size));
    filter.filter(*dst);
}

template <typename PointT>
void voxelize(const pcl::PointCloud<PointT>& src,
              const typename pcl::PointCloud<PointT>::Ptr& dst,
              double voxel_size) {
    auto src_ptr = typename pcl::PointCloud<PointT>::Ptr(
        new pcl::PointCloud<PointT>(src));
    voxelize<PointT>(src_ptr, dst, voxel_size);
}

}  // namespace quatro

#endif  // QUATRO_PCL_COMPAT_HPP
```

- [ ] **Step 2.3: Verify the catkin build still works (no regression — nothing includes it yet)**

```bash
./docker/run.sh build-pkg 2>&1 | tail -10
```

Expected: build succeeds. Nothing in the project includes the new file yet, so behavior is unchanged.

- [ ] **Step 2.4: Commit**

```bash
git add include/quatro/pcl_compat.hpp
git commit -m "Add include/quatro/pcl_compat.hpp with PCL-version-agnostic voxelize"
```

---

## Task 3: Header hygiene in `include/quatro.hpp` and qualify the example's `voxelize` callers

**Why:** `quatro.hpp` has `using namespace std;` and `using namespace pcl;` (forbidden in headers — pollutes every translation unit), unused ROS/system includes, and an inline `voxelize` we just moved into `namespace quatro`. Removing the inline `voxelize` will break the example's two unqualified call sites (lines 206-207 of `run_global_registration.cpp`), so this task updates them in the same commit to keep the build green.

**Files:**
- Modify: `include/quatro.hpp` (lines 10-11, 45-46, 49-68)
- Modify: `examples/run_global_registration.cpp` (lines 206-207)

**Steps:**

- [ ] **Step 3.1: Remove unused system/ROS includes**

In `include/quatro.hpp`, delete these two lines (currently lines 10-11):

```cpp
#include <unistd.h>
#include <geometry_msgs/Pose.h>
```

- [ ] **Step 3.2: Replace inline `voxelize` definitions with the new header**

Delete the two `voxelize` template definitions (currently lines 49-68 of `quatro.hpp` — both overloads, ending with `voxel_filter.filter(*dstPtr);  }`).

Add this include near the other Quatro-internal includes (just below `#include "conversion.hpp"`):

```cpp
#include "quatro/pcl_compat.hpp"
```

Leave the existing `#include <pcl/filters/voxel_grid.h>` in place; other code in the file (and in includes) may rely on it.

- [ ] **Step 3.3: Remove `using namespace` directives**

Delete these two lines (currently lines 45-46):

```cpp
using namespace std;
using namespace pcl;
```

- [ ] **Step 3.4: Update `voxelize` callers in the example**

In `examples/run_global_registration.cpp`, find lines 206-207:

```cpp
voxelize(srcValidSegments, srcFeat, voxel_size);
voxelize(tgtValidSegments, tgtFeat, voxel_size);
```

Replace with:

```cpp
quatro::voxelize<PointType>(srcValidSegments, srcFeat, voxel_size);
quatro::voxelize<PointType>(tgtValidSegments, tgtFeat, voxel_size);
```

(The explicit template arg avoids deduction ambiguity between the two overloads when the first argument is a `Ptr`.)

- [ ] **Step 3.5: Restore `std::` and `pcl::` qualifications inside the header**

Build first, then qualify each unqualified symbol the compiler complains about. Common ones to qualify in `quatro.hpp` (do all proactively):

- `cout` → `std::cout`
- `endl` → `std::endl`
- `cerr` → `std::cerr`
- `string` → `std::string`
- `vector` → `std::vector`
- `pair` → `std::pair`
- `make_pair` → `std::make_pair`
- `sort` → `std::sort`
- `ostream_iterator` → `std::ostream_iterator`
- `ofstream` → `std::ofstream`
- `setprecision` → `std::setprecision`
- `numeric_limits` → `std::numeric_limits`
- `invalid_argument` → `std::invalid_argument`
- `Registration<...>` → `pcl::Registration<...>`
- `PointCloud<...>` → `pcl::PointCloud<...>` (only where it appears bare; `PointCloudSource` / `PointCloudTarget` typedefs are not affected)
- `PointXYZ` → `pcl::PointXYZ`
- `transformPointCloud` → `pcl::transformPointCloud`
- `VoxelGrid` → `pcl::VoxelGrid` (if any references survived removing the inline `voxelize`)

`PointType` is a project-wide typedef defined in `include/utility.h` — leave it as-is.

- [ ] **Step 3.6: Build until clean**

```bash
./docker/run.sh build-pkg 2>&1 | tail -40
```

Expected: clean build. If it fails, the error names the unqualified symbol — qualify it and rebuild. Repeat until clean.

- [ ] **Step 3.7: Run the regression check**

```bash
./docker/regress.sh 2>&1 | tail -20
```

Expected: last line `REGRESSION OK`.

Expected: empty diff and `REGRESSION OK`. If not, the qualification touched something that changed behavior — investigate.

- [ ] **Step 3.8: Commit**

```bash
git add include/quatro.hpp examples/run_global_registration.cpp
git commit -m "Clean up quatro.hpp: remove using-namespace, drop unused ROS/system includes, use pcl_compat.hpp; qualify voxelize callers"
```

---

## Task 4: Extract algorithm core into a private helper

**Why:** Two public entry points (the PCL virtual override in Task 5, and the legacy method in Task 6) need to share the same algorithm body. Extracting it now lets both callers reuse it without duplication.

**Files:**
- Modify: `include/quatro.hpp` — extract method body, no behavior change

**Steps:**

- [ ] **Step 4.1: Add a private helper declaration**

In `include/quatro.hpp`, find the existing `private:` section near the bottom (currently around line 1059). Just below `private:`, add:

```cpp
    /** \brief Algorithm core. Computes the registration transform in
     *  double precision and writes it into `output`. Sets `solution_.valid`.
     *  Does NOT touch `final_transformation_` or `converged_` — the public
     *  entry points are responsible for that.
     */
    void computeQuatroTransformation_(Eigen::Matrix4d& output);
```

(Since `Quatro` is a class template, both the declaration and definition live in this header.)

- [ ] **Step 4.2: Move the existing body into the helper**

Find the existing definition (currently `void computeTransformation(Eigen::Matrix4d &output) {` near line 769). The body starting at the next line and ending with the closing `}` of that method is the algorithm core.

Move that body verbatim into a new out-of-class member-template definition, placed at the end of the file just before `#endif //QUATRO_H`:

```cpp
template <typename PointSource, typename PointTarget, typename Scalar>
void Quatro<PointSource, PointTarget, Scalar>::computeQuatroTransformation_(
        Eigen::Matrix4d& output) {
    // ... the existing body, verbatim ...
}
```

- [ ] **Step 4.3: Replace the original `computeTransformation(Eigen::Matrix4d&)` body with a delegating wrapper**

Where the original body used to be, leave a thin wrapper:

```cpp
void computeTransformation(Eigen::Matrix4d& output) {
    computeQuatroTransformation_(output);
}
```

(Task 6 will enrich this wrapper to also populate `final_transformation_`.)

- [ ] **Step 4.4: Build**

```bash
./docker/run.sh build-pkg 2>&1 | tail -20
```

Expected: clean build. Common errors:
- "redefinition of `computeTransformation`" → the previously empty stub at line ~767 (`void computeTransformation(PointCloudSource &output, const Matrix4 &guess) override {};`) is fine and should be left alone; verify the rename only touched the `Eigen::Matrix4d&` overload.

- [ ] **Step 4.5: Regression check**

```bash
./docker/regress.sh 2>&1 | tail -20
```

Expected: last line `REGRESSION OK`. Refactoring without behavior change is the criterion.

- [ ] **Step 4.6: Commit**

```bash
git add include/quatro.hpp
git commit -m "Extract Quatro algorithm core into computeQuatroTransformation_ helper"
```

---

## Task 5: Implement the PCL `Registration` virtual override (enables `align()`)

**Why:** `pcl::Registration::align(output)` calls `computeTransformation(PointCloudSource& output, const Matrix4& guess)` internally. The current empty-body override (`{};` at line ~767) means `align()` is a no-op. We replace it with a real implementation that delegates to our core helper.

**Files:**
- Modify: `include/quatro.hpp`

**Steps:**

- [ ] **Step 5.1: Add `using` to expose protected base members and unhide overloads**

In `include/quatro.hpp`, near the other `using Registration<...>::...` lines (currently around lines 80-104), add (only if not already present — `grep` first):

```cpp
    using pcl::Registration<PointSource, PointTarget, Scalar>::final_transformation_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::converged_;
    using pcl::Registration<PointSource, PointTarget, Scalar>::computeTransformation;
```

The `computeTransformation` `using` keeps the inherited overload visible alongside our `Eigen::Matrix4d&` legacy overload, so callers can resolve either signature.

- [ ] **Step 5.2: Add an include for `PCL_WARN`**

Near the top of `quatro.hpp`, with the other PCL includes, add:

```cpp
#include <pcl/console/print.h>
```

- [ ] **Step 5.3: Replace the empty `computeTransformation(PointCloudSource&, const Matrix4&)` override**

Find:
```cpp
void computeTransformation(PointCloudSource &output, const Matrix4 &guess) override {};
```

Replace with:

```cpp
void computeTransformation(PointCloudSource& output, const Matrix4& guess) override {
    if (!guess.isIdentity()) {
        PCL_WARN("[%s] Quatro is a global registration method and ignores "
                 "the `guess` argument passed via align(...). Use "
                 "setPreEstaimatedRyRx() to inject a rotational prior.\n",
                 this->getClassName().c_str());
    }

    Eigen::Matrix4d transform_d = Eigen::Matrix4d::Identity();
    computeQuatroTransformation_(transform_d);

    final_transformation_ = transform_d.template cast<Scalar>();
    converged_ = solution_.valid;

    if (solution_.valid) {
        pcl::transformPointCloud(*input_, output, final_transformation_);
    } else {
        output = *input_;
    }
}
```

- [ ] **Step 5.4: Build**

```bash
./docker/run.sh build-pkg 2>&1 | tail -30
```

Expected: clean build. Common errors and fixes:
- "`final_transformation_` was not declared" → Step 5.1 missed; add the `using` line.
- "`converged_` was not declared" → same fix.
- "`PCL_WARN` was not declared" → Step 5.2 missed; add the include.

- [ ] **Step 5.5: Smoke-test the new `align()` API in a temporary block**

Append this temporary block at the end of `examples/run_global_registration.cpp::main` — just before the `while (ros::ok())` loop:

```cpp
{
    Quatro<PointType, PointType> quatro2;
    quatro2.reset(params);
    quatro2.setInputSource(srcMatched);
    quatro2.setInputTarget(tgtMatched);
    pcl::PointCloud<PointType> aligned_via_align;
    quatro2.align(aligned_via_align);
    Eigen::Matrix4d via_align =
        quatro2.getFinalTransformation().template cast<double>();
    std::cout << "[ALIGN-API]";
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            std::cout << " " << via_align(r, c);
        }
    }
    std::cout << std::endl;
}
```

Run:

```bash
./docker/run.sh test 2>&1 | grep -E '^\[QUATRO_OUTPUT\]|^\[ALIGN-API\]'
```

Both lines must show identical numerical values (modulo the prefix).

- [ ] **Step 5.6: Revert the temporary smoke-test block**

```bash
git checkout -- examples/run_global_registration.cpp
```

(The permanent example update happens in Task 7.)

- [ ] **Step 5.7: Regression check (legacy API still produces baseline)**

```bash
./docker/regress.sh 2>&1 | tail -20
```

Expected: last line `REGRESSION OK`.

- [ ] **Step 5.8: Commit**

```bash
git add include/quatro.hpp
git commit -m "Implement pcl::Registration virtual computeTransformation override (enables align())"
```

---

## Task 6: Wire the legacy method to also set `final_transformation_`

**Why:** Spec section 4.2.3 — both API surfaces should leave the object in the same observable state, so `getFinalTransformation()` works regardless of which entry point the user calls. Costless improvement and a strict superset of the prior behavior.

**Files:**
- Modify: `include/quatro.hpp` (the legacy wrapper introduced in Task 4.3)

**Steps:**

- [ ] **Step 6.1: Enrich the legacy wrapper**

Find the wrapper from Task 4.3:

```cpp
void computeTransformation(Eigen::Matrix4d& output) {
    computeQuatroTransformation_(output);
}
```

Replace with:

```cpp
void computeTransformation(Eigen::Matrix4d& output) {
    computeQuatroTransformation_(output);
    // Mirror the state that align()-style callers expect, so
    // getFinalTransformation() / hasConverged() work after this entry point too.
    final_transformation_ = output.template cast<Scalar>();
    converged_ = solution_.valid;
}
```

- [ ] **Step 6.2: Build**

```bash
./docker/run.sh build-pkg 2>&1 | tail -20
```

Expected: clean build.

- [ ] **Step 6.3: Regression check**

```bash
./docker/regress.sh 2>&1 | tail -20
```

Expected: last line `REGRESSION OK`.

- [ ] **Step 6.4: Commit**

```bash
git add include/quatro.hpp
git commit -m "Legacy computeTransformation(Eigen::Matrix4d&) also sets final_transformation_"
```

---

## Task 7: Switch the example to the `align()` API

**Why:** Spec section 4.4 — show the PCL-idiomatic flow as the recommended usage. Keep the legacy call path documented in a comment so readers see both options.

**Files:**
- Modify: `examples/run_global_registration.cpp`

**Steps:**

- [ ] **Step 7.1: Replace the legacy call with `align()`**

In `examples/run_global_registration.cpp`, find the block (currently lines 242-246):

```cpp
std::chrono::system_clock::time_point before_optim = std::chrono::system_clock::now();
quatro.setInputSource(srcMatched);
quatro.setInputTarget(tgtMatched);
Eigen::Matrix4d output;
quatro.computeTransformation(output);
```

Replace with:

```cpp
std::chrono::system_clock::time_point before_optim = std::chrono::system_clock::now();
quatro.setInputSource(srcMatched);
quatro.setInputTarget(tgtMatched);

// PCL-idiomatic flow (recommended). The aligned cloud parameter is
// required by pcl::Registration::align(), but for this example we only
// need the resulting transform; we use it later via pcl::transformPointCloud
// against the *raw* source (srcRaw) below.
pcl::PointCloud<PointType> aligned_matched;
quatro.align(aligned_matched);
Eigen::Matrix4d output =
        quatro.getFinalTransformation().template cast<double>();

// Legacy API (kept working for back-compat):
//   Eigen::Matrix4d output;
//   quatro.computeTransformation(output);
```

- [ ] **Step 7.2: Build**

```bash
./docker/run.sh build-pkg 2>&1 | tail -20
```

Expected: clean build.

- [ ] **Step 7.3: Regression check**

```bash
./docker/regress.sh 2>&1 | tail -20
```

Expected: last line `REGRESSION OK`. The example now exercises the `align()` path; the printed matrix MUST equal the baseline (which was captured via the legacy path).

- [ ] **Step 7.4: Commit**

```bash
git add examples/run_global_registration.cpp
git commit -m "Switch run_global_registration example to align() API; document legacy path in comment"
```

---

## Task 8: Fix `CMakeLists.txt`

**Why:** Spec section 4.3 — bump the PCL minimum to reflect what we actually support, and fix the `${PCL_LIBRARY_DIRS}` typo (a directory string, not the library list).

**Files:**
- Modify: `CMakeLists.txt`

**Steps:**

- [ ] **Step 8.1: Bump PCL minimum version**

In `CMakeLists.txt`, find:
```cmake
find_package(PCL 1.8 REQUIRED)
```

Replace with:
```cmake
find_package(PCL 1.10 REQUIRED)
```

- [ ] **Step 8.2: Fix the link line typo**

Find:
```cmake
target_link_libraries(run_example
        PUBLIC
        ${PCL_LIBRARY_DIRS}
        ${catkin_LIBRARIES}
        stdc++fs
        pmc::pmc
        )
```

Replace with:
```cmake
target_link_libraries(run_example
        PUBLIC
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
        stdc++fs
        pmc::pmc
        )
```

- [ ] **Step 8.3: Clean rebuild**

```bash
./docker/run.sh clean
./docker/run.sh build-pkg 2>&1 | tail -30
```

Expected: clean build from scratch.

- [ ] **Step 8.4: Regression check**

```bash
./docker/regress.sh 2>&1 | tail -20
```

Expected: last line `REGRESSION OK`.

- [ ] **Step 8.5: Commit**

```bash
git add CMakeLists.txt
git commit -m "Bump PCL minimum to 1.10; fix \${PCL_LIBRARY_DIRS} -> \${PCL_LIBRARIES} typo"
```

---

## Task 9: Final verification

**Why:** End-to-end confidence sweep before declaring done.

**Steps:**

- [ ] **Step 9.1: Static checks on `include/quatro.hpp`**

```bash
grep -n "using namespace" include/quatro.hpp        # expect: no output
grep -n "boost::shared_ptr" include/quatro.hpp      # expect: no output
grep -n "geometry_msgs" include/quatro.hpp          # expect: no output
grep -n "<unistd.h>" include/quatro.hpp             # expect: no output
```

If any of these prints lines, fix and amend.

- [ ] **Step 9.2: Both API surfaces work in a single run**

Temporarily extend `examples/run_global_registration.cpp` to also call the legacy API alongside the existing `align()` flow. Add this block just before the `while (ros::ok())` loop:

```cpp
{
    Quatro<PointType, PointType> quatro_legacy;
    quatro_legacy.reset(params);
    quatro_legacy.setInputSource(srcMatched);
    quatro_legacy.setInputTarget(tgtMatched);
    Eigen::Matrix4d legacy_output;
    quatro_legacy.computeTransformation(legacy_output);
    std::cout << "[LEGACY-API]";
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            std::cout << " " << legacy_output(r, c);
        }
    }
    std::cout << std::endl;
    Eigen::Matrix4d legacy_via_getter =
        quatro_legacy.getFinalTransformation().template cast<double>();
    std::cout << "[LEGACY-GETTER]";
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            std::cout << " " << legacy_via_getter(r, c);
        }
    }
    std::cout << std::endl;
}
```

Run:

```bash
./docker/run.sh test 2>&1 | grep -E '^\[QUATRO_OUTPUT\]|^\[LEGACY-API\]|^\[LEGACY-GETTER\]'
```

Verify all three lines have the same numerical values (modulo prefix). The `[QUATRO_OUTPUT]` line uses the `align()` path (Task 7); `[LEGACY-API]` uses `computeTransformation(Eigen::Matrix4d&)`; `[LEGACY-GETTER]` confirms `getFinalTransformation()` is now wired by Task 6.

Then revert: `git checkout -- examples/run_global_registration.cpp`.

- [ ] **Step 9.3: Final clean build + regression**

```bash
./docker/run.sh clean
./docker/regress.sh 2>&1 | tail -20
```

Expected: last line `REGRESSION OK`. This is the authoritative end-to-end check.

- [ ] **Step 9.4: Inspect the commit log**

```bash
git log --oneline main..HEAD 2>/dev/null || git log --oneline -8
```

Expected: 7 refactor commits (Tasks 2–8), each with a clear message. No "WIP" or "fixup".

If the implementation is on a worktree branch, the branch is now ready to merge. If on `main` directly, no further git work needed before pushing.

---

## Out of scope reminder

The following were explicitly excluded by the spec and remain follow-up work:
- ROS-free build option (split algorithm core out of the catkin package)
- Install rules + CMake config files for `find_package(quatro)`
- Python bindings
- Removing ROS dependencies from `imageProjection.hpp`, `patchwork.hpp`, `fpfh_manager.hpp`

These are future iterations. Do not pull them into this plan.
