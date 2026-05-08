# Quatro PCL Compatibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Quatro registration class behave like a real `pcl::Registration` derivative (`align()` callable, `getFinalTransformation()` working) while building cleanly across PCL 1.10 – 1.14 and preserving the existing `computeTransformation(Eigen::Matrix4d&)` API.

**Architecture:** Refactor `include/quatro.hpp` in place. Move PCL utility code (`voxelize`) into a new `include/quatro/pcl_compat.hpp`. Extract the existing algorithm core into a private helper, then expose two thin entry points: the PCL virtual `computeTransformation(PointCloudSource&, const Matrix4&)` (new, enables `align()`) and the legacy `computeTransformation(Eigen::Matrix4d&)` (kept for back-compat). Both populate `final_transformation_` so `getFinalTransformation()` works after either call. The algorithm's internal math stays in `double`; only public boundary types use the `Scalar` template parameter.

**Tech Stack:** C++17, PCL 1.10+, Eigen 3.2+, Boost, OpenMP, catkin (ROS Melodic / Noetic compatible).

**Reference design:** `docs/superpowers/specs/2026-05-09-quatro-pcl-compatibility-design.md`

**Build environment assumption:** ROS catkin workspace (Ubuntu 18.04 / 20.04 / 22.04 with corresponding ROS distro). All build/run verifications below run inside `catkin build quatro` and `roslaunch quatro quatro.launch`. If the executing engineer is on macOS or another non-ROS host, the verification steps must be performed on a Linux host with ROS installed (a Docker container is acceptable).

---

## File Structure

| Path | Status | Responsibility |
|------|--------|----------------|
| `include/quatro/pcl_compat.hpp` | NEW | PCL smart-pointer-version-agnostic utilities. Today: `voxelize<T>(...)`. Designed to grow as more PCL utilities are extracted. |
| `include/quatro.hpp` | MODIFIED | The `Quatro<PointSource, PointTarget, Scalar>` class. PCL `Registration` derivative. Header hygiene cleaned up. Algorithm core extracted into a private helper. |
| `examples/run_global_registration.cpp` | MODIFIED | Demo. Switched to `quatro.align(...)` + `getFinalTransformation()` (the PCL-idiomatic flow). Legacy call documented in a comment. |
| `CMakeLists.txt` | MODIFIED | PCL minimum version bumped 1.8 → 1.10. `${PCL_LIBRARY_DIRS}` typo fixed to `${PCL_LIBRARIES}`. |
| `materials/baseline_transform.txt` | NEW (transient) | Baseline reference output captured before refactor; used as the regression check oracle. Removed at end of plan. |

Files explicitly NOT touched in this plan: `include/conversion.hpp`, `include/fpfh_manager.hpp`, `include/imageProjection.hpp`, `include/patchwork.hpp`, `include/utility.h`, `include/teaser/**`, `include/teaser_utils/**`, `src/**`, `package.xml`, `3rdparty/**`, `config/**`, `launch/**`, `rviz/**`, `msg/**`.

---

## Task 1: Capture baseline behavior (regression oracle)

**Why this task exists:** The codebase has no unit tests. Our verification model is "the example produces the same numerical output before and after." We need to record the "before" output now so later tasks can check against it.

**Files:**
- Create: `materials/baseline_transform.txt` (transient, removed in Task 8)

**Steps:**

- [ ] **Step 1.1: Build the repo at the current commit**

```bash
cd ~/catkin_ws
catkin build quatro
```

Expected: build succeeds. If it does not, capture the failure — the `boost::shared_ptr` issues from the spec may already be biting. Note the failure and proceed; the refactor will fix it. (In that case, skip Step 1.2 and Step 1.3 — record "baseline did not build on PCL <version>" in `materials/baseline_transform.txt` and use *post-refactor build success* as the verification instead.)

- [ ] **Step 1.2: Run the example**

```bash
source ~/catkin_ws/devel/setup.bash
OMP_NUM_THREADS=4 roslaunch quatro quatro.launch
```

Wait until the console prints the dashed-line summary block and the `Total takes: ... sec.` line. Press Ctrl-C to exit the visualization loop.

- [ ] **Step 1.3: Capture the output transformation matrix**

The example currently prints info via `std::cout`, but does not directly print the final `output` matrix. Add a *temporary* one-line print just before the `pcl::transformPointCloud(*srcRaw, aligned, output);` call in `examples/run_global_registration.cpp`:

```cpp
std::cout << "[BASELINE] output=\n" << output << std::endl;
```

Rebuild, rerun (Steps 1.1, 1.2). Copy the printed 4x4 matrix into `materials/baseline_transform.txt`. **Then revert the temporary print** (`git checkout -- examples/run_global_registration.cpp`).

- [ ] **Step 1.4: Stage the baseline reference file**

```bash
git add materials/baseline_transform.txt
git status
```

Expected: only `materials/baseline_transform.txt` is staged.

- [ ] **Step 1.5: Commit**

```bash
git commit -m "Add baseline reference output for regression check"
```

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

- [ ] **Step 2.3: Verify it compiles in isolation**

```bash
cd ~/catkin_ws/src/Quatro
g++ -std=c++17 -fsyntax-only -Iinclude $(pkg-config --cflags-only-I pcl_common-1.10 2>/dev/null || pkg-config --cflags-only-I pcl_common 2>/dev/null) include/quatro/pcl_compat.hpp 2>&1 | head -40
```

Expected: empty output (clean compile). If `pkg-config` fails to find a version-suffixed name, the unsuffixed `pcl_common` should work. If both fail, fall back to relying on the catkin build in Step 2.4.

- [ ] **Step 2.4: Verify the catkin build still works (no regression)**

```bash
cd ~/catkin_ws
catkin build quatro
```

Expected: same outcome as the baseline build in Task 1. Nothing in the project includes the new file yet, so the build is unchanged.

- [ ] **Step 2.5: Commit**

```bash
git add include/quatro/pcl_compat.hpp
git commit -m "Add include/quatro/pcl_compat.hpp with PCL-version-agnostic voxelize"
```

---

## Task 3: Header hygiene in `include/quatro.hpp` and example caller update

**Why:** `quatro.hpp` has `using namespace std;` and `using namespace pcl;` (forbidden in headers — pollutes every translation unit), unused ROS/system includes, and an inline `voxelize` we just moved into `namespace quatro`. Removing the inline `voxelize` will break the example's two unqualified call sites (lines 206-207 of `run_global_registration.cpp`), so this task updates them in the same commit to keep the build green.

**Files:**
- Modify: `include/quatro.hpp` (lines 10-11, 45-46, 49-68)
- Modify: `examples/run_global_registration.cpp` (lines 206-207 — call `quatro::voxelize` instead of unqualified `voxelize`)

**Steps:**

- [ ] **Step 3.1: Remove unused system/ROS includes**

In `include/quatro.hpp`, delete these two lines (currently at lines 10-11):

```cpp
#include <unistd.h>
#include <geometry_msgs/Pose.h>
```

(Verification: search the rest of the file for `geometry_msgs` and any POSIX-only `unistd` symbol — there should be no occurrences. If you find any, stop and consult the spec.)

- [ ] **Step 3.2: Replace inline `voxelize` definitions with the new header**

Delete the two `voxelize` template definitions (currently lines 49-68 of `quatro.hpp` — both overloads, ending with `voxel_filter.filter(*dstPtr);  }`).

Add this include near the other Quatro-internal includes (just below `#include "conversion.hpp"`):

```cpp
#include "quatro/pcl_compat.hpp"
```

(There may also still be a `#include <pcl/filters/voxel_grid.h>` line in `quatro.hpp` — leave it alone; it's harmless and other code in the file may rely on it.)

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

- [ ] **Step 3.5: Restore `std::` and `pcl::` qualifications inside the file**

The simplest way: build, read the errors, qualify each symbol the compiler complains about. Common ones to expect (do them all proactively to save round-trips):

In `quatro.hpp`, qualify:
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
- `PointCloud<...>` → `pcl::PointCloud<...>` (where it appears bare, e.g., in `setInliers`'s parameter types — but NOT inside the class where `PointCloudSource`/`PointCloudTarget` are typedefed)
- `PointXYZ` → `pcl::PointXYZ`
- `transformPointCloud` → `pcl::transformPointCloud`
- `VoxelGrid` → `pcl::VoxelGrid` (if any references remain after removing the inline `voxelize`)

`PointType` is a project-wide typedef defined in `include/utility.h` — leave it as-is.

- [ ] **Step 3.6: Build**

```bash
cd ~/catkin_ws
catkin build quatro 2>&1 | tail -60
```

Expected: build succeeds. If it fails, the error will name the unqualified symbol — qualify it and rebuild. Repeat until clean.

- [ ] **Step 3.7: Run the example, confirm output matches baseline**

```bash
source ~/catkin_ws/devel/setup.bash
OMP_NUM_THREADS=4 roslaunch quatro quatro.launch
```

Re-add the temporary print from Task 1.3 if you removed it, run, compare the printed `output` matrix against `materials/baseline_transform.txt`. They MUST match (deterministic algorithm, fixed input).

- [ ] **Step 3.8: Commit**

```bash
git add include/quatro.hpp examples/run_global_registration.cpp
git commit -m "Clean up quatro.hpp: remove using-namespace, drop unused ROS/system includes, use pcl_compat.hpp; qualify voxelize callers"
```

---

## Task 4: Extract algorithm core into a private helper

**Why:** Two public entry points (the PCL virtual override in Task 5, and the legacy method in Task 6) need to share the same algorithm body. Right now the body lives inside `computeTransformation(Eigen::Matrix4d& output)` — extracting it lets both callers reuse it without duplication.

**Files:**
- Modify: `include/quatro.hpp` — extract method body, no behavior change

**Steps:**

- [ ] **Step 4.1: Add a private helper declaration**

In `include/quatro.hpp`, find the existing `private:` section near the bottom (currently at line 1059). Replace it with:

```cpp
private:
    /** \brief Algorithm core. Computes the registration transform in
     *  double precision and writes it into `output`. Sets `solution_.valid`.
     *  Does NOT touch `final_transformation_` or `converged_` — the public
     *  entry points are responsible for that.
     */
    void computeQuatroTransformation_(Eigen::Matrix4d& output);
```

- [ ] **Step 4.2: Move the existing body**

Find the existing definition (currently `void computeTransformation(Eigen::Matrix4d &output) {` near line 769). Rename the function to `computeQuatroTransformation_` AND move its definition to just below the class (or to the very end of the file, before the `#endif`). Because Quatro is a class template, the helper must remain in the header and be defined inline (e.g., at namespace scope as a member-function template definition):

```cpp
template <typename PointSource, typename PointTarget, typename Scalar>
void Quatro<PointSource, PointTarget, Scalar>::computeQuatroTransformation_(
        Eigen::Matrix4d& output) {
    // ... existing body verbatim ...
}
```

(Since `void computeTransformation(PointCloudSource &output, const Matrix4 &guess) override {};` already exists at line 767 as an empty stub, leave it for now — Task 5 will replace it.)

- [ ] **Step 4.3: Replace the original `computeTransformation(Eigen::Matrix4d&)` body with a delegating call**

Where the original body used to be, leave a thin wrapper for backward compatibility:

```cpp
void computeTransformation(Eigen::Matrix4d& output) {
    computeQuatroTransformation_(output);
}
```

(Task 6 will enrich this wrapper to also populate `final_transformation_`.)

- [ ] **Step 4.4: Build**

```bash
cd ~/catkin_ws
catkin build quatro 2>&1 | tail -30
```

Expected: clean build. If you see "redefinition of `computeTransformation`" errors, the empty stub at line 767 is colliding — leave it; the issue is the rename. Re-check Step 4.2.

- [ ] **Step 4.5: Run example, verify output matches baseline**

```bash
source ~/catkin_ws/devel/setup.bash
OMP_NUM_THREADS=4 roslaunch quatro quatro.launch
```

Re-add the temporary `output` print from Task 1.3 if needed. Compare the printed matrix to `materials/baseline_transform.txt` — must match exactly. Refactoring without behavior change is the criterion.

- [ ] **Step 4.6: Commit**

```bash
git add include/quatro.hpp
git commit -m "Extract Quatro algorithm core into computeQuatroTransformation_ helper"
```

---

## Task 5: Implement the PCL `Registration` virtual override (enables `align()`)

**Why:** `pcl::Registration::align(output)` calls `computeTransformation(PointCloudSource& output, const Matrix4& guess)` internally. The current empty-body override (`{};` at line 767) means `align()` is a no-op. We replace it with a real implementation that delegates to our core helper.

**Files:**
- Modify: `include/quatro.hpp`

**Steps:**

- [ ] **Step 5.1: Add `using` to expose both `computeTransformation` overloads**

In `include/quatro.hpp`, near the other `using Registration<...>::...` lines (currently around lines 102-104), add:

```cpp
using pcl::Registration<PointSource, PointTarget, Scalar>::final_transformation_;
using pcl::Registration<PointSource, PointTarget, Scalar>::converged_;
```

(`final_transformation_` may already be in the using list — search before adding. `converged_` likely is not. Both are protected members of `pcl::Registration`.)

Also add, to keep the legacy `computeTransformation(Eigen::Matrix4d&)` from hiding the parent's overload set:

```cpp
using pcl::Registration<PointSource, PointTarget, Scalar>::computeTransformation;
```

(This may already be present — search for it.)

- [ ] **Step 5.2: Replace the empty `computeTransformation(PointCloudSource&, const Matrix4&)` override**

Find:
```cpp
void computeTransformation(PointCloudSource &output, const Matrix4 &guess) override {};
```

Replace with:

```cpp
void computeTransformation(PointCloudSource& output, const Matrix4& guess) override {
    if (!guess.isIdentity()) {
        PCL_WARN("[%s] Quatro is a global registration method and ignores the "
                 "`guess` argument passed via align(...). Use "
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

- [ ] **Step 5.3: Build**

```bash
cd ~/catkin_ws
catkin build quatro 2>&1 | tail -30
```

Expected: clean build. Common errors and fixes:
- "`final_transformation_` was not declared" → Step 5.1 missed; add the `using` line.
- "`converged_` was not declared" → same fix.
- "`PCL_WARN` was not declared" → add `#include <pcl/console/print.h>` near the top of `quatro.hpp`.

- [ ] **Step 5.4: Smoke-test the new `align()` API**

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
    std::cout << "[ALIGN-API] output=\n" << via_align << std::endl;
}
```

Rebuild, rerun. The `[ALIGN-API]` matrix MUST match `materials/baseline_transform.txt`.

- [ ] **Step 5.5: Revert the temporary smoke test**

```bash
git checkout -- examples/run_global_registration.cpp
```

(The permanent example update happens in Task 7.)

- [ ] **Step 5.6: Commit**

```bash
git add include/quatro.hpp
git commit -m "Implement pcl::Registration virtual computeTransformation override (enables align())"
```

---

## Task 6: Wire legacy method to also set `final_transformation_`

**Why:** Spec section 4.2.3 — both API surfaces should leave the object in the same observable state, so `getFinalTransformation()` works regardless of which entry point the user calls. This is a costless improvement and a strict superset of the prior behavior.

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
cd ~/catkin_ws
catkin build quatro 2>&1 | tail -20
```

Expected: clean build.

- [ ] **Step 6.3: Run example, confirm output unchanged**

```bash
source ~/catkin_ws/devel/setup.bash
OMP_NUM_THREADS=4 roslaunch quatro quatro.launch
```

The example still uses the legacy `computeTransformation(Eigen::Matrix4d&)` API at this point. The `output` matrix it prints (use the temporary print from Task 1.3 if needed) MUST match the baseline.

- [ ] **Step 6.4: Commit**

```bash
git add include/quatro.hpp
git commit -m "Legacy computeTransformation(Eigen::Matrix4d&) also sets final_transformation_"
```

---

## Task 7: Update example to use the `align()` API

**Why:** Spec section 4.4 — show the PCL-idiomatic flow as the recommended usage. Keep the legacy call path documented in a comment so readers see both options.

**Files:**
- Modify: `examples/run_global_registration.cpp`

**Steps:**

- [ ] **Step 7.1: Replace the legacy call with `align()`**

In `examples/run_global_registration.cpp`, find lines 242-246:

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
cd ~/catkin_ws
catkin build quatro 2>&1 | tail -20
```

Expected: clean build.

- [ ] **Step 7.3: Run the example, confirm bit-identical output**

```bash
source ~/catkin_ws/devel/setup.bash
OMP_NUM_THREADS=4 roslaunch quatro quatro.launch
```

Add the temporary print from Task 1.3 if needed. The matrix MUST match `materials/baseline_transform.txt`.

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
cd ~/catkin_ws
catkin clean quatro -y
catkin build quatro 2>&1 | tail -30
```

Expected: clean build.

- [ ] **Step 8.4: Run example one final time, confirm baseline match**

```bash
source ~/catkin_ws/devel/setup.bash
OMP_NUM_THREADS=4 roslaunch quatro quatro.launch
```

The output matrix MUST still match `materials/baseline_transform.txt`.

- [ ] **Step 8.5: Remove the baseline reference file**

It served its purpose. Keeping a recorded numerical output in the repo invites confusion when the algorithm parameters legitimately change.

```bash
git rm materials/baseline_transform.txt
```

- [ ] **Step 8.6: Commit**

```bash
git add CMakeLists.txt
git commit -m "Bump PCL minimum to 1.10; fix \${PCL_LIBRARY_DIRS} -> \${PCL_LIBRARIES} typo; remove baseline file"
```

---

## Task 9: Final verification

**Why:** Sanity-check the whole refactor end-to-end before declaring done.

**Steps:**

- [ ] **Step 9.1: Read through `include/quatro.hpp` top-to-bottom**

Confirm with `grep`:

```bash
grep -n "using namespace" include/quatro.hpp        # expect: no output
grep -n "boost::shared_ptr" include/quatro.hpp      # expect: no output
grep -n "geometry_msgs" include/quatro.hpp          # expect: no output
grep -n "<unistd.h>" include/quatro.hpp             # expect: no output
```

If any of these print lines, fix them and amend.

- [ ] **Step 9.2: Confirm both API surfaces work in a single run**

Temporarily extend `examples/run_global_registration.cpp` to call BOTH APIs in sequence:

```cpp
// After the existing align() call, also call the legacy API and compare:
{
    Quatro<PointType, PointType> quatro_legacy;
    quatro_legacy.reset(params);
    quatro_legacy.setInputSource(srcMatched);
    quatro_legacy.setInputTarget(tgtMatched);
    Eigen::Matrix4d legacy_output;
    quatro_legacy.computeTransformation(legacy_output);
    std::cout << "[LEGACY-API] output=\n" << legacy_output << std::endl;
    std::cout << "[LEGACY-API] getFinalTransformation()=\n"
              << quatro_legacy.getFinalTransformation().template cast<double>()
              << std::endl;
}
```

Build, run. Verify:
- `[LEGACY-API] output` matches the baseline.
- `[LEGACY-API] getFinalTransformation()` matches `[LEGACY-API] output` exactly (Task 6 wired this).
- The earlier `align()`-style output (printed by the regular example flow) also matches.

Then revert: `git checkout -- examples/run_global_registration.cpp`.

- [ ] **Step 9.3: Final clean build**

```bash
cd ~/catkin_ws
catkin clean quatro -y
catkin build quatro 2>&1 | tail -10
```

Expected: clean build.

- [ ] **Step 9.4: Sanity-check commit log**

```bash
cd ~/catkin_ws/src/Quatro
git log --oneline main..HEAD
```

Expected: 8 commits, in the order produced by Tasks 1–8. No "WIP" or "fixup" commits.

If the implementation is in a worktree branch (per `superpowers:using-git-worktrees`), the branch is now ready to merge. If on `main` directly, no further git work needed.

---

## Out of scope reminder

The following were explicitly excluded by the spec and remain follow-up work:
- ROS-free build option (split algorithm core out of the catkin package)
- Install rules + CMake config files for `find_package(quatro)`
- Python bindings
- Removing ROS dependencies from `imageProjection.hpp`, `patchwork.hpp`, `fpfh_manager.hpp`

These are future iterations. Do not pull them into this plan.
