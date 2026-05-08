# Quatro PCL Compatibility Refactor — Design Spec

**Date:** 2026-05-09
**Status:** Draft, awaiting user review
**Reference:** [koide3/fast_gicp](https://github.com/koide3/fast_gicp)

## 1. Goal

Improve PCL compatibility of the Quatro repository so that:

1. The class can be used like any other PCL `Registration` derivative (e.g., `quatro.align(aligned)` followed by `quatro.getFinalTransformation()`).
2. The code builds cleanly across **PCL 1.10 (Ubuntu 20.04) through PCL 1.14 (Ubuntu 24.04)**, despite the smart pointer transition (`boost::shared_ptr` → `pcl::shared_ptr` / `std::shared_ptr`).
3. Existing call sites that use `quatro.computeTransformation(Eigen::Matrix4d&)` keep working (backward compatibility).

Out of scope for this iteration:
- Splitting the algorithm core out of the ROS catkin package into a standalone library.
- Installing CMake config files for `find_package(quatro)`.
- Python bindings.
- Removing `imageProjection.hpp` / `patchwork.hpp` ROS dependencies (these are used only by the example, not the algorithm core).

These are deferred to a follow-up iteration.

## 2. Non-Negotiable Constraints

- **Backward compatibility for the existing example.** `examples/run_global_registration.cpp` and any in-house consumer code that uses `quatro.computeTransformation(Eigen::Matrix4d&)` must continue to compile and produce equivalent numerical results.
- **No changes to algorithm semantics.** The numerical output of the GNC-TLS rotation solver, scale solver, COTE translation solver, and max clique selection must be unchanged.
- **No new external dependencies.** Stay within PCL, Eigen, Boost, OpenMP, and the existing `pmc` third-party.

## 3. Identified Issues (Current State)

| # | Issue | Location | Effect |
|---|-------|----------|--------|
| 1 | `boost::shared_ptr` hardcoded in function signatures | `quatro.hpp:51,62` | Breaks build on PCL 1.12+ when callers pass `pcl::PointCloud<T>::Ptr` (which is `std::shared_ptr` there). |
| 2 | `static pcl::VoxelGrid<T>` keeps state across calls | `quatro.hpp:53,64` | Re-using the same `voxelize` function with different inputs/leaf sizes inherits stale internal state. |
| 3 | `computeTransformation(Eigen::Matrix4d&)` does NOT override the PCL virtual | `quatro.hpp:769` | `pcl::Registration::align()` cannot be used. The PCL virtual `computeTransformation(PointCloudSource&, const Matrix4&)` is left as a no-op (line 767). |
| 4 | `Scalar` template parameter ignored | `quatro.hpp:71+` | Internally hardcodes `Eigen::Matrix4d`, `Eigen::Matrix3d`, etc., so `Quatro<Pt, Pt, float>` produces type mismatches with `final_transformation_` (which is `Matrix<Scalar,4,4>`). |
| 5 | `using namespace std;` and `using namespace pcl;` in a header | `quatro.hpp:45-46` | Pollutes the namespace of every translation unit that includes the header. |
| 6 | Unused ROS includes in algorithm header | `quatro.hpp:11` (`geometry_msgs/Pose.h`), `quatro.hpp:10` (`unistd.h`) | Forces ROS even where it would otherwise not be needed; misleading dependency. |
| 7 | `target_link_libraries(... ${PCL_LIBRARY_DIRS} ...)` | `CMakeLists.txt:78` | Wrong variable. Should be `${PCL_LIBRARIES}`. Currently links a *path string*, not the libraries. (The `catkin_LIBRARIES` link transitively pulls PCL in, masking the bug.) |
| 8 | `find_package(PCL 1.8 REQUIRED)` minimum version stated as 1.8 | `CMakeLists.txt:25` | Likely already broken on 1.8 due to other code; bumping to 1.10 reflects the supported range. |

## 4. Design

### 4.1 New file: `include/quatro/pcl_compat.hpp`

Lightweight compatibility header. Contains:

- A `pcl::PointCloud<T>::Ptr` typedef — already exists in PCL itself, so this header simply documents that **all in-tree code MUST use `pcl::PointCloud<T>::Ptr` (or `ConstPtr`) and never `boost::shared_ptr<pcl::PointCloud<T>>` directly**.
- The free-function `voxelize<T>(...)` moved here, with two overloads, both taking `typename pcl::PointCloud<T>::Ptr` and `typename pcl::PointCloud<T>::ConstPtr` as appropriate. The `static` qualifier on the internal `pcl::VoxelGrid<T>` instance is removed.
- Other small PCL utility functions if discovered during implementation.

This header is included by `quatro.hpp`. Existing client code that only includes `quatro.hpp` keeps working.

### 4.2 Refactor: `include/quatro.hpp`

#### 4.2.1 Header hygiene
- Remove `using namespace std;` and `using namespace pcl;`.
- Remove `#include <unistd.h>` and `#include <geometry_msgs/Pose.h>` (both unused).
- Inside the file, prefix with `std::` and `pcl::` everywhere they were implicit.
- Move the `voxelize` template to `pcl_compat.hpp` (see 4.1).

#### 4.2.2 PCL `Registration` virtual override (the big one)

Implement the actual PCL virtual:

```cpp
void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;
```

It will:
1. Call the existing core algorithm (extracted into a private helper `computeQuatroTransformation_(Eigen::Matrix4d&)`).
2. Cast the resulting `Eigen::Matrix4d` to `Matrix4` (`Eigen::Matrix<Scalar,4,4>`) and store it in `final_transformation_`.
3. Apply `pcl::transformPointCloud(*input_, output, final_transformation_)` to populate `output`.
4. Set `converged_ = solution_.valid`.

`guess` parameter handling: Quatro is a *global* registration method, so an arbitrary 4x4 initial guess from `align(out, guess)` is meaningless to the algorithm (it does not iterate from a prior). The override therefore **ignores `guess`** and emits a one-time PCL warning if `guess != Matrix4::Identity()`. Users who want to inject a rotational prior (e.g., from IMU) should keep using the existing `setPreEstaimatedRyRx()` entry point, which is already wired into the rotation solver.

This makes `quatro.align(aligned_cloud)` work exactly as it does for ICP / NDT / fast_gicp.

#### 4.2.3 Backward-compatible legacy method

Keep `void computeTransformation(Eigen::Matrix4d& output)` as a member function that:
1. Calls the extracted core helper `computeQuatroTransformation_(output)`.
2. Also assigns `final_transformation_ = output.template cast<Scalar>();` and `converged_ = solution_.valid;` — so `getFinalTransformation()` and `hasConverged()` work after either entry point. This is costless and keeps both APIs in sync.
3. Does NOT transform any `PointCloudSource` output (there is no output cloud parameter), preserving the existing call shape.

Existing callers — e.g., `examples/run_global_registration.cpp` — see no behavior change in `output` (numerically identical). Users who additionally probe `getFinalTransformation()` after a legacy call now get a meaningful matrix, which is a strict improvement.

> **Naming note:** The legacy method shadows the parent class's virtual on the same name with a different signature. C++ allows this, but it suppresses overload resolution from the base. We will add an explicit `using Registration<...>::computeTransformation;` to avoid hiding the inherited overload.

#### 4.2.4 Algorithm core extraction

Refactor the existing 167-line body of `computeTransformation(Eigen::Matrix4d&)` into:

```cpp
private:
    void computeQuatroTransformation_(Eigen::Matrix4d& output);
```

Both the new PCL virtual and the legacy method delegate to this.

#### 4.2.5 `Scalar` template parameter — actually use it

- All input/output public-API types use `Matrix4 = Eigen::Matrix<Scalar,4,4>` (the typedef already declared at `quatro.hpp:108`).
- Internal solver math stays in `double` for numerical stability (mirrors fast_gicp's approach).
- Cast at the boundary: at the end of the core, `final_transformation_ = output.template cast<Scalar>();`.
- Effect: `Quatro<pcl::PointXYZ, pcl::PointXYZ, float>` works (this is the PCL default Scalar).

### 4.3 `CMakeLists.txt` edits (minimal)

- Bump PCL minimum: `find_package(PCL 1.10 REQUIRED)`.
- Fix the link line:
  ```cmake
  target_link_libraries(run_example
      PUBLIC
      ${PCL_LIBRARIES}     # was ${PCL_LIBRARY_DIRS}
      ${catkin_LIBRARIES}
      stdc++fs
      pmc::pmc
  )
  ```
- No `add_library`, no install rules — that is deferred (Section 1, out of scope).

### 4.4 `examples/run_global_registration.cpp` edits

Replace:
```cpp
Eigen::Matrix4d output;
quatro.computeTransformation(output);
```
with the new PCL-idiomatic flow:
```cpp
pcl::PointCloud<PointType> aligned_dummy;  // not used downstream; kept for align() contract
quatro.align(aligned_dummy);
Eigen::Matrix4d output = quatro.getFinalTransformation().template cast<double>();
```

Then keep the rest of the file untouched (the existing `pcl::transformPointCloud(*srcRaw, aligned, output)` call uses the `output` matrix, not `aligned_dummy`).

This demonstrates the new API. Document the legacy entry point in a comment as still supported.

## 5. File-by-file Change Map

| File | Change |
|------|--------|
| `include/quatro/pcl_compat.hpp` | NEW. `voxelize<T>` overloads (no `static` filter), guidance on `pcl::PointCloud<T>::Ptr`. |
| `include/quatro.hpp` | Remove using-directives. Remove unused includes. Move `voxelize`. Add PCL `Registration` virtual override. Extract algorithm core. Wire `Scalar` template through. Add `using Registration<...>::computeTransformation;`. |
| `include/conversion.hpp` | Touch only if `using namespace` removal forces explicit `pcl::` prefix in callers. (Likely unchanged — already mostly explicit.) |
| `examples/run_global_registration.cpp` | Switch to `align()` API. |
| `CMakeLists.txt` | PCL min version bump 1.8 → 1.10. Fix `${PCL_LIBRARY_DIRS}` → `${PCL_LIBRARIES}`. |

Files not touched: `include/fpfh_manager.hpp`, `include/imageProjection.hpp`, `include/patchwork.hpp`, `include/utility.h`, `include/teaser/*`, `include/teaser_utils/*`, `src/**`, `package.xml`, `3rdparty/**`.

## 6. Verification Plan

After implementation, verify in this order:

1. **Compile** with the existing catkin build on the developer's machine (PCL 1.10 / 18.04 melodic per README).
2. **Run** `roslaunch quatro quatro.launch` against the bundled `000540.bin` / `001319.bin` toy pair, confirm the printed transformation and inlier counts match a baseline captured before refactor (within bit-exact tolerance — the algorithm is deterministic given fixed input).
3. **Sanity build (manual)** with PCL 1.12+ if the developer has access, OR confirm via a static read-through that no `boost::shared_ptr` remains in function signatures of the touched files.
4. **API smoke test**: write a 10-line consumer that calls both:
   - `quatro.computeTransformation(Eigen::Matrix4d&)` (legacy)
   - `quatro.align(cloud)` + `quatro.getFinalTransformation()` (new)
   and confirm both produce equivalent transforms.

## 7. Risks and Mitigations

| Risk | Likelihood | Mitigation |
|------|------------|------------|
| PCL `Registration::align()` does pre/post processing (e.g., setting `final_transformation_`) that conflicts with what we set ourselves. | Medium | Read PCL `Registration` source; ensure our override sets `final_transformation_` exactly as ICP does, and that `converged_` is set before return. Test against the toy data to confirm `getFinalTransformation()` returns what we expect. |
| `Scalar = float` triggers a numerical regression we don't catch with the `Scalar = double` toy test. | Low | Keep the example using `double`. Add a one-line static_assert-free template instantiation `Quatro<pcl::PointXYZ, pcl::PointXYZ, float>` in a comment to document tested coverage. Full float testing is a follow-up. |
| Removing `static` from `pcl::VoxelGrid<T>` slows things down (re-allocation per call). | Very low | `pcl::VoxelGrid` construction is cheap; the `static` was a misuse, not an optimization. |
| The PCL `Registration` virtual override unexpectedly hides our legacy `computeTransformation(Eigen::Matrix4d&)`. | Low | Add explicit `using Registration<...>::computeTransformation;` and verify with a compile that both overloads resolve. |

## 8. Acceptance Criteria

- `examples/run_global_registration.cpp` compiles and produces the same `output` matrix (to within deterministic equality) as before refactor.
- `quatro.align(aligned)` works and `quatro.getFinalTransformation()` returns the correct `Matrix4`.
- Header `quatro.hpp` contains zero `using namespace ...;` directives.
- Header `quatro.hpp` does not include `geometry_msgs/Pose.h` or `unistd.h`.
- No function signature in `include/quatro.hpp` or `include/quatro/pcl_compat.hpp` mentions `boost::shared_ptr`.
- `CMakeLists.txt` requires PCL ≥ 1.10 and links `${PCL_LIBRARIES}`, not `${PCL_LIBRARY_DIRS}`.
